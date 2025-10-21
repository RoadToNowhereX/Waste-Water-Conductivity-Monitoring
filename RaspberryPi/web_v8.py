# server.py
import argparse, threading, time, struct, socket
from typing import Optional
import serial
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, PlainTextResponse
import uvicorn
import json
import math
import asyncio

DEFAULT_SERIAL_PORT = "COM5"
# DEFAULT_SERIAL_PORT = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF485251667187111343-if02" # "/dev/ttyACM0"
# ========= Protocol (aligned with STM32) =========
FRAME_HDR = b"\xAD\xDA\x12\x34"
HDR_LEN = len(FRAME_HDR)
# Payload structure: u32 seq, u32 fs_per_ch, u16 win_pairs, u16 method_code, 6×float = 36 bytes, little-endian
PAYLOAD_FMT  = "<IIHH6f"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)  # 36

# ========= Global State (only the latest is kept) =========
latest_pkt = None
latest_seq = -1
pkt_lock = threading.Lock()
stop_evt = threading.Event()

def serial_reader_worker(port: str, baud: int):
    """Background serial reader thread: Reads serial port in real-time, decodes frames, and updates latest_pkt (discards old packets, not new ones)"""
    global latest_pkt, latest_seq

    try:
        ser = serial.Serial(port, baud, timeout=0)
        try:
            ser.reset_input_buffer()
        except:
            pass
        print(f"[INFO] Serial port opened successfully: {ser.port} @ {ser.baudrate}")
    except Exception as e:
        print(f"[ERR] Failed to open serial port: {e}")
        stop_evt.set()
        return

    buf = bytearray()
    H = HDR_LEN

    try:
        while not stop_evt.is_set():
            n_wait = ser.in_waiting
            chunk = ser.read(n_wait if n_wait > 0 else 1024)
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.002)

            # Parse multiple frames
            while True:
                idx = buf.find(FRAME_HDR)
                if idx < 0:
                    if len(buf) > (H - 1):
                        del buf[:-(H - 1)]
                    break

                if len(buf) < idx + H + 2:
                    break

                payload_len = buf[idx + H] | (buf[idx + H + 1] << 8)
                total_len = H + 2 + payload_len + 2
                if len(buf) < idx + total_len:
                    break

                frame = buf[idx:idx + total_len]
                payload = frame[H + 2 : H + 2 + payload_len]
                csum_calc = (sum(frame[H : H + 2 + payload_len]) & 0xFFFF)
                csum_recv = frame[H + 2 + payload_len] | (frame[H + 2 + payload_len + 1] << 8)

                # Checksum or length mismatch -> skip to the next HDR
                if csum_calc != csum_recv or payload_len != PAYLOAD_SIZE:
                    del buf[:idx + H]
                    continue

                # Unpack (consistent with STM32: 6 floats)
                (seq, fs_per_ch, win_pairs, method_code,
                 vpp_exc, vpp_adc, lo0, hi0, lo1, hi1) = struct.unpack(PAYLOAD_FMT, payload)

                pkt = {
                    "t": time.time(),
                    "seq": int(seq),
                    "fs": int(fs_per_ch),
                    "win": int(win_pairs),
                    "method": "Edge" if int(method_code) == 1 else "MinMax",
                    "vpp_exc": float(vpp_exc),
                    "vpp_adc": float(vpp_adc),
                    "lo0": float(lo0), "hi0": float(hi0),
                    "lo1": float(lo1), "hi1": float(hi1),
                }
                with pkt_lock:
                    latest_pkt = pkt
                    latest_seq = pkt["seq"]

                del buf[:idx + total_len]

    except serial.SerialException as e:
        print(f"[ERR] Serial port exception: {e}")
    except Exception as e:
        print(f"[ERR] Unknown exception: {e}")
    finally:
        try:
            ser.close()
        except:
            pass
        print("[INFO] Reader thread exited.")

# ========= FastAPI =========
app = FastAPI()

INDEX_HTML = """<!doctype html>
<html lang="en" data-theme="dark">
<head>
<meta charset="utf-8">
<title>Conductivity Monitoring (LAN)</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  :root{
    --bg:#0f172a;        /* Background */
    --text:#e2e8f0;      /* Text */
    --card:#111827;      /* Card Background */
    --border:#1f2937;    /* Border */
    --hint:#94a3b8;      /* Hint Text */
    --inputbg:#0b1220;   /* Input Background */
    --link:#7dd3fc;
    --axis:#475569;      /* Axis Line */
    --grid:#1f2937;      /* Grid Line */
    --primary:#22c55e;
    --secondary:#0ea5e9;
    --danger:#ef4444;
  }
  [data-theme="light"]{
    --bg:#f8fafc;
    --text:#0f172a;
    --card:#ffffff;
    --border:#e2e8f0;
    --hint:#475569;
    --inputbg:#ffffff;
    --link:#0ea5e9;
    --axis:#64748b;
    --grid:#e2e8f0;
    --primary:#16a34a;
    --secondary:#0284c7;
    --danger:#dc2626;
  }

  html, body { height:100%; margin:0; font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial; background:var(--bg); color:var(--text); }
  .wrap { max-width:1200px; margin:12px auto; padding:0 12px; }
  .cards { display:grid; grid-template-columns:1fr; gap:12px; }
  .card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:12px; }
  .title { font-weight:600; margin:0 0 8px 0; font-size:16px; color:var(--text); display:flex; align-items:center; justify-content:space-between; gap:8px;}
  .grid2 { display:grid; grid-template-columns:1fr 1fr; gap:12px; }
  .grid3 { display:grid; grid-template-columns:repeat(3, 1fr); gap:12px; }
  .row { margin-top:12px; font-size:14px; line-height:1.5; color:var(--text); }
  .kv { display:grid; grid-template-columns:180px 1fr; gap:8px; }
  .badge { display:inline-block; padding:2px 8px; border-radius:999px; background:var(--secondary); color:white; font-size:12px; }
  a { color:var(--link); text-decoration:none; }
  canvas { width:100%; height:320px; }
  #footer { color:var(--hint); font-size:12px; margin-top:10px; text-align:right; }
  input[type="text"], input[type="number"]{
    width:100%; box-sizing:border-box; padding:8px 10px; border-radius:10px; border:1px solid var(--border); background:var(--inputbg); color:var(--text);
  }
  .btn { background:var(--primary); color:white; border:none; padding:8px 12px; border-radius:10px; cursor:pointer; font-weight:600;}
  .btn:active{ transform:translateY(1px); }
  .btn-secondary { background:var(--secondary); color:white; }
  .seg { display:inline-flex; border:1px solid var(--border); border-radius:999px; overflow:hidden; }
  .seg button{ background:var(--inputbg); color:var(--hint); border:none; padding:6px 12px; cursor:pointer; }
  .seg button.active{ background:var(--primary); color:white; }
  .hint { color:var(--hint); font-size:12px; }
  .topbar { display:flex; align-items:center; justify-content:space-between; gap:12px; flex-wrap:wrap; }
  .right-tools { display:flex; align-items:center; gap:8px; }
  .alert { display:none; margin:8px 0; padding:10px 12px; border-radius:10px; background:var(--danger); color:#fff; font-weight:600; }
</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <h2 style="margin:6px 0 12px 0;">Conductivity Monitoring (LAN) <span id="stat" class="badge">Connecting...</span></h2>
    <div class="right-tools">
      <div class="seg" id="themeSeg">
        <button id="btnDark" class="active">Dark</button>
        <button id="btnLight">Light</button>
      </div>
      <button id="btnVppToggle" class="btn btn-secondary" type="button">Show Voltage Chart</button>
    </div>
  </div>

  <div id="alertBox" class="alert"></div>

  <div class="card">
    <div class="title">
      <span>Mode & Parameters</span>
      <span class="hint">(Units: R_up/R_down in Ω; d in cm; S in cm²; DiffGain, k_c, γ, β are unitless; κ → µS/cm@25°C)</span>
    </div>
    <div style="display:flex; align-items:center; gap:12px; flex-wrap:wrap;">
      <div class="seg" id="modeSeg">
        <button id="btnRun" class="active">Run</button>
        <button id="btnCal">Calibrate</button>
      </div>
      <!-- Calibrate sub-mode switch -->
      <div class="seg" id="calSubSeg" style="display:none;">
        <button id="btnCalRes" class="active">Resistance</button>
        <button id="btnCalCell">Electrode Constant</button>
      </div>
      <div id="applyMsg" class="hint"></div>
    </div>
    <div style="height:8px;"></div>
    <div class="grid3">
      <div>
        <div class="hint">R_up (Ω)</div>
        <input id="inpRup" type="number" step="1" value="1500">
      </div>
      <div>
        <div class="hint">R_down (Ω)</div>
        <input id="inpRdown" type="number" step="1" value="100">
      </div>
      <div>
        <div class="hint">DiffGain</div>
        <input id="inpGain" type="number" step="0.001" value="1.2">
      </div>
      <div>
        <div class="hint">d (electrode spacing, cm)</div>
        <input id="inpD" type="number" step="0.01" value="1.00">
      </div>
      <div>
        <div class="hint">S (electrode area, cm²)</div>
        <input id="inpS" type="number" step="0.01" value="1.00">
      </div>

      <!-- Run + Cal(Resistance): keep k_c input -->
      <div id="corrInputWrap">
        <div class="hint">Electrode Constant Correction k_c (×)</div>
        <input id="inpKcorr" type="number" step="0.001" value="1.000">
      </div>

      <!-- === γ & β for linear regression correction (Run + Cal/Resistance; Cal/Cell 可选显示) === -->
      <div id="inpGammaWrap">
        <div class="hint">γ (gamma: gain for y)</div>
        <input id="inpGamma" type="number" step="0.0000001" value="1.00050256">
      </div>
      <div id="inpBetaWrap">
        <div class="hint">β (beta: offset for y)</div>
        <input id="inpBeta" type="number" step="0.0000001" value="-0.002214615">
      </div>

      <!-- Calibrate(Resistance) only -->
      <div id="calOnlyRes" style="display:none;">
        <div class="hint">Rx_actual (Ω) — Calibrate: Resistance</div>
        <input id="inpRxAct" type="number" step="0.1" value="1000">
      </div>

      <!-- Calibrate(Cell) only -->
      <div id="calOnlyCell" style="display:none;">
        <div class="hint">Conductivity_actual (µS/cm) — Calibrate: Electrode Constant</div>
        <input id="inpKappaAct" type="number" step="1" placeholder="e.g., 1413">
      </div>

      <!-- Calibrate(Cell) only · γ/β 开关（默认不叠加=解耦） -->
      <div id="cellGyWrap" style="display:none;">
        <label class="hint" style="display:flex;gap:8px;align-items:center;">
          <input id="chkApplyGyInCell" type="checkbox">
          Apply γ/β in Cell calibration
        </label>
      </div>

      <!-- Adopt correction -->
      <div id="adoptWrap" style="display:none; grid-column: 1 / -1;">
        <button id="btnAdoptKc" class="btn">Adopt k_c(real) → Run</button>
        <span class="hint" style="margin-left:8px;">Apply K_c and switch to Run mode.</span>
      </div>

      <!-- Run only -->
      <div id="runOnly">
        <div class="hint">
            κ Upper Threshold (µS/cm) — Run Mode
            <span style="opacity:0.8; font-weight:normal;">(Leave blank for none)</span>
        </div>
        <input id="inpKappaMax" type="number" step="1" placeholder="e.g., 3000">
      </div>
    </div>
    <div style="height:10px;"></div>
    <button id="btnApply" class="btn">Apply</button>
  </div>

  <!-- Top: Conductivity -->
  <div class="card">
    <div class="title">
      <span id="rightTitle">Conductivity κ (µS/cm@25°C): —</span>
    </div>
    <div id="rightChart" style="width:100%;height:420px;"></div>
  </div>

  <!-- Bottom: Voltage (collapsed by default) -->
  <div class="card" id="cardVpp" style="display:none;">
    <div class="title">
      <span>Vpp (A1=Excitation, A0=ADC Measurement)</span>
    </div>
    <div id="vpp" style="width:100%;height:360px;"></div>
  </div>

  <div class="card row">
    <div class="title"><span>Current Frame Parameters / Readings</span></div>
    <div class="kv">
      <div>Sampling Rate (Hz) / Window Pairs</div><div id="kv1">-</div>
      <div>A0 Detection Method</div><div id="kv2">-</div>
      <div>Vpp_exc / Vpp_ADC (V)</div><div id="kv3">-</div>
      <div id="kvRunLbl">Rx (Ω) / κ (µS/cm@25°C)</div><div id="kvRunVal">-</div>
      <div id="kvCalLbl" style="display:none;">Cal (%): est / ref / error</div><div id="kvCalVal" style="display:none;">-</div>
      <div id="kvCellLbl" style="display:none;">K_real (cm⁻¹) / k_c(real) (×)</div><div id="kvCellVal" style="display:none;">-</div>
      <div>A0 lo/hi (V)</div><div id="kv4">-</div>
      <div>A1 lo/hi (V)</div><div id="kv5">-</div>
    </div>
  </div>

  <div id="footer">Real-time WebSocket: Displays recent window data; x-axis scrolls continuously.</div>
</div>

<!-- ECharts CDN -->
<script src="https://cdn.jsdelivr.net/npm/echarts@5/dist/echarts.min.js"></script>
<script>
(function(){
  // ======== Theme Utilities ========
  function cssVar(name){ return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
  function themeColors(){
    return {
      axis: cssVar('--axis'),
      grid: cssVar('--grid'),
      text: cssVar('--text'),
      hint: cssVar('--hint'),
      danger: cssVar('--danger')
    };
  }
  function setTheme(mode){
    const root = document.documentElement;
    if(mode === 'light'){ root.setAttribute('data-theme','light'); }
    else { root.setAttribute('data-theme','dark'); }
    localStorage.setItem('adc_theme', mode);
    refreshTheme(); // Update chart colors
  }
  function initThemeFromPref(){
    const saved = localStorage.getItem('adc_theme');
    if(saved === 'light' || saved === 'dark') return saved;
    return window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark';
  }

  // ======== Parameters/Mode ========
  let mode = 'run'; // 'run' | 'cal'
  let calMode = 'res'; // 'res' (Resistance) | 'cell' (Electrode Constant)
  const params = {
    Rup: 1500,
    Rdown: 100,
    Gain: 1.2,
    d: 1.00,        // cm
    S: 1.00,        // cm^2
    kcorr: 1.000,   // electrode constant correction (×)
    // Linear regression correction for y = Vpp_ADC/(Vpp_exc*Gain)
    gamma: 1.00050256,
    beta: -0.002214615,
    // Cell 模式是否叠加 γ/β（默认 false = 解耦）
    applyGyInCell: false,

    Rx_actual: 1000,          // for Cal-Resistance
    kappa_actual_uS: NaN,     // for Cal-Cell (µS/cm)
    kappaMax: NaN             // Run threshold (µS/cm)
  };

  // ======== UI Elements ========
  const statEl = document.getElementById('stat');
  const btnRun = document.getElementById('btnRun');
  const btnCal = document.getElementById('btnCal');
  const calSubSeg = document.getElementById('calSubSeg');
  const btnCalRes = document.getElementById('btnCalRes');
  const btnCalCell = document.getElementById('btnCalCell');

  const calOnlyRes = document.getElementById('calOnlyRes');
  const calOnlyCell = document.getElementById('calOnlyCell');
  const runOnly = document.getElementById('runOnly');

  const btnApply = document.getElementById('btnApply');
  const applyMsg = document.getElementById('applyMsg');

  const inpRup = document.getElementById('inpRup');
  const inpRdown = document.getElementById('inpRdown');
  const inpGain = document.getElementById('inpGain');
  const inpD = document.getElementById('inpD');
  const inpS = document.getElementById('inpS');
  const inpKcorr = document.getElementById('inpKcorr');

  const inpGamma = document.getElementById('inpGamma');
  const inpBeta  = document.getElementById('inpBeta');
  const inpGammaWrap = document.getElementById('inpGammaWrap');
  const inpBetaWrap  = document.getElementById('inpBetaWrap');

  const inpRxAct = document.getElementById('inpRxAct');
  const inpKappaAct = document.getElementById('inpKappaAct');
  const inpKappaMax = document.getElementById('inpKappaMax');
  const corrInputWrap = document.getElementById('corrInputWrap');

  const rightTitle = document.getElementById('rightTitle');
  const alertBox = document.getElementById('alertBox');

  const kv1 = document.getElementById('kv1');
  const kv2 = document.getElementById('kv2');
  const kv3 = document.getElementById('kv3');
  const kv4 = document.getElementById('kv4');
  const kv5 = document.getElementById('kv5');
  const kvRunLbl = document.getElementById('kvRunLbl');
  const kvRunVal = document.getElementById('kvRunVal');
  const kvCalLbl = document.getElementById('kvCalLbl');
  const kvCalVal = document.getElementById('kvCalVal');
  const kvCellLbl = document.getElementById('kvCellLbl');
  const kvCellVal = document.getElementById('kvCellVal');

  const btnDark = document.getElementById('btnDark');
  const btnLight = document.getElementById('btnLight');

  // Adopt correction button elements and cache values
  const adoptWrap = document.getElementById('adoptWrap');
  const btnAdoptKc = document.getElementById('btnAdoptKc');
  let lastKcReal = NaN;

  // Vpp toggle switch
  const cardVpp = document.getElementById('cardVpp');
  const btnVppToggle = document.getElementById('btnVppToggle');
  let vppVisible = false; // Collapsed by default

  // Cal/Cell γβ 叠加复选框
  const cellGyWrap = document.getElementById('cellGyWrap');
  const chkApplyGyInCell = document.getElementById('chkApplyGyInCell');

  // 绑定复选框（页面加载一次即可；避免在 setMode/setCalSubMode 里重复绑定）
  if (chkApplyGyInCell){
    chkApplyGyInCell.onchange = ()=>{
      params.applyGyInCell = !!chkApplyGyInCell.checked;
      // 勾上时显示 γ/β，取消时隐藏（仅在 Cell 子模式下处理可见性）
      if (mode==='cal' && calMode==='cell'){
        setGammaBetaVisible(params.applyGyInCell);
      }
    };
  }

  // ======== Tick/axis styling and formatting ========
  function fmtNum(n, frac=1){
    if (!Number.isFinite(n)) return '—';
    return n.toLocaleString(undefined, { minimumFractionDigits: frac, maximumFractionDigits: frac });
  }
  function fmtTick(v){
    const a = Math.abs(v);
    if (a >= 100) return v.toFixed(0);
    if (a >= 10)  return v.toFixed(1);
    return v.toFixed(2);
  }
  function fmtKappa(v){ // µS/cm
    return fmtNum(v, (Math.abs(v) >= 100 ? 0 : 1));
  }
  function fmtPct(v){   // %
    return v.toFixed(3);
  }

  function axisStyle(){
    const th = themeColors();
    return {
      axisLine:{ lineStyle:{ color: th.axis } },
      splitLine:{ lineStyle:{ color: th.grid } }
    };
  }

  function axisX(label){
    const th = themeColors();
    return {
      type:'value',
      name:label,
      splitNumber: 6,
      axisLabel:{ color: th.text, formatter:(v)=>fmtTick(v), hideOverlap:true },
      ...axisStyle()
    };
  }

  function axisY(label, minv, maxv, forceInt=false, kind='plain'){
    const th = themeColors();
    const base = {
      type:'value', name:label, min:minv, max:maxv,
      axisLabel:{ color: th.text },
      ...axisStyle()
    };
    if (forceInt){
      base.interval = 1;
      base.axisLabel.formatter = (v)=> Math.round(v);
    } else if (kind === 'kappa'){
      base.axisLabel.formatter = (v)=> fmtKappa(v);
    } else if (kind === 'pct'){
      base.axisLabel.formatter = (v)=> fmtPct(v);
    }
    return base;
  }

  // ======== Chart instances ========
  const chartVpp = echarts.init(document.getElementById('vpp'));
  const rightChart = echarts.init(document.getElementById('rightChart'));

  const VPP_WINDOW_S = 20;     // Last N seconds
  const FPS = 30;              // Frontend refresh throttle

  function initVppChart(){
    const th = themeColors();
    chartVpp.setOption({
      animation:false,
      legend:{ data:['A1 Vpp_exc','A0 Vpp_ADC'], textStyle:{color:th.text} },
      grid:{left:50,right:50,top:32,bottom:32},
      xAxis: axisX('t (s)'),
      yAxis: axisY('V', 0, 4, true),
      series:[
        {name:'A1 Vpp_exc', type:'line', showSymbol:false, data:[]},
        {name:'A0 Vpp_ADC', type:'line', showSymbol:false, data:[]}
      ]
    }, true);
  }

  function initRightRun(){
    const th = themeColors();
    rightChart.clear();
    rightChart.setOption({
      animation:false,
      legend:{ data:['κ (µS/cm@25°C)'], textStyle:{color:th.text} },
      grid:{left:50,right:64,top:32,bottom:32},
      xAxis: axisX('t (s)'),
      yAxis: axisY('µS/cm@25°C', 0, 4000, false, 'kappa'),
      series:[{id:'series_kappa', name:'κ (µS/cm@25°C)', type:'line', showSymbol:false, data:[]}]
    }, true);
  }
  function initRightCal(){
    const th = themeColors();
    rightChart.clear();
    rightChart.setOption({
      animation:false,
      legend:{ data:['Error %'], textStyle:{color:th.text} },
      grid:{left:50,right:64,top:32,bottom:32},
      xAxis: axisX('t (s)'),
      yAxis: axisY('%', 0, 10, false, 'pct'),
      series:[{ name:'Error %', type:'line', showSymbol:false, data:[] }]
    }, true);
  }

  // Data buffers
  let t0 = performance.now()/1000;
  const vpp_exc_data = [];
  const vpp_adc_data = [];
  const condData = []; // κ (µS/cm)
  const errData  = []; // % (for either R error or κ error depending on sub-mode)
  function pushPoint(arr, x, y){ arr.push([x,y]); }
  function trimWindow(){
    const t = performance.now()/1000 - t0;
    function trim(arr){ while(arr.length>0 && t - arr[0][0] > VPP_WINDOW_S) arr.shift(); }
    trim(vpp_exc_data); trim(vpp_adc_data); trim(condData); trim(errData);
  }
  function seriesMax(arr){ let m=0; for(const p of arr) if(p[1]>m) m=p[1]; return m; }

  // ======== K helpers ========
  function K_nominal(d_cm, S_cm2){ // cm^-1
    return (isFinite(d_cm)&&d_cm>0&&isFinite(S_cm2)&&S_cm2>0) ? (d_cm / S_cm2) : NaN;
  }

  // ======== Mode switching & visibility of γ/β ========
  function setGammaBetaVisible(vis){
    inpGammaWrap.style.display = vis ? '' : 'none';
    inpBetaWrap.style.display  = vis ? '' : 'none';
  }

  function setCalSubMode(m){
    calMode = m; // 'res' | 'cell'
    if (calMode === 'res'){
      btnCalRes.classList.add('active'); btnCalCell.classList.remove('active');
      calOnlyRes.style.display = '';
      calOnlyCell.style.display = 'none';
      corrInputWrap.style.display = ''; // keep k_c input
      kvCellLbl.style.display = 'none'; kvCellVal.style.display = 'none';
      adoptWrap.style.display = 'none';

      // res 模式：隐藏复选框；总是显示 γ/β
      if (cellGyWrap) cellGyWrap.style.display = 'none';
      setGammaBetaVisible(true);

      // 清空校准曲线，避免切换残留
      errData.length = 0;
      if (mode === 'cal') initRightCal();
    }else{
      btnCalCell.classList.add('active'); btnCalRes.classList.remove('active');
      calOnlyRes.style.display = 'none';
      calOnlyCell.style.display = '';
      corrInputWrap.style.display = 'none'; // hide k_c input
      kvCellLbl.style.display = ''; kvCellVal.style.display = '';
      adoptWrap.style.display = 'block';

      // cell 模式：显示复选框，并根据开关显示/隐藏 γ/β
      if (cellGyWrap) cellGyWrap.style.display = '';
      if (chkApplyGyInCell){
        chkApplyGyInCell.checked = !!params.applyGyInCell;
      }
      setGammaBetaVisible(!!params.applyGyInCell);

      // 清空校准曲线
      errData.length = 0;
      if (mode === 'cal') initRightCal();
    }
  }

  function setMode(m){
    mode = m;
    const th = themeColors();
    if(m === 'run'){
      btnRun.classList.add('active'); btnCal.classList.remove('active');
      calSubSeg.style.display = 'none';
      calOnlyRes.style.display = 'none';
      calOnlyCell.style.display = 'none';
      corrInputWrap.style.display = '';
      setGammaBetaVisible(true);      // Run 模式显示 γ/β
      runOnly.style.display = '';
      if (typeof adoptWrap !== 'undefined' && adoptWrap) adoptWrap.style.display = 'none';  // 隐藏“Adopt k_c(real)”按钮
      kvRunLbl.style.display = ''; kvRunVal.style.display = '';
      kvCalLbl.style.display = 'none'; kvCalVal.style.display = 'none';
      kvCellLbl.style.display = 'none'; kvCellVal.style.display = 'none';
      if (cellGyWrap) cellGyWrap.style.display = 'none';

      initRightRun();
      rightTitle.textContent = 'Conductivity κ (µS/cm@25°C): —';
      rightTitle.style.color = th.text;
      alertBox.style.display = 'none';
      condData.length = 0;
    }else{
      btnCal.classList.add('active'); btnRun.classList.remove('active');
      calSubSeg.style.display = '';
      runOnly.style.display = 'none';
      kvRunLbl.style.display = 'none'; kvRunVal.style.display = 'none';
      kvCalLbl.style.display = ''; kvCalVal.style.display = '';
      // 子模式决定是否显示 kcorr / γβ 或 K_real/kc(real)
      setCalSubMode(calMode);
      initRightCal();
      rightTitle.textContent = 'Error (%): —';
      rightTitle.style.color = th.text;
      alertBox.style.display = 'none';
      errData.length = 0;
    }
  }
  btnRun.onclick = ()=>setMode('run');
  btnCal.onclick = ()=>setMode('cal');
  btnCalRes.onclick = ()=>setCalSubMode('res');
  btnCalCell.onclick = ()=>setCalSubMode('cell');

  // -- Threshold markLine refresh/clear (no residue)
  function refreshThresholdMarkLine(){
    if(mode !== 'run') return;
    const th = themeColors();
    const hasThr = Number.isFinite(params.kappaMax) && params.kappaMax > 0;
    rightChart.setOption({
      series: [{
        id:'series_kappa',
        type:'line',
        data: condData,
        showSymbol:false,
        markLine: hasThr ? {
          symbol:'none',
          silent:true,
          label:{ color: th.text, formatter:'Threshold', position:'insideEndTop', distance:6 },
          lineStyle:{ type:'dashed', color: th.danger },
          data:[{ yAxis: params.kappaMax }]
        } : { data: [] }
      }]
    });
    if(!hasThr){
      rightTitle.style.color = themeColors().text;
      alertBox.style.display = 'none';
    }
  }

  // Apply parameters
  btnApply.onclick = ()=>{
    const vRup   = parseFloat(inpRup.value);
    const vRdown = parseFloat(inpRdown.value);
    const vGain  = parseFloat(inpGain.value);
    const vD     = parseFloat(inpD.value);
    const vS     = parseFloat(inpS.value);
    const vKc    = parseFloat(inpKcorr.value);

    const vGamma = parseFloat(inpGamma.value);
    const vBeta  = parseFloat(inpBeta.value);

    const vRxA   = parseFloat(inpRxAct.value);
    const vKAct  = parseFloat(inpKappaAct.value);
    const vKMax  = parseFloat(inpKappaMax.value);

    const vApplyGyCell = !!(chkApplyGyInCell && chkApplyGyInCell.checked);

    const baseOk = isFinite(vRup)&&vRup>0 && isFinite(vRdown)&&vRdown>0 &&
               isFinite(vGain)&&vGain>0 && isFinite(vD)&&vD>0 && isFinite(vS)&&vS>0 &&
               // γ 必须正；β 可正可负
               isFinite(vGamma) && vGamma>0 && isFinite(vBeta) &&
               (mode!=='cal' || (calMode!=='res' || (!isNaN(vRxA) && vRxA>0))) &&
               (mode!=='cal' || (calMode!=='cell' || (!isNaN(vKAct) && vKAct>0)));
    if(!baseOk){
      applyMsg.textContent = 'Invalid input. Please check inputs (positive numbers required; γ>0).';
      applyMsg.style.color = cssVar('--danger');
      return;
    }
    params.Rup=vRup; params.Rdown=vRdown; params.Gain=vGain; params.d=vD; params.S=vS;
    if(isFinite(vKc) && vKc>0) params.kcorr = vKc;

    // γ / β（Run 与 Cal/Resistance 使用；Cal/Cell 在勾选时也应用）
    params.gamma = vGamma;
    params.beta  = vBeta;
    params.applyGyInCell = vApplyGyCell;

    if(isFinite(vRxA) && vRxA>0) params.Rx_actual=vRxA;
    if(isFinite(vKAct) && vKAct>0) params.kappa_actual_uS = vKAct;

    if(isNaN(vKMax) || vKMax<=0) params.kappaMax = NaN; else params.kappaMax = vKMax;

    applyMsg.textContent = 'Applied ✓';
    applyMsg.style.color = cssVar('--primary');

    // Instantly update threshold line to prevent residue
    refreshThresholdMarkLine();

    // 如果当前在 Cell 子模式，根据勾选状态刷新 γ/β 可见性
    if (mode==='cal' && calMode==='cell'){
      setGammaBetaVisible(!!params.applyGyInCell);
    }
  };

  // ======== Calculations ========
  // ratio y = Vpp_ADC / (Vpp_exc * Gain)
  function calc_y(vpp_exc, vpp_adc, Gain){
    if(!(isFinite(vpp_exc)&&vpp_exc>0&&isFinite(vpp_adc)&&vpp_adc>=0&&isFinite(Gain)&&Gain>0)) return NaN;
    return vpp_adc / (vpp_exc * Gain);
  }
  // corrected ratio y' = γ*y + β
  function correct_y(y, gamma, beta){
    if(!isFinite(y) || !isFinite(gamma) || gamma<=0 || !isFinite(beta)) return NaN;
    return gamma * y + beta;
  }
  // Invert bridge to get Rx given ratio y (already corrected if needed)
  // A=Rup, B=Rdown; x = y*(A+B)^2 / (A - y*(A+B))
  function estimateRx_from_y(y, Rup, Rdown){
    if(!isFinite(y)) return NaN;
    const A = Rup, B = Rdown;
    const den = (A - y*(A+B));
    if(Math.abs(den) < 1e-12) return NaN;
    const x = y * (A+B)*(A+B) / den;
    return (x>0 && isFinite(x)) ? x : NaN;
  }

  function kappa_from_Rx_uS(d_cm, S_cm2, Rx, kcorr){
    if(!(isFinite(d_cm)&&d_cm>0&&isFinite(S_cm2)&&S_cm2>0&&isFinite(Rx)&&Rx>0&&isFinite(kcorr)&&kcorr>0)) return NaN;
    const K = d_cm / S_cm2; // cm^-1
    const kappa_S_per_cm = (K * kcorr) / Rx; // S/cm
    return kappa_S_per_cm * 1e6;             // µS/cm
  }
  function kappa_from_Rx_nominal_uS(d_cm, S_cm2, Rx){
    if(!(isFinite(d_cm)&&d_cm>0&&isFinite(S_cm2)&&S_cm2>0&&isFinite(Rx)&&Rx>0)) return NaN;
    const K = d_cm / S_cm2; // cm^-1
    return (K / Rx) * 1e6;  // µS/cm (no correction)
  }

  // ======== WebSocket ========
  const wsProto = (location.protocol === 'https:') ? 'wss' : 'ws';
  const wsUrl = wsProto + '://' + location.host + '/ws';
  let ws = null;
  let lastDraw = 0;

  function connect(){
    ws = new WebSocket(wsUrl);
    ws.onopen = ()=>{ statEl.textContent = 'Connected'; statEl.style.background=cssVar('--primary'); };
    ws.onclose = ()=>{ statEl.textContent = 'Disconnected, reconnecting...'; statEl.style.background=cssVar('--danger'); setTimeout(connect, 1000); };
    ws.onerror = ()=>{ statEl.textContent = 'Error'; statEl.style.background=cssVar('--danger'); };
    ws.onmessage = (ev)=>{
      const pkt = JSON.parse(ev.data);
      const t = performance.now()/1000 - t0;

      // Base curves: Vpp
      vpp_exc_data.push([t, pkt.vpp_exc]);
      vpp_adc_data.push([t, pkt.vpp_adc]);

      // Compute ratio y and conditionally corrected y'
      const y_raw = calc_y(pkt.vpp_exc, pkt.vpp_adc, params.Gain);
      const useCorr = (mode==='run') ||
                      (mode==='cal' && (calMode==='res' || (calMode==='cell' && params.applyGyInCell)));
      const y_use = useCorr ? correct_y(y_raw, params.gamma, params.beta) : y_raw;

      // Estimate Rx
      const Rx_est = estimateRx_from_y(y_use, params.Rup, params.Rdown);

      // === RUN or CAL data push ===
      let kappa_est_uS = NaN, err_pct = NaN;

      if (mode === 'run'){
        // Run: use kcorr + (possibly corrected) Rx
        kappa_est_uS = kappa_from_Rx_uS(params.d, params.S, Rx_est, params.kcorr);
        if(isFinite(kappa_est_uS)) condData.push([t, kappa_est_uS]);
      } else {
        if (calMode === 'res'){
          // Resistance calibration: 基于“电阻”的误差
          const R_true = params.Rx_actual;
          if(isFinite(R_true) && R_true>0 && isFinite(Rx_est)){
            err_pct = Math.abs(Rx_est - R_true) / R_true * 100.0;
          }
          if(isFinite(err_pct)) errData.push([t, err_pct]);
        } else {
          // Electrode Constant calibration: 基于 κ 的误差（使用 nominal K，不用 kcorr）
          const kappa_true = params.kappa_actual_uS;
          kappa_est_uS = kappa_from_Rx_nominal_uS(params.d, params.S, Rx_est);
          if(isFinite(kappa_true) && kappa_true>0 && isFinite(kappa_est_uS)){
            err_pct = Math.abs(kappa_est_uS - kappa_true) / kappa_true * 100.0;
          }
          if(isFinite(err_pct)) errData.push([t, err_pct]);
        }
      }

      trimWindow();

      // Refresh only at FPS
      const now = performance.now();
      if (now - lastDraw > 1000/FPS) {
        const xmin = Math.max(0, t - VPP_WINDOW_S);
        const xmax = Math.max(VPP_WINDOW_S, t);

        // Vpp chart (continuous x-axis scroll)
        const yMaxV = Math.max(seriesMax(vpp_exc_data), seriesMax(vpp_adc_data));
        chartVpp.setOption({
          xAxis:{ min: xmin, max: xmax },
          yAxis:{ min:0, max: Math.max(1.0, yMaxV*1.2), interval:1 },
          series:[ {name:'A1 Vpp_exc', data:vpp_exc_data}, {name:'A0 Vpp_ADC', data:vpp_adc_data} ]
        });

        // Right (top) chart
        if(mode==='run'){
          const yMaxC = seriesMax(condData);
          const ymax  = Math.max(100, yMaxC * 1.2);
          const th = themeColors();

          // Threshold check & alert
          const over = (isFinite(kappa_est_uS) && isFinite(params.kappaMax) && params.kappaMax>0 && (kappa_est_uS > params.kappaMax));
          rightTitle.style.color = over ? th.danger : th.text;
          alertBox.style.display = over ? 'block' : 'none';
          if(over){
            alertBox.textContent = 'Alert: Conductivity exceeds threshold (' + params.kappaMax.toFixed(0) + ' µS/cm)!';
          }

          const hasThr = Number.isFinite(params.kappaMax) && params.kappaMax>0;
          const seriesRun = {
            id:'series_kappa',
            name:'κ (µS/cm@25°C)', type:'line', showSymbol:false, data:condData,
            markLine: hasThr ? {
              symbol:'none',
              silent:true,
              label:{ color: th.text, formatter:'Threshold', position:'insideEndTop', distance:6 },
              lineStyle:{ type:'dashed', color: th.danger },
              data:[{ yAxis: params.kappaMax }]
            } : { data: [] }
          };

          rightChart.setOption({
            xAxis:{ min: xmin, max: xmax },
            yAxis: axisY('µS/cm@25°C', 0, ymax, false, 'kappa'),
            series:[ seriesRun ]
          });

          rightTitle.textContent = 'Conductivity κ (µS/cm@25°C): ' + (Number.isFinite(kappa_est_uS) ? (kappa_est_uS>=100? kappa_est_uS.toFixed(0):kappa_est_uS.toFixed(1)) : '—');
        }else{
          const yMaxE = seriesMax(errData);
          const ymax  = Math.max(1.0, yMaxE * 1.2);
          rightChart.setOption({
            xAxis:{ min: xmin, max: xmax },
            yAxis: axisY('%', 0, ymax, false, 'pct'),
            series:[ {name: 'Error %', type:'line', showSymbol:false, data:errData} ]
          });
          rightTitle.textContent = 'Error (%): ' + (Number.isFinite(err_pct) ? err_pct.toFixed(3) : '—');
        }

        // Readings and frame info
        const fsdisp = (typeof pkt.fs === 'number') ? pkt.fs : (pkt.fs||0);
        kv1.textContent = `${fsdisp} / ${pkt.win}`;
        kv2.textContent = `${pkt.method}`;
        kv3.textContent = `${(pkt.vpp_exc||0).toFixed(3)}  /  ${(pkt.vpp_adc||0).toFixed(3)}`;
        kv4.textContent = `${(pkt.lo0||0).toFixed(3)}  /  ${(pkt.hi0||0).toFixed(3)}`;
        kv5.textContent = `${(pkt.lo1||0).toFixed(3)}  /  ${(pkt.hi1||0).toFixed(3)}`;

        if(mode==='run'){
          const sRx = (isFinite(Rx_est)? Rx_est.toFixed(2) : '—');
          const sK  = (isFinite(kappa_est_uS)? (kappa_est_uS>=100? kappa_est_uS.toFixed(0):kappa_est_uS.toFixed(1)) : '—');
          kvRunVal.textContent = `${sRx}  /  ${sK}`;
        }else{
          if (calMode==='res'){
            const sEst = (isFinite(Rx_est)? Rx_est.toFixed(2) : '—');
            const sRef = (isFinite(params.Rx_actual)? params.Rx_actual.toFixed(2) : '—');
            const sEr  = (isFinite(err_pct)? err_pct.toFixed(3) : '—');
            kvCalLbl.textContent = 'Cal (R, %): est / ref / error';
            kvCalVal.textContent = `${sEst}  /  ${sRef}  /  ${sEr}`;
            kvCellLbl.style.display='none'; kvCellVal.style.display='none';
          } else {
            const sEst = (isFinite(kappa_est_uS)? (kappa_est_uS>=100? kappa_est_uS.toFixed(0):kappa_est_uS.toFixed(1)) : '—');
            const sRef = (isFinite(params.kappa_actual_uS)? (params.kappa_actual_uS>=100? params.kappa_actual_uS.toFixed(0):params.kappa_actual_uS.toFixed(1)) : '—');
            const sEr  = (isFinite(err_pct)? err_pct.toFixed(3) : '—');
            kvCalLbl.textContent = 'Cal (κ, %): est / ref / error';
            kvCalVal.textContent = `${sEst}  /  ${sRef}  /  ${sEr}`;

            // Show K_real and kc(real)
            const Knom = K_nominal(params.d, params.S);
            let Kreal = NaN, kcreal = NaN;
            if (isFinite(Rx_est) && isFinite(params.kappa_actual_uS)){
              const kappa_S_per_cm = params.kappa_actual_uS / 1e6;
              Kreal = kappa_S_per_cm * Rx_est; // cm^-1
              if (isFinite(Knom) && Knom>0) kcreal = Kreal / Knom;
            }
            const sKreal = Number.isFinite(Kreal) ? Kreal.toFixed(6) : '—';
            const skc    = Number.isFinite(kcreal) ? kcreal.toFixed(4) : '—';
            kvCellLbl.style.display=''; kvCellVal.style.display='';
            kvCellVal.textContent = `${sKreal}  /  ${skc}`;
            lastKcReal = (Number.isFinite(kcreal) && kcreal > 0) ? kcreal : NaN;
          }
        }

        lastDraw = now;
      }
    };
  }
  connect();

  function adoptKcorr(){
    if(!(Number.isFinite(lastKcReal) && lastKcReal > 0)){
      applyMsg.textContent = 'k_c(real) not ready, please provide κ_actual and wait for readings.';
      applyMsg.style.color = cssVar('--danger');
      return;
    }
    inpKcorr.value = lastKcReal.toFixed(4);
    params.kcorr = lastKcReal;
    applyMsg.textContent = 'Adopted k_c(real) = ' + lastKcReal.toFixed(4) + ' ✓';
    applyMsg.style.color = cssVar('--primary');

    // 切回 Run 模式并刷新
    setMode('run');              // 会清空 condData 并重置右图
    if (typeof adoptWrap !== 'undefined' && adoptWrap) adoptWrap.style.display = 'none';
    refreshTheme();              // 以确保主题/轴/样式统一
    refreshThresholdMarkLine();  // 维持阈值线
  }
  if (btnAdoptKc){
    btnAdoptKc.onclick = adoptKcorr;
  }

  // Collapse/Expand Vpp
  function setVppVisible(vis){
    vppVisible = !!vis;
    cardVpp.style.display = vis ? '' : 'none';
    btnVppToggle.textContent = vis ? 'Hide Voltage Chart' : 'Show Voltage Chart';
    if (vis){
      // Resize immediately after expanding
      setTimeout(()=>{ chartVpp.resize(); }, 0);
    }
  }
  btnVppToggle.onclick = ()=> setVppVisible(!vppVisible);
  setVppVisible(false); // Collapsed by default

  // Responsive resizing
  window.addEventListener('resize', ()=>{ rightChart.resize(); if(vppVisible) chartVpp.resize(); });

  // Theme switch binding
  function refreshTheme(){
    const th = themeColors();
    chartVpp.setOption({
      legend:{ textStyle:{ color: th.text } },
      xAxis: axisX('t (s)'),
      yAxis: axisY('V', 0, 4, true)
    });
    if(mode === 'run'){ initRightRun(); refreshThresholdMarkLine(); }
    else { initRightCal(); }

    if (mode === 'cal') {
      rightTitle.style.color = th.text;
    } else {
      const over = (alertBox.style.display === 'block');
      rightTitle.style.color = over ? th.danger : th.text;
    }
  }

  btnDark.onclick = ()=>{
    btnDark.classList.add('active'); btnLight.classList.remove('active');
    setTheme('dark');
  };
  btnLight.onclick = ()=>{
    btnLight.classList.add('active'); btnDark.classList.remove('active');
    setTheme('light');
  };

  // ===== Initial theme and chart setup order =====
  const initTheme = initThemeFromPref();
  if(initTheme==='light'){
    document.documentElement.setAttribute('data-theme','light');
    btnLight.classList.add('active'); btnDark.classList.remove('active');
  }else{
    document.documentElement.setAttribute('data-theme','dark');
    btnDark.classList.add('active'); btnLight.classList.remove('active');
  }
  initVppChart();
  setCalSubMode('res');
  setMode('run');
  //initRightRun();
  refreshTheme();   // Unify theme once

  // Initialize in Run mode then ensure default Cal sub-mode
  
  
})();
</script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
def index():
    return INDEX_HTML

@app.get("/healthz", response_class=PlainTextResponse)
def healthz():
    return "ok"

@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    last_sent_seq = -1
    try:
        while True:
            await asyncio.sleep(0.03)  # Poll every 30ms
            with pkt_lock:
                pkt = latest_pkt
            if pkt is not None and pkt["seq"] != last_sent_seq:
                await websocket.send_json(pkt)
                last_sent_seq = pkt["seq"]
    except WebSocketDisconnect:
        pass
    except Exception:
        pass

def get_local_ip() -> str:
    """Get the local IP address on the LAN if possible (for display purposes)"""
    ip = "127.0.0.1"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
    except:
        pass
    return ip

def main():
    parser = argparse.ArgumentParser(description="ADC LAN Web Server")
    parser.add_argument("--serial", default=DEFAULT_SERIAL_PORT, help="Serial port, e.g., COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate, default 921600")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP listening address, default 0.0.0.0 (accessible on LAN)")
    parser.add_argument("--http", type=int, default=12367, help="HTTP port, default 12367")
    args = parser.parse_args()

    th = threading.Thread(target=serial_reader_worker, args=(args.serial, args.baud), daemon=True)
    th.start()

    ip = get_local_ip()
    print(f"[INFO] Open your browser and visit: http://{ip}:{args.http}  (Accessible from any device on the same LAN)")

    uvicorn.run(app, host=args.host, port=args.http, log_level="info")

    stop_evt.set()
    try:
        th.join(timeout=1.0)
    except:
        pass

if __name__ == "__main__":
    main()
