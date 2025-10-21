#include "pwmtracker.h"
#include <math.h>
#include <string.h>
#include <algorithm>

static inline float _median_inplace(float* a, int n) {
  if (n <= 0) return 0.f;
  for (int i=1;i<n;++i){
    float v=a[i]; int j=i-1;
    while (j>=0 && a[j]>v){ a[j+1]=a[j]; --j; }
    a[j+1]=v;
  }
  return (n&1) ? a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}

// 12-bit ADC → 256 桶直方图近似分位（右移4位）
static void _percentiles_u16_hist(const uint16_t* x, int n, float p_lo, float p_hi,
                                  uint16_t& out_lo_code, uint16_t& out_hi_code) {
  static uint32_t hist[256];
  memset(hist, 0, sizeof(hist));
  for (int i=0;i<n;++i) { uint8_t b = (uint8_t)(x[i] >> 4); hist[b]++; }
  uint32_t tot=n, k_lo=(uint32_t)((p_lo/100.f)*tot), k_hi=(uint32_t)((p_hi/100.f)*tot);
  uint32_t acc=0; uint16_t lo_bin=0, hi_bin=255;
  for (int b=0;b<256;++b){ acc+=hist[b]; if (acc>k_lo){ lo_bin=(uint16_t)b; break; } }
  acc=0;
  for (int b=0;b<256;++b){ acc+=hist[b]; if (acc>k_hi){ hi_bin=(uint16_t)b; break; } }
  out_lo_code = (uint16_t)(lo_bin<<4);
  out_hi_code = (uint16_t)(hi_bin<<4);
}

static int _rising_edges(const uint16_t* x, int n, float thr_up, float thr_dn,
                         float* out_idx, int max_idx) {
  if (n<2) return 0;
  bool state_high = (x[0] >= thr_up);
  int count=0;
  for (int i=0;i<n-1 && count<max_idx; ++i){
    float xi=(float)x[i], xn=(float)x[i+1];
    if (!state_high) {
      if (xi < thr_up && xn >= thr_up) {
        float denom = (xn - xi);
        float frac = (denom==0.f)?0.5f:(thr_up - xi)/denom;
        if (frac<0.f) frac=0.f; if (frac>1.f) frac=1.f;
        out_idx[count++] = (float)i + frac;
        state_high = true;
      }
    } else {
      if (xi > thr_dn && xn <= thr_dn) state_high = false;
    }
  }
  return count;
}

static int _falling_edges(const uint16_t* x, int n, float thr_up, float thr_dn,
                          int* out_idx, int max_idx){
  if (n<2) return 0;
  bool state_high = (x[0] >= thr_up);
  int count=0;
  for (int i=0;i<n-1 && count<max_idx; ++i){
    float xi=(float)x[i], xn=(float)x[i+1];
    if (!state_high) {
      if (xi < thr_up && xn >= thr_up) state_high = true;
    } else {
      if (xi > thr_dn && xn <= thr_dn) { out_idx[count++]=i; state_high=false; }
    }
  }
  return count;
}

void pwmtracker_estimate(const uint16_t* x, int n, float fs,
                         const PWMTrackerCfg& cfg, PWMTrackerState& st,
                         float& A_code, float& Vpp_code, float& f_est,
                         float& lo_code, float& hi_code) {
  if (n<8){ A_code=Vpp_code=lo_code=hi_code=0.f; f_est=0.f; return; }
  uint16_t lo_c=0, hi_c=4095;
  if (cfg.trim_pct>0.f) _percentiles_u16_hist(x,n,cfg.trim_pct,100.f-cfg.trim_pct,lo_c,hi_c);
  else {
    uint16_t lo=0xFFFF, hi=0;
    for (int i=0;i<n;++i){ uint16_t v=x[i]; if(v<lo)lo=v; if(v>hi)hi=v; }
    lo_c=lo; hi_c=hi;
  }
  if (!st.ema_inited) { st.ema_lo=(float)lo_c; st.ema_hi=(float)hi_c; st.ema_inited=true; }
  else {
    float a=cfg.ema_alpha;
    st.ema_lo=(1.f-a)*st.ema_lo + a*(float)lo_c;
    st.ema_hi=(1.f-a)*st.ema_hi + a*(float)hi_c;
  }
  lo_code=st.ema_lo; hi_code=st.ema_hi;
  Vpp_code=(hi_code>lo_code)?(hi_code-lo_code):0.f; A_code=0.5f*Vpp_code;

  float thr_up = lo_code + cfg.hyst_high*(hi_code-lo_code);
  float thr_dn = lo_code + cfg.hyst_low *(hi_code-lo_code);
  float rise[128]; int nr=_rising_edges(x,n,thr_up,thr_dn,rise,128);
  f_est=0.f;
  if (nr>=cfg.min_cycles+1){
    float per[127]; int np=0; for (int i=1;i<nr;++i) per[np++]=rise[i]-rise[i-1];
    float med=_median_inplace(per,np);
    if (med>0.f){
      int m2=0; for (int i=0;i<np;++i) if (fabsf(per[i]-med)<=cfg.outlier_tol*med) per[m2++]=per[i];
      float T=(m2>0)?_median_inplace(per,m2):med; if (T>0.f) f_est = fs / T;
    }
  }
}

void pwmtracker_accurate(const uint16_t* x, int n, float fs,
                         const PWMTrackerCfg& cfg, PWMTrackerState& st,
                         float guard_frac, int min_guard_samps,
                         float& A_code, float& Vpp_code, float& f_est,
                         float& lo_code, float& hi_code) {
  if (n<32){ A_code=Vpp_code=lo_code=hi_code=0.f; f_est=0.f; return; }
  float A0,Vpp0,f0,lo0,hi0;
  pwmtracker_estimate(x,n,fs,cfg,st,A0,Vpp0,f0,lo0,hi0);
  float thr_up = lo0 + cfg.hyst_high*(hi0-lo0);
  float thr_dn = lo0 + cfg.hyst_low *(hi0-lo0);

  float risef[256]; int nr=_rising_edges(x,n,thr_up,thr_dn,risef,256);
  int falli[256];   int nf=_falling_edges(x,n,thr_up,thr_dn,falli,256);

  f_est=0.f;
  if (nr>=cfg.min_cycles+1){
    float per[255]; int np=0; for (int i=1;i<nr;++i) per[np++]=risef[i]-risef[i-1];
    float med=_median_inplace(per,np);
    if (med>0.f){
      int m2=0; for (int i=0;i<np;++i) if(fabsf(per[i]-med)<=cfg.outlier_tol*med) per[m2++]=per[i];
      float T=(m2>0)?_median_inplace(per,m2):med; if (T>0.f) f_est = fs / T;
    }
  }

  float hi_levels[128]; int nhi=0;
  float lo_levels[128]; int nlo=0;
  for (int i=0;i<nr-1; ++i){
    int start=(int)risef[i], end=(int)risef[i+1];
    int fall=-1; for (int k=0;k<nf;++k){ if (falli[k]>start && falli[k]<end){ fall=falli[k]; break; } }
    if (fall<0) continue;
    int lenH=fall-start, lenL=end-fall; if (lenH<3||lenL<3) continue;
    int gH=std::max((int)(lenH*guard_frac), min_guard_samps);
    int gL=std::max((int)(lenL*guard_frac), min_guard_samps);
    int hs=start+gH, he=fall-gH, ls=fall+gL, le=end-gL;
    if (he-hs>=3){
      double s=0; int c=0; for (int t=hs;t<he;++t){ s+=x[t]; ++c; }
      if (c>0 && nhi<128) hi_levels[nhi++]=(float)(s/c);
    }
    if (le-ls>=3){
      double s=0; int c=0; for (int t=ls;t<le;++t){ s+=x[t]; ++c; }
      if (c>0 && nlo<128) lo_levels[nlo++]=(float)(s/c);
    }
  }
  float hi_med=(nhi>0)?_median_inplace(hi_levels,nhi):hi0;
  float lo_med=(nlo>0)?_median_inplace(lo_levels,nlo):lo0;

  if (!st.ema_inited){ st.ema_lo=lo_med; st.ema_hi=hi_med; st.ema_inited=true; }
  else { float a=cfg.ema_alpha; st.ema_lo=(1.f-a)*st.ema_lo+a*lo_med; st.ema_hi=(1.f-a)*st.ema_hi+a*hi_med; }

  lo_code=st.ema_lo; hi_code=st.ema_hi;
  Vpp_code=(hi_code>lo_code)?(hi_code-lo_code):0.f; A_code=0.5f*Vpp_code;
}

Method hybrid_estimate_pwm(const uint16_t* x, int n, float fs,
                           const PWMTrackerCfg& cfg, PWMTrackerState& st,
                           float switch_vpp_v, float vref, float adc_fs,
                           float& A_code, float& Vpp_code, float& f_est,
                           float& lo_code, float& hi_code) {
  float Ae,Vppe,fe,loe,hie;
  pwmtracker_estimate(x,n,fs,cfg,st,Ae,Vppe,fe,loe,hie);
  float Vppe_v = Vppe * (vref/adc_fs);
  if (Vppe_v < switch_vpp_v) {
    pwmtracker_accurate(x,n,fs,cfg,st,0.18f,4,A_code,Vpp_code,f_est,lo_code,hi_code);
    //A_code=Ae; Vpp_code=Vppe; f_est=fe; lo_code=loe; hi_code=hie;
    return Method::Edge;
  } else {
    A_code=Ae; Vpp_code=Vppe; f_est=fe; lo_code=loe; hi_code=hie;
    return Method::MinMax;
  }
}
