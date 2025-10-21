#include <Arduino.h>
#include <functional>
#include <math.h>
#include <string.h>
#include <algorithm>

// ====== STM32L4 LL/HAL 头文件 ======
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_tim.h"

#include "pwmtracker.h"

// ===================== 用户可调参数 =====================
static const uint32_t SAMPLE_RATE_HZ   = 240000;   // ★ 每“通道”的采样率（TRGO=240kHz，两个 rank → 512kSPS）
static const uint32_t BAUD_FAST        = 921600;  // ★ 统计帧很小，921600 足够
static const uint32_t BAUD_BOOTMSG     = 115200;   // 上电文本打印
static const uint16_t BUF_LEN          = 4096;     // DMA 环形缓冲（偶数）
static const uint32_t PWM_FREQ_HZ      = 7000;     // D9/PA8/TIM1_CH1 -> 7 kHz
static const float    PWM_DUTY         = 0.5f;     // 50 %
#define  SEND_EVERY_MS_INT 1u                    // ★ 每 1ms 发送一次统计帧
static const float    SEND_EVERY_MS    = (float)SEND_EVERY_MS_INT;

// —— 量化/标定 ——（与 PC 端保持一致）
static const float VREF      = 3.3f;
static const float ADC_FS    = 4095.0f;
static const float CAL_FACTOR= 1.000f;

// —— 电桥与放大器参数（用于理论值与误差）——

// —— PWM 目标引脚 ——（保持不变）
#define PWM_GPIO              GPIOA
#define PWM_GPIO_PIN          LL_GPIO_PIN_8
#define PWM_GPIO_AF           LL_GPIO_AF_1
#define PWM_TIMER             TIM1
#define PWM_LL_CHANNEL        LL_TIM_CHANNEL_CH1

// —— ADC 通道映射 ——（A0→Rank1，A1→Rank2）
#define ADC_CH0_GPIO          GPIOA
#define ADC_CH0_PIN           LL_GPIO_PIN_0
#define ADC_CH0_LL_CHANNEL    LL_ADC_CHANNEL_5   // PA0 (A0)
#define ADC_CH1_GPIO          GPIOA
#define ADC_CH1_PIN           LL_GPIO_PIN_1
#define ADC_CH1_LL_CHANNEL    LL_ADC_CHANNEL_6   // PA1 (A1)
static const uint32_t ADC_NUM_CHANNELS = 2;

// ===== 统计帧协议 =====
static const uint8_t FRAME_HDR[4] = {0xAD, 0xDA, 0x12, 0x34};
static const uint16_t PAYLOAD_FLOATS = 6; // vpp_exc, vpp_adc, lo0, hi0, lo1, hi1
static const uint16_t PAYLOAD_BYTES  = 4+4+2+2 + (PAYLOAD_FLOATS*4); // = 36 bytes

// ===== DMA/ADC 缓冲与标志 =====
__attribute__((aligned(4))) static volatile uint16_t g_adcBuf[BUF_LEN];
static volatile bool g_halfReady = false;
static volatile bool g_fullReady  = false;

// ===== Min/Max 累积（用于快速 Vpp） =====
static volatile uint16_t s_lo0 = 0xFFFF, s_hi0 = 0x0000; // A0
static volatile uint16_t s_lo1 = 0xFFFF, s_hi1 = 0x0000; // A1
static volatile uint32_t s_accum_pairs = 0;              // 累计样本对数
static volatile uint32_t s_seq = 0;

// ===== 分析窗口：每通道保存最近 20ms 样本（用于稳健法） =====
#define  ANAL_SAMPS_PER_CH ((uint16_t)((SAMPLE_RATE_HZ * SEND_EVERY_MS_INT)/1000u)) // 256k * 0.02 = 5120
__attribute__((aligned(4))) static volatile uint16_t s_abuf0[ANAL_SAMPS_PER_CH];
__attribute__((aligned(4))) static volatile uint16_t s_abuf1[ANAL_SAMPS_PER_CH];
static volatile uint16_t s_apos = 0; // 环形写指针（以 pair 粒度推进）

// ====================== 工具与协议 ======================
static inline float code_to_v(uint16_t code) {
  return (float)code * (VREF / ADC_FS) * CAL_FACTOR;
}
static inline bool wait_until(std::function<bool(void)> cond, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (!cond()) {
    if ((millis() - t0) > timeout_ms) return false;
    __NOP();
  }
  return true;
}

static inline void sendPayloadFrame(
  uint32_t seq, uint32_t fs_per_ch, uint16_t win_samples_per_ch, uint16_t method_code,
  float vpp_exc_v, float vpp_adc_v,
  float lo0_v, float hi0_v, float lo1_v, float hi1_v
) {
  uint8_t payload[PAYLOAD_BYTES];
  uint8_t *p = payload;

  auto put_u32 = [&](uint32_t v){ *p++=v&0xFF; *p++=(v>>8)&0xFF; *p++=(v>>16)&0xFF; *p++=(v>>24)&0xFF; };
  auto put_u16 = [&](uint16_t v){ *p++=v&0xFF; *p++=(v>>8)&0xFF; };
  auto put_f32 = [&](float f){ uint32_t u=*(uint32_t*)&f; put_u32(u); };

  put_u32(seq);
  put_u32(fs_per_ch);
  put_u16(win_samples_per_ch);
  put_u16(method_code);               // ★ reserved: 0=MinMax, 1=Edge

  put_f32(vpp_exc_v);
  put_f32(vpp_adc_v);
  put_f32(lo0_v); put_f32(hi0_v);
  put_f32(lo1_v); put_f32(hi1_v);

  // —— 发帧 —— HDR + LEN + PAYLOAD + CSUM16
  Serial.write(FRAME_HDR, sizeof(FRAME_HDR));
  uint8_t lenle[2] = { (uint8_t)(PAYLOAD_BYTES & 0xFF), (uint8_t)(PAYLOAD_BYTES >> 8) };
  Serial.write(lenle, 2);
  Serial.write(payload, PAYLOAD_BYTES);

  uint32_t sum = lenle[0] + lenle[1];
  for (uint16_t i = 0; i < PAYLOAD_BYTES; ++i) sum += payload[i];
  uint16_t csum = (uint16_t)(sum & 0xFFFF);
  uint8_t csle[2] = { (uint8_t)(csum & 0xFF), (uint8_t)(csum >> 8) };
  Serial.write(csle, 2);
}

// === TIM2/TIM1 时钟 ===
static uint32_t tim2_get_clock_hz() {
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  uint32_t apb1_div = LL_RCC_GetAPB1Prescaler();
  return (apb1_div != LL_RCC_APB1_DIV_1) ? (pclk1 * 2U) : pclk1;
}
static uint32_t tim1_get_clock_hz() {
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t apb2_div = LL_RCC_GetAPB2Prescaler();
  return (apb2_div != LL_RCC_APB2_DIV_1) ? (pclk2 * 2U) : pclk2;
}
static double calc_fs_from_tim2() {
  const uint32_t tim_clk = tim2_get_clock_hz();
  const uint32_t psc = (uint32_t)LL_TIM_GetPrescaler(TIM2);
  const uint32_t arr = (uint32_t)LL_TIM_GetAutoReload(TIM2);
  return (double)tim_clk / (double)((psc + 1ULL) * (arr + 1ULL));
}
static uint32_t get_adc_async_clk_hz() {
  uint32_t sys = HAL_RCC_GetSysClockFreq();
  return sys / 8U; // LL_ADC_CLOCK_ASYNC_DIV8
}
static void print_rate_info_once(uint32_t tim_clk) {
  const double fs_trig = calc_fs_from_tim2();
  const double fs_eff  = fs_trig * (double)ADC_NUM_CHANNELS;
  const uint32_t adcclk = get_adc_async_clk_hz();
  const double tconv_cycles = 12.5 + 6.5; // ★ 12bit + 6.5 cycles sampling
  const double max_sps = (double)adcclk / tconv_cycles;
  const double margin_pct = (max_sps > 0.0) ? (100.0 * (max_sps - fs_eff) / max_sps) : 0.0;

  Serial.print("RATE INFO: SYSCLK=");
  Serial.print(HAL_RCC_GetSysClockFreq());
  Serial.print(" Hz, TIM2clk=");
  Serial.print(tim_clk);
  Serial.print(" Hz, ADCclk=");
  Serial.print(adcclk);
  Serial.print(" Hz, Tconv=");
  Serial.print(tconv_cycles, 1);
  Serial.print(" cyc => max=");
  Serial.print(max_sps / 1000.0, 1);
  Serial.print(" kSPS, TRGO=");
  Serial.print(fs_trig, 3);
  Serial.print(" Hz, effective ADC SPS=");
  Serial.print(fs_eff, 1);
  Serial.print(", margin=");
  Serial.print(margin_pct, 1);
  Serial.println("%");
}

// ====== PWM 与 DMA/ADC 初始化（与你之前版本一致，略有调整）======
static void pwm_init_and_print() {
  LL_GPIO_SetPinMode(PWM_GPIO, PWM_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(PWM_GPIO, PWM_GPIO_PIN, PWM_GPIO_AF);
  LL_GPIO_SetPinSpeed(PWM_GPIO, PWM_GPIO_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinPull(PWM_GPIO, PWM_GPIO_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinOutputType(PWM_GPIO, PWM_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);

  uint32_t tim_clk = tim1_get_clock_hz();
  const uint32_t target_cnt = 1000000UL; // 1 MHz
  uint32_t psc = (tim_clk / target_cnt); if (psc==0) psc=1; psc -= 1;
  uint32_t arr = (target_cnt / PWM_FREQ_HZ); if (arr==0) arr=1; arr -= 1;

  LL_TIM_DisableCounter(PWM_TIMER);
  LL_TIM_SetPrescaler(PWM_TIMER, (uint16_t)psc);
  LL_TIM_SetAutoReload(PWM_TIMER, (uint32_t)arr);
  LL_TIM_SetCounterMode(PWM_TIMER, LL_TIM_COUNTERMODE_UP);
  LL_TIM_EnableARRPreload(PWM_TIMER);

  LL_TIM_OC_InitTypeDef oc = {};
  oc.OCMode       = LL_TIM_OCMODE_PWM1;
  oc.OCState      = LL_TIM_OCSTATE_ENABLE;
  oc.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
  oc.CompareValue = (uint32_t)((arr + 1U) * PWM_DUTY + 0.5f);
  LL_TIM_OC_Init(PWM_TIMER, PWM_LL_CHANNEL, &oc);
  LL_TIM_OC_EnablePreload(PWM_TIMER, PWM_LL_CHANNEL);
  LL_TIM_CC_EnableChannel(PWM_TIMER, PWM_LL_CHANNEL);

  LL_TIM_EnableAllOutputs(PWM_TIMER);
  LL_TIM_GenerateEvent_UPDATE(PWM_TIMER);
  LL_TIM_EnableCounter(PWM_TIMER);

  const double pwm_freq = (double)tim_clk / ((double)(psc + 1ULL) * (double)(arr + 1ULL));
  const uint32_t ccr = LL_TIM_OC_GetCompareCH1(PWM_TIMER);
  const double duty = 100.0 * ((double)ccr / (double)(arr + 1ULL));

  Serial.print("PWM INFO: D9/PA8 (TIM1_CH1) target=1kHz@50%. Real F=");
  Serial.print(pwm_freq, 3);
  Serial.print(" Hz, duty=");
  Serial.print(duty, 1);
  Serial.println("%");
}

// ===== DMA ISR =====
extern "C" void DMA1_Channel1_IRQHandler(void) {
  if (LL_DMA_IsActiveFlag_HT1(DMA1)) { LL_DMA_ClearFlag_HT1(DMA1); g_halfReady = true; }
  if (LL_DMA_IsActiveFlag_TC1(DMA1)) { LL_DMA_ClearFlag_TC1(DMA1); g_fullReady = true; }
  if (LL_DMA_IsActiveFlag_TE1(DMA1)) { LL_DMA_ClearFlag_TE1(DMA1); }
}

// ===== 半/满缓冲处理：累积 min/max + 写入分析窗口 =====
static inline void accumulate_and_fill_abuf(const uint16_t* p, uint16_t count_samples) {
  uint16_t lo0 = 0xFFFF, hi0 = 0x0000;
  uint16_t lo1 = 0xFFFF, hi1 = 0x0000;
  uint32_t pairs = 0;

  for (uint16_t i=0; i<count_samples; i+=2) {
    uint16_t a0 = p[i+0];
    uint16_t a1 = p[i+1];
    if (a0 < lo0) lo0=a0; if (a0 > hi0) hi0=a0;
    if (a1 < lo1) lo1=a1; if (a1 > hi1) hi1=a1;

    // 写入分析环形缓冲
    uint16_t pos = s_apos;
    s_abuf0[pos] = a0;
    s_abuf1[pos] = a1;
    pos++; if (pos >= ANAL_SAMPS_PER_CH) pos = 0;
    s_apos = pos;

    ++pairs;
  }

  __disable_irq();
  if (lo0 < s_lo0) s_lo0 = lo0;
  if (hi0 > s_hi0) s_hi0 = hi0;
  if (lo1 < s_lo1) s_lo1 = lo1;
  if (hi1 > s_hi1) s_hi1 = hi1;
  s_accum_pairs += pairs;
  __enable_irq();
}

// ====================== setup / loop ======================
void setup() {
  Serial.begin(BAUD_BOOTMSG);
  delay(30);
  Serial.println("\nBOOT @115200");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 时钟
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); // ADC 触发
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1); // PWM
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
  Serial.println("CLK OK");

  // GPIO 模拟
  LL_GPIO_SetPinMode(ADC_CH0_GPIO, ADC_CH0_PIN, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinPull(ADC_CH0_GPIO, ADC_CH0_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(ADC_CH1_GPIO, ADC_CH1_PIN, LL_GPIO_MODE_ANALOG);
  LL_GPIO_SetPinPull(ADC_CH1_GPIO, ADC_CH1_PIN, LL_GPIO_PULL_NO);
  Serial.println("GPIO OK");

  // DMA
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);
  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,  LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA));
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)g_adcBuf);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, BUF_LEN);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  Serial.println("DMA OK");

  // ADC 上电校准
  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);
  delay(2);
  if (LL_ADC_IsActiveFlag_ADRDY(ADC1)) LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  bool cal_ok = wait_until([](){ return !LL_ADC_IsCalibrationOnGoing(ADC1); }, 500);
  Serial.println(cal_ok ? "ADC CAL OK" : "ADC CAL FAIL (timeout)");
  delay(1);

  // ADC 配置：TRGO 触发、双通道扫描
  LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_ASYNC_DIV8); // ~10MHz
  LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
  LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);
  LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM2_TRGO);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);
  LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  LL_ADC_REG_SetOverrun(ADC1, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

  // ★ 6.5 cycles 采样时间（支撑 512kSPS）
  LL_ADC_SetChannelSamplingTime(ADC1, ADC_CH0_LL_CHANNEL, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSamplingTime(ADC1, ADC_CH1_LL_CHANNEL, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, ADC_CH0_LL_CHANNEL);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, ADC_CH1_LL_CHANNEL);

  LL_ADC_ClearFlag_ADRDY(ADC1);
  LL_ADC_Enable(ADC1);
  bool rdy_ok = wait_until([](){ return LL_ADC_IsActiveFlag_ADRDY(ADC1); }, 500);
  Serial.println(rdy_ok ? "ADC READY OK" : "ADC READY FAIL (timeout)");

  // TIM2: 256kHz TRGO（psc=0, arr≈80MHz/256k - 1 ≈ 312）
  uint32_t tim2_clk = tim2_get_clock_hz();
  LL_TIM_DisableCounter(TIM2);
  uint32_t psc = 0;
  uint32_t arr = (tim2_clk / SAMPLE_RATE_HZ) - 1; if (arr<1) arr=1;
  LL_TIM_SetPrescaler(TIM2, (uint16_t)psc);
  LL_TIM_SetAutoReload(TIM2, (uint32_t)arr);
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);

  print_rate_info_once(tim2_clk);
  delay(15);

  // PWM 1k
  pwm_init_and_print();
  delay(15);
  print_rate_info_once(tim2_clk);
  delay(15);

  // 启动链路
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
  LL_ADC_REG_StartConversion(ADC1);
  LL_TIM_EnableCounter(TIM2);

  // 切到 2M 串口
  Serial.println("SWITCH TO 2M ...");
  Serial.flush();
  Serial.end();
  delay(15);
  Serial.begin(BAUD_FAST);
  delay(5);
}

static PWMTrackerCfg   g_cfg = { 2.0f, 0.2f, 0.40f, 0.60f, 0.35f, 4 };
static PWMTrackerState g_st0; // A0
static PWMTrackerState g_st1; // A1

void loop() {
  // 心跳灯
  static uint32_t blink_t0 = 0;
  if (millis() - blink_t0 > 250) { blink_t0 = millis(); digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }

  if (g_halfReady) { g_halfReady = false; accumulate_and_fill_abuf((const uint16_t*)g_adcBuf, BUF_LEN/2); }
  if (g_fullReady) { g_fullReady = false; accumulate_and_fill_abuf((const uint16_t*)(g_adcBuf + BUF_LEN/2), BUF_LEN/2); }

  static uint32_t t0 = millis();
  uint32_t now = millis();
  if ((now - t0) >= SEND_EVERY_MS_INT) {
    t0 = now;

    // 取出并复位统计 + 拷贝分析窗
    uint16_t lo0, hi0, lo1, hi1; uint32_t pairs; uint32_t seq;
    static uint16_t win0[ANAL_SAMPS_PER_CH];
    static uint16_t win1[ANAL_SAMPS_PER_CH];
    __disable_irq();
    lo0=s_lo0; hi0=s_hi0; lo1=s_lo1; hi1=s_hi1; pairs=s_accum_pairs;
    s_lo0=0xFFFF; s_hi0=0x0000; s_lo1=0xFFFF; s_hi1=0x0000; s_accum_pairs=0;
    seq=++s_seq;
    uint16_t pos = s_apos;
    for (uint16_t i=0;i<ANAL_SAMPS_PER_CH;++i){
      uint16_t idx = pos + i; if (idx >= ANAL_SAMPS_PER_CH) idx -= ANAL_SAMPS_PER_CH;
      win0[i] = s_abuf0[idx];
      win1[i] = s_abuf1[idx];
    }
    __enable_irq();

    // A1（激励）快速法
    float A1c,Vpp1c,f1c,lo1c,hi1c;
    pwmtracker_estimate(win1, ANAL_SAMPS_PER_CH, (float)SAMPLE_RATE_HZ, g_cfg, g_st1,
                        A1c,Vpp1c,f1c,lo1c,hi1c);
    float Vpp1_v = Vpp1c * (VREF/ADC_FS) * CAL_FACTOR;

    // A0（差分放大器）hybrid
    float A0c,Vpp0c,f0c,lo0c,hi0c;
    Method meth = hybrid_estimate_pwm(win0, ANAL_SAMPS_PER_CH, (float)SAMPLE_RATE_HZ,
                                      g_cfg, g_st0, 2.9f, VREF, ADC_FS,
                                      A0c,Vpp0c,f0c,lo0c,hi0c);
    uint16_t method_code = (meth==Method::Edge)? 1u : 0u;
    float Vpp0_v = Vpp0c * (VREF/ADC_FS) * CAL_FACTOR;

    // 发送统计帧（lo/hi 转电压便于观测）
    sendPayloadFrame(seq, SAMPLE_RATE_HZ, (uint16_t)pairs, method_code,
                     Vpp1_v, Vpp0_v,
                     lo0c*(VREF/ADC_FS), hi0c*(VREF/ADC_FS),
                     lo1c*(VREF/ADC_FS), hi1c*(VREF/ADC_FS));
  }
}
