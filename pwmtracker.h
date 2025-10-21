#pragma once
#include <stdint.h>

struct PWMTrackerCfg {
  float trim_pct   = 0.0f;
  float ema_alpha  = 0.2f;
  float hyst_low   = 0.40f;
  float hyst_high  = 0.60f;
  float outlier_tol= 0.35f;
  int   min_cycles = 4;
};

struct PWMTrackerState {
  bool  ema_inited = false;
  float ema_lo = 0.0f;
  float ema_hi = 0.0f;
};

enum class Method : uint16_t { MinMax=0, Edge=1 };

// 函数原型
void pwmtracker_estimate(const uint16_t* x, int n, float fs,
                         const PWMTrackerCfg& cfg, PWMTrackerState& st,
                         float& A_code, float& Vpp_code, float& f_est,
                         float& lo_code, float& hi_code);

void pwmtracker_accurate(const uint16_t* x, int n, float fs,
                         const PWMTrackerCfg& cfg, PWMTrackerState& st,
                         float guard_frac, int min_guard_samps,
                         float& A_code, float& Vpp_code, float& f_est,
                         float& lo_code, float& hi_code);

Method hybrid_estimate_pwm(const uint16_t* x, int n, float fs,
                           const PWMTrackerCfg& cfg, PWMTrackerState& st,
                           float switch_vpp_v, float vref, float adc_fs,
                           float& A_code, float& Vpp_code, float& f_est,
                           float& lo_code, float& hi_code);
