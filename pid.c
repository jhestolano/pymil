#include "pid.h"

typedef struct {
  float kp;
  float ki;
  float kd;
} pid_t;

typedef struct {
  pid_t gains;
  float ts;
  float err_act;
  float err_prev;
  float err_igr;
  float err_drv;
  float ctrl;
} pid_state_t;

static pid_state_t g_pid_state;

void pid_init(float kp, float ki, float kd, float ts) {
  g_pid_state.gains.kp = kp;
  g_pid_state.gains.kd = kd;
  g_pid_state.gains.ki = ki;
  g_pid_state.ts = ts;
  g_pid_state.err_act = 0.0f;
  g_pid_state.err_prev = 0.0f;
  g_pid_state.err_igr = 0.0f;
  g_pid_state.ctrl = 0.0f;
}

float pid_run(float y_ref, float y_act) {
  g_pid_state.err_act = y_ref - y_act;
  g_pid_state.err_igr += g_pid_state.err_act * g_pid_state.ts;
  g_pid_state.err_drv = (g_pid_state.err_act - g_pid_state.err_prev) / g_pid_state.ts;

  g_pid_state.ctrl = g_pid_state.gains.kp * g_pid_state.err_act
                   + g_pid_state.gains.kd * g_pid_state.err_drv
                   + g_pid_state.gains.ki * g_pid_state.err_igr;

  return g_pid_state.ctrl;
}
