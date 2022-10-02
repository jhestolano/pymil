#ifndef PID_H

void pid_init(float kp, float ki, float kd, float ts);

float pid_run(float y_ref, float y_act);

#endif // !PID_H
