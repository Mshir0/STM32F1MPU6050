//
// Created by admin on 25-6-20.
//
#include "pid.h"

struct PID_SPEED {
    int Setspeed;
    int Actualspeed;
    int Errorspeed;
    int Error_last;
    int kp,ki,kd;
    int integral;
}pid_speed;

void pid_init() {
    pid_speed.Setspeed = 0;
    pid_speed.Actualspeed = 0;
    pid_speed.Errorspeed = 0;
    pid_speed.Error_last = 0;
    pid_speed.kp = 100;
    pid_speed.ki = 150;
    pid_speed.kd = 100;
    pid_speed.integral = 0;
}

int pid_update(int speed) {

    pid_speed.Setspeed = speed;
    pid_speed.Errorspeed = pid_speed.Setspeed - pid_speed.Actualspeed;
    pid_speed.integral += pid_speed.Errorspeed;
    pid_speed.Actualspeed = pid_speed.kp * pid_speed.Errorspeed + pid_speed.ki * pid_speed.integral + pid_speed.kd*(pid_speed.Errorspeed-pid_speed.Error_last);
    pid_speed.Error_last = pid_speed.Errorspeed;
    return pid_speed.Actualspeed;

}