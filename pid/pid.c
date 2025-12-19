#include "pid.h"

void PID_InitSpeed(PID *pid){
    pid->kp = 300.0; // 示例值，需实际调整
    pid->ki = 30.0;
    pid->kd = 2;
    pid->sumErr = 0;
    pid->prevErr = 0;
    
    pid->minSumErr = -100;
    pid->maxSumErr = 100;

    pid->minOutput = 0;
    pid->maxOutput = 7200;
}

void PID_InitAngle(PID *pid){
    pid->kp = 0.0; // 示例值，需实际调整
    pid->ki = 40.0;
    pid->kd = 0;
    pid->sumErr = 0;
    pid->prevErr = 0;
    
    pid->minSumErr = -100;
    pid->maxSumErr = 100;

    pid->minOutput = 0;
    pid->maxOutput = 7200;
}

float PID_Output(PID *pid, float curr, float target, float dt){
    float err = target - curr;
    float P = pid->kp * err;

    pid-> sumErr += err * dt;
    // 抗积分饱和：对积分项进行限幅
    if (pid->sumErr > pid->maxSumErr) {
        pid->sumErr = pid->maxSumErr;
    } else if (pid->sumErr < pid->minSumErr) {
        pid->sumErr = pid->minSumErr;
    }
    float I = pid->ki * pid->sumErr;

    // 微分项计算，使用标准的离散形式
    float D = pid->kd * (err - pid->prevErr) / dt;

    float res = P + I + D;

    // 输出限幅
    if (res > pid->maxOutput) {
        res = pid->maxOutput;
    } else if (res < pid->minOutput) {
        res = pid->minOutput;
    }

    // 更新上一次的误差
    pid->prevErr = err;

    return res;
}