#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float kp;
    float ki;
    float kd;

    float sumErr;
    float prevErr;
    
    float minSumErr;    // 积分项限幅
    float maxSumErr;
    float minOutput;    // 总输出限幅
    float maxOutput;
} PID;

void PID_InitSpeed(PID *pid);
void PID_InitAngle(PID *pid);
float PID_Output(PID *pid, float curr, float target, float dt);

#endif