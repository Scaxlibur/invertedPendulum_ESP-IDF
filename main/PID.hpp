#pragma once

typedef struct PID_t
{
   float Target;    //目标值
   float Actual;    //实际值
   float Out;       //输出值

   float Kp;
   float Ki;
   float Kd;

   float Error0;   //本次误差
   float Error1;   //上次误差
   float ErrorInt; //误差积分

   float OutMax;    //输出上限
   float OutMin;    //输出下限

}PID_t;

void PID_Update(PID_t *p);


