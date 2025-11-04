#include "rotary_encoder.hpp"
#include "adc.hpp"
#include "motor.hpp"
#include "key.hpp"
#include "PID.hpp"
#include "freertos/ringbuf.h"

#define Center_Angle 2045    //中心角度
#define Center_Range 500     //调控区间，±500
#define START_PWM    35      //启摆时的PWM
#define START_TIME   100     //启摆时的驱动时间

/**
 * 引脚宏定义
 */

#define KEY1_GPIO_NUM GPIO_NUM_11
#define KEY2_GPIO_NUM GPIO_NUM_12
#define KEY3_GPIO_NUM GPIO_NUM_13
#define KEY4_GPIO_NUM GPIO_NUM_14

TaskHandle_t rotary_encoder_task_handle = NULL;
TaskHandle_t angle_task_handle = NULL;
TaskHandle_t motor_task_handle = NULL;
TaskHandle_t key_task_handle = NULL;
TaskHandle_t control_task_handle = NULL;

QueueHandle_t rotary_encoder_com_handle = NULL;     // 旋转编码器通信队列
QueueHandle_t angle_com_handle = NULL;              // ADC通信队列

QueueHandle_t rotary_encoder_request_handle = NULL; // 旋转编码器请求队列
QueueHandle_t angle_request_handle = NULL;          // ADC请求队列

PCNT rotary_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 1000, EC11_GPIO_A, EC11_GPIO_B, EC11_GPIO_B, EC11_GPIO_A, pcnt_on_reach);
ADC vertical_position;

PID_t Location_Pid; // 外环，位置环

uint8_t RunState = 0;

void rotary_encoder_task(void *arg)
{
    bool request_status = false;
    int location;
    while (true)
    {
        // rotary_encoder.print_count();
        xQueueReceive(rotary_encoder_request_handle, &request_status, portMAX_DELAY);
        location = rotary_encoder.location();
        xQueueSendToFront(rotary_encoder_com_handle, &location, portMAX_DELAY);
    }
}

void angle_task(void *arg)
{
    bool request_status = false;
    int angle;
    while (true)
    {
        xQueueReceive(angle_request_handle, &request_status, portMAX_DELAY);
        angle = vertical_position.read();
        xQueueSendToFront(angle_com_handle, &angle, portMAX_DELAY);
    }
}

void motor_task(void *arg)
{
    motor_timer_init();
    motor_channel_init();
    motor_control_init();
    motor_set_duty(128);
    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void key_task(void *arg)
{
    key key1("KEY1", KEY1_GPIO_NUM, key1_press_cb);
    key key2("KEY2", KEY2_GPIO_NUM, key2_press_cb);
    key key3("KEY3", KEY3_GPIO_NUM, key3_press_cb);
    key key4("KEY4", KEY4_GPIO_NUM, key4_press_cb);
    while(true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void control_task(void *arg)
{
    BaseType_t rotary_encoder_com_status;
    BaseType_t ADC_com_status;

    static uint16_t Count;
    static uint16_t Count2;
    static uint16_t CountStart;
    static uint16_t Count0;
    static uint16_t Angle0,Angle1,Angle2;//本次，上次，上上次

    PID_t Angle_Pid;
        Angle_Pid.Target = Center_Angle;
        Angle_Pid.OutMax = 100;
        Angle_Pid.OutMin = -100;
        Angle_Pid.Kp = 0.3;
        Angle_Pid.Ki = 0.01;
        Angle_Pid.Kd = 0.4;

    //外环，位置环
    
        Location_Pid.Target = 0;
        Location_Pid.OutMax = 100;
        Location_Pid.OutMin = -100;
        Location_Pid.Kp = 0.4;
        Location_Pid.Ki = 0;
        Location_Pid.Kd = 4;

    int Angle;
    //int Speed;
    int Location;

    bool request = true;
    xQueueSendToFront(angle_request_handle, &request, portMAX_DELAY);
    xQueueSendToFront(rotary_encoder_request_handle, &request, portMAX_DELAY);

    rotary_encoder_com_status = xQueueReceive(rotary_encoder_com_handle, &Location, portMAX_DELAY);
    ADC_com_status = xQueueReceive(angle_com_handle, &Angle, portMAX_DELAY);
    // 下面三行都用通信队列替换了
    // Angle = vertical_position.read();
    // Speed = rotary_encoder.delta();
    // Location = rotary_encoder.location();
    while(true){
        //自动启摆程序
        if (RunState == 0)
        {
            motor_set_duty(0);
        }
        else if (RunState == 1)
        {
            Count0++;
            if (Count0 >= 40)
            {
                Count0 = 0;

                Angle2 = Angle1;
                Angle1 = Angle0;        
                Angle0 = Angle;

                //判断位于右侧最高点
                if (Angle0 > Center_Angle + Center_Range
                && Angle1 > Center_Angle + Center_Range
                && Angle2 > Center_Angle + Center_Range
                && Angle1 < Angle0
                && Angle1 < Angle2)
                {
                    RunState = 21;
                }

                //判断位于左侧最高点
                if (Angle0 < Center_Angle - Center_Range
                && Angle1 < Center_Angle - Center_Range
                && Angle2 < Center_Angle - Center_Range
                && Angle1 > Angle0
                && Angle1 > Angle2)
                {
                    RunState = 31;
                }
                if (Angle0 > Center_Angle - Center_Range && Angle0 < Center_Angle + Center_Range
                && Angle1 > Center_Angle - Center_Range && Angle1 < Center_Angle + Center_Range)
                {
                    Location = 0;
                    Angle_Pid.ErrorInt = 0;
                    Location_Pid.ErrorInt = 0;
                    RunState = 4;
                }     
            }
        }
        //左驱动力
        else if (RunState == 21)
        {
            motor_set_duty(START_PWM);
            CountStart = START_TIME;
            RunState = 22;
        }
        else if (RunState == 22)
        {
        //延时
            CountStart--;
            if (CountStart == 0)
            {
                RunState = 23;
            }      
        }
        else if (RunState == 23)
        {
            motor_set_duty(-START_PWM);
            CountStart = START_TIME;
            RunState = 24;
        }
        else if (RunState == 24)
        {
            //延时
            CountStart--;
            if (CountStart == 0)
            {
                motor_set_duty(0);
                RunState = 1;
            }      
        }
        //右驱动力
        else if (RunState == 31)
        {
            motor_set_duty(-START_PWM);
            CountStart = START_TIME;
            RunState = 32;      
        }
        else if (RunState == 32)
        {
            //延时
            CountStart--;
        if (CountStart == 0)
        {
            RunState = 33;
        }    
        }
        else if (RunState == 33)
        {
            motor_set_duty(START_PWM);
            CountStart = START_TIME;
            RunState = 34;      
        }
        else if (RunState == 34)
        {
            //延时
            CountStart--;
            if (CountStart == 0)
            {
                motor_set_duty(0);
                RunState = 1;
            }      
        }
        //倒立摆启动
        else if (RunState == 4)
        {
            if ( !(Center_Angle - Center_Range < Angle && Angle < Center_Angle + Center_Range) )//倒立摆不在可调控区间
            {
                RunState = 0;
            }

            Count++;
            if (Count >= 5)//设置PID调控周期
            {
                Count = 0;
                Angle_Pid.Actual = Angle;
                PID_Update(&Angle_Pid);
                motor_set_duty(Angle_Pid.Out);
            }     

            Count2++;
            if (Count2 >= 50)
            {
                Count2 = 0;
                Location_Pid.Actual = Location;
                PID_Update(&Location_Pid);
                Angle_Pid.Target = Location_Pid.Out + Center_Angle;//位置环输出通过改变中心角度，从而实现位置的固定
            }     
        }
    }
}

void key1_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key1", "按下");
    if (RunState == 0)
    {
        RunState = 21;
    }else{
        RunState = 0;
      }
}

void key2_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key2", "按下");
    Location_Pid.Target += 408;
    if (Location_Pid.Target > 4080)
    {
    Location_Pid.Target = 4080;
    }
}

void key3_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key3", "按下");
    Location_Pid.Target -= 408;
    if (Location_Pid.Target < -4080)
    {
        Location_Pid.Target = -4080;
    }
}

void key4_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key4", "按下");
    RunState = !RunState;
}     

extern "C" void app_main(void)
{
    rotary_encoder_com_handle = xQueueCreate(1, sizeof(int));
    angle_com_handle = xQueueCreate(1, sizeof(int));

    rotary_encoder_request_handle = xQueueCreate(1, sizeof(bool));
    angle_request_handle = xQueueCreate(1, sizeof(bool));

    xTaskCreate(rotary_encoder_task, "rotary_encoder_task", 1024, NULL, 5, NULL);
    xTaskCreate(angle_task, "angle", 4096, NULL, 5, NULL); 
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(key_task, "key_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 8192, NULL, 10, NULL);
}