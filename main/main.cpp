#include "rotary_encoder.hpp"
#include "adc.hpp"
#include "motor.hpp"
#include "key.hpp"
#include "PID.hpp"

constexpr int16_t Center_Angle = 1960;  //中心角度
constexpr int16_t Center_Range = 500;   //调控区间，±500
constexpr int16_t START_PWM = 90;       //启摆时的PWM
constexpr uint8_t START_TIME = 100;     //启摆时的驱动时间

/**
 * 引脚宏定义
 */

constexpr gpio_num_t KEY1_GPIO_NUM = GPIO_NUM_4;
constexpr gpio_num_t KEY2_GPIO_NUM = GPIO_NUM_5;
constexpr gpio_num_t KEY3_GPIO_NUM = GPIO_NUM_6;
constexpr gpio_num_t KEY4_GPIO_NUM = GPIO_NUM_7;

TaskHandle_t rotary_encoder_task_handle = NULL;
TaskHandle_t angle_task_handle = NULL;
TaskHandle_t motor_task_handle = NULL;
TaskHandle_t key_task_handle = NULL;
TaskHandle_t control_task_handle = NULL;
TaskHandle_t log_task_handle = NULL;

/**
 * 控制任务的通信队列
 */
QueueHandle_t rotary_encoder_com_handle = NULL;     // 旋转编码器通信队列
QueueHandle_t angle_com_handle = NULL;              // ADC通信队列
QueueHandle_t log_runstate_com_handle = NULL;       // 日志_状态通信队列

/**
 * 控制任务的请求任务
 */
QueueHandle_t rotary_encoder_request_handle = NULL; // 旋转编码器请求队列
QueueHandle_t angle_request_handle = NULL;          // ADC请求队列

PCNT rotary_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 1000, MOTOR_ENCODER_GPIO_A, MOTOR_ENCODER_GPIO_B, MOTOR_ENCODER_GPIO_B, MOTOR_ENCODER_GPIO_A, pcnt_on_reach);
ADC vertical_position;

PID_t Location_Pid; // 外环，位置环

typedef enum runstate_t
{
    STOPPED,
    WAITING_FOR_START,
    SWINGING_UP_LEFT,
    SWINGING_UP_LEFT_DELAY,
    SWINGING_UP_LEFT_BACK,
    SWINGING_UP_LEFT_JUDGE,

    SWINGING_UP_RIGHT,
    SWINGING_UP_RIGHT_DELAY,
    SWINGING_UP_RIGHT_BACK,
    SWINGING_UP_RIGHT_JUDGE,

    BALANCING
} runstate_t;

runstate_t RunState = STOPPED;

void rotary_encoder_task(void *arg)
{
    const char *TAG = "rotary_encoder_task";
    bool request_status = false;
    int location;
    while (true)
    {
        xQueueReceive(rotary_encoder_request_handle, &request_status, portMAX_DELAY);
        location = rotary_encoder.location();
        // ESP_LOGI(TAG, "编码器%d", location);
        xQueueSendToFront(rotary_encoder_com_handle, &location, portMAX_DELAY);
        // rotary_encoder.print_data();
    }
}

void angle_task(void *arg)
{
    bool request_status = false;
    int angle;
    // ESP_LOGI("angle_task", "角度传感器任务就绪");
    while (true)
    {
        // ESP_LOGI("angle_task", "等待请求");
        xQueueReceive(angle_request_handle, &request_status, portMAX_DELAY);
        angle = vertical_position.read();
        // vertical_position.print_data();
        xQueueSendToFront(angle_com_handle, &angle, portMAX_DELAY);

        // ESP_LOGI("angle_task", "请求完成");
    }
}

void motor_task(void *arg)
{
    motor_timer_init();
    motor_channel_init();
    motor_control_init();
    motor_set_duty(100);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    motor_set_duty(-100);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // ESP_LOGI("motor_task", "电机任务就绪");
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
    // ESP_LOGI("key_task", "按键任务就绪");
    while(true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void control_task(void *arg)
{
    const char *TAG = "control_task";

    BaseType_t rotary_encoder_com_status;
    BaseType_t ADC_com_status;

    static uint16_t Count;
    static uint16_t Angle0,Angle1,Angle2;//本次，上次，上上次

    PID_t Angle_Pid;
        Angle_Pid.Target = Center_Angle;
        Angle_Pid.OutMax = 255;
        Angle_Pid.OutMin = -255;
        Angle_Pid.Kp = 0.3;
        Angle_Pid.Ki = 0.01;
        Angle_Pid.Kd = 0.4;

    //外环，位置环
    
        Location_Pid.Target = 0;
        Location_Pid.OutMax = 255;
        Location_Pid.OutMin = -255;
        Location_Pid.Kp = 0.4;
        Location_Pid.Ki = 0;
        Location_Pid.Kd = 4;

    int Angle;
    int Location;

    uint16_t count_stop_log = 0;

    bool request = true;

    while(true)
    {
        xQueueSendToFront(angle_request_handle, &request, portMAX_DELAY);
        xQueueSendToFront(rotary_encoder_request_handle, &request, portMAX_DELAY);

        rotary_encoder_com_status = xQueueReceive(rotary_encoder_com_handle, &Location, portMAX_DELAY);
        ADC_com_status = xQueueReceive(angle_com_handle, &Angle, portMAX_DELAY);
        //自动启摆程序
        switch(RunState)
        {
            case STOPPED:  // 停止状态
                motor_set_duty(0);
                count_stop_log++;
                if(count_stop_log >= 10000)
                {
                    count_stop_log = 0;
                    ESP_LOGI(TAG, "停止状态，当前角度：%d，位置：%d", Angle, Location);
                }
                
                break;
                
            case WAITING_FOR_START: // 等待起摆检测状态
                /**
                 * 检测摆杆是否到达摆动极限位置：
                 * 右侧最高点检测: 连续3个角度值都在中心角度+调控区间以上，且中间值为最低点
                 * 左侧最高点检测: 连续3个角度值都在中心角度-调控区间以下，且中间值为最高点
                 * 进入平衡条件: 连续2个角度值都在调控区间内
                 */
                ESP_LOGI(TAG, "等待状态，当前角度：%d，位置：%d", Angle, Location);
                vTaskDelay(40 / portTICK_PERIOD_MS);

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
                    RunState = SWINGING_UP_LEFT;
                }

                //判断位于左侧最高点
                if (Angle0 < Center_Angle - Center_Range
                && Angle1 < Center_Angle - Center_Range
                && Angle2 < Center_Angle - Center_Range
                && Angle1 > Angle0
                && Angle1 > Angle2)
                {
                    RunState = SWINGING_UP_RIGHT;
                }
                if (Angle0 > Center_Angle - Center_Range && Angle0 < Center_Angle + Center_Range
                && Angle1 > Center_Angle - Center_Range && Angle1 < Center_Angle + Center_Range)
                {
                    Location = 0;
                    Angle_Pid.ErrorInt = 0;
                    Location_Pid.ErrorInt = 0;
                    RunState = BALANCING;
                }     
                break;

            /**
             * RunState = 21-24: 左侧启摆过程
            */
            case SWINGING_UP_LEFT:
                motor_set_duty(START_PWM);
                RunState = SWINGING_UP_LEFT_DELAY;
                ESP_LOGI(TAG, "左侧启摆");
                break;

            case SWINGING_UP_LEFT_DELAY:    // 保持脉冲一定时间(START_TIME)
                vTaskDelay(START_TIME / portTICK_PERIOD_MS);
                RunState = SWINGING_UP_LEFT_BACK;
                ESP_LOGI(TAG, "左侧等待");
                break;

            case SWINGING_UP_LEFT_BACK:
                motor_set_duty(-START_PWM);
                RunState = SWINGING_UP_LEFT_JUDGE;
                ESP_LOGI(TAG, "左侧回转");
                break;

            case SWINGING_UP_LEFT_JUDGE:    // 保持脉冲后返回检测状态
                vTaskDelay(START_TIME / portTICK_PERIOD_MS);
                motor_set_duty(0);
                RunState = WAITING_FOR_START;
                ESP_LOGI(TAG, "左侧回转等待");
                break;

            /**
             * RunState = 31-34: 右侧启摆过程
             */
            case SWINGING_UP_RIGHT:    // 施加反向PWM脉冲
                motor_set_duty(-START_PWM);
                RunState = SWINGING_UP_RIGHT_DELAY;
                ESP_LOGI(TAG, "右侧起摆");
                break;

            case SWINGING_UP_RIGHT_DELAY:    // 保持脉冲
                vTaskDelay(START_TIME / portTICK_PERIOD_MS);
                RunState = SWINGING_UP_RIGHT_BACK;
                ESP_LOGI(TAG, "右侧等待");
                break;

            case SWINGING_UP_RIGHT_BACK:    // 施加正向PWM脉冲
                motor_set_duty(START_PWM);
                RunState = SWINGING_UP_RIGHT_JUDGE;   
                ESP_LOGI(TAG, "右侧回转");
                break;

            case SWINGING_UP_RIGHT_JUDGE:    // 保持脉冲后返回检测状态
                vTaskDelay(START_TIME / portTICK_PERIOD_MS);
                motor_set_duty(0);
                RunState = WAITING_FOR_START;
                ESP_LOGI(TAG, "右侧回转等待");
                break;

            /**
             * RunState = 4: 平衡控制状态
             */
            case BALANCING:
                ESP_LOGI(TAG, "平衡控制");
                if ( !(Center_Angle - Center_Range < Angle && Angle < Center_Angle + Center_Range) )//倒立摆不在可调控区间
                {
                    RunState = STOPPED;
                }

                Count++;
                if (Count >= 1)//设置PID调控周期
                {
                    // 角度环(内环): 5ms周期，直接控制电机维持角度平衡
                    Count = 0;
                    Angle_Pid.Actual = Angle;
                    PID_Update(&Angle_Pid);
                    motor_set_duty(Angle_Pid.Out);
                }     

                /*
                
                Count2++;
                if (Count2 >= 10)
                {
                    // 位置环(外环): 250ms周期，通过调整角度目标值来控制小车位置
                    Count2 = 0;
                    Location_Pid.Actual = Location;
                    PID_Update(&Location_Pid);
                    Angle_Pid.Target = Location_Pid.Out + Center_Angle;//位置环输出通过改变中心角度，从而实现位置的固定
                }
                
                */
                
                break;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void log_task(void *arg)    // 日志任务
{
    int Angle;
    int Location;
    while(true)
    {
        xQueueReceive(rotary_encoder_com_handle, &Location, portMAX_DELAY);
        xQueueReceive(angle_com_handle, &Angle, portMAX_DELAY);
    }
    
}

void key1_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key1", "按下，启动/停止切换");
    if (RunState == STOPPED)
    {
        RunState = SWINGING_UP_LEFT;
    }else{
        RunState = STOPPED;
      }
}

void key2_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key2", "按下，位置目标值增加（向右移动）");
    Location_Pid.Target += 408;
    if (Location_Pid.Target > 4080)
    {
    Location_Pid.Target = 4080;
    }
}

void key3_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key3", "按下，位置目标值减少（向左移动）");
    Location_Pid.Target -= 408;
    if (Location_Pid.Target < -4080)
    {
        Location_Pid.Target = -4080;
    }
}

void key4_press_cb(void *arg,void *usr_data)
{
    ESP_LOGI("key4", "按下，直接进入平衡控制模式");
    RunState = BALANCING;
}     

extern "C" void app_main(void)
{
    rotary_encoder_com_handle = xQueueCreate(1, sizeof(int));
    angle_com_handle = xQueueCreate(1, sizeof(int));
    log_runstate_com_handle = xQueueCreate(1, sizeof(runstate_t));

    rotary_encoder_request_handle = xQueueCreate(1, sizeof(bool));
    angle_request_handle = xQueueCreate(1, sizeof(bool));

    xTaskCreate(rotary_encoder_task, "rotary_encoder_task", 1024, NULL, 5, NULL);
    xTaskCreate(angle_task, "angle", 4096, NULL, 5, NULL); 
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 5, NULL);
    xTaskCreate(key_task, "key_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 8192, NULL, 10, NULL);

}