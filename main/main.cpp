#include "rotary_encoder.hpp"
#include "adc.hpp"
#include "motor.hpp"
#include "key.hpp"
#include "PID.hpp"
#include "gpionum_setting.hpp"

TaskHandle_t motor_encoder_task_handle = NULL;
TaskHandle_t angle_task_handle = NULL;
TaskHandle_t motor_task_handle = NULL;
TaskHandle_t key_task_handle = NULL;
TaskHandle_t control_task_handle = NULL;
TaskHandle_t log_task_handle = NULL;
TaskHandle_t PIDset_task_handle = NULL;
TaskHandle_t figure_task_handle = NULL;
TaskHandle_t location_set_task_handle = NULL;

/**
 * 控制任务的通信队列
 */
QueueHandle_t motor_encoder_com_handle = NULL;      // 电机编码器通信队列
QueueHandle_t angle_com_handle = NULL;              // ADC通信队列
QueueHandle_t log_runstate_com_handle = NULL;       // 日志_状态通信队列
QueueHandle_t pid_set_com_handle = NULL;            // PID设置通信队列
QueueHandle_t figure_com_handle = NULL;             // 串口曲线打印通信队列
QueueHandle_t location_set_com_handle = NULL;   // 位置设置通信队列

/**
 * 控制任务的请求队列
 */
QueueHandle_t motor_encoder_request_handle = NULL;  // 旋转编码器请求队列
QueueHandle_t angle_request_handle = NULL;          // ADC请求队列

/**
 * 日志任务的请求队列
 */
QueueHandle_t log_request_handle = NULL;            // 日志请求队列
QueueHandle_t figure_request_handle = NULL;         // 曲线打印请求队列

ADC vertical_position;

PID_t Location_Pid; // 外环，位置环


typedef enum runstate_t
{
    STOPPED,                    // 停止状态
    WAITING_FOR_START,          // 等待起摆检测状态
    SWINGING_UP_LEFT,           // 左侧启摆
    SWINGING_UP_LEFT_DELAY,     // 左侧启摆等待
    SWINGING_UP_LEFT_BACK,      // 左侧回转
    SWINGING_UP_LEFT_JUDGE,     // 左侧回转判断

    SWINGING_UP_RIGHT,          // 右侧启摆
    SWINGING_UP_RIGHT_DELAY,    // 右侧启摆等待
    SWINGING_UP_RIGHT_BACK,     // 右侧回转
    SWINGING_UP_RIGHT_JUDGE,    // 右侧回转判断

    BALANCING                   // 平衡控制状态
} runstate_t;

typedef struct logdata_t
{
    int Angle;
    int Location;
    runstate_t State;
}logdata_t;

runstate_t RunState = STOPPED;

/**
 * @brief 电机编码器任务
 * @param[in]   来自控制任务的编码器读取请求
 * @param[out]  当前编码器位置
 */ 
void motor_encoder_task(void *arg)
{
    const char *TAG = "motor_encoder_task";
    bool request_status = false;
    int location;
    PCNT motor_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 1000, MOTOR_ENCODER_GPIO_A, MOTOR_ENCODER_GPIO_B, MOTOR_ENCODER_GPIO_B, MOTOR_ENCODER_GPIO_A, NULL, "motor_encoder");
    while (true)
    {
        xQueueReceive(motor_encoder_request_handle, &request_status, portMAX_DELAY);
        location = motor_encoder.location();
        xQueueSendToFront(motor_encoder_com_handle, &location, portMAX_DELAY);
    }
}

/**
 * @brief 角度传感器任务
 * @param[in]   来自控制任务的角度读取请求
 * @param[out]  当前角度值
 */
void angle_task(void *arg)
{
    bool request_status = false;
    int angle;
    ESP_LOGI("angle_task", "角度传感器任务就绪");
    while (true)
    {
        xQueueReceive(angle_request_handle, &request_status, portMAX_DELAY);
        angle = vertical_position.read();
        xQueueSendToFront(angle_com_handle, &angle, portMAX_DELAY);
    }
}

/**
 * @brief 电机初始化任务
 * @param[in]   无
 * @param[out]  无
 */
void motor_task(void *arg)
{
    motor_timer_init();
    motor_channel_init();
    motor_control_init();
    ESP_LOGI("motor_task", "电机任务就绪");
    while (true)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

/**
 * @brief 按键初始化任务
 * @param[in]   无
 * @param[out]  无
 */
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

/**
 * @brief 控制任务
 * @param[in]   角度传感器数据
 * @param[in]   电机编码器数据
 * @param[in]   PID参数设置
 * @param[in]   位置目标值设置
 * @param[out]  角度传感器读取请求
 * @param[out]  电机编码器读取请求
 */
void control_task(void *arg)
{
    const char *TAG = "control_task";
    constexpr int16_t Center_Angle = 2055;  //中心角度
    constexpr int16_t Center_Range = 500;   //调控区间，±500
    constexpr int16_t START_PWM = 100;       //启摆时的PWM
    constexpr uint8_t START_TIME = 100;     //启摆时的驱动时间

    static uint16_t Count;

    static uint16_t Angle0,Angle1,Angle2;//本次，上次，上上次

    PID_t Angle_Pid;    // 内环，角度环
    PID_t receive_Pid;  // 从PID设置接受的参数

        Angle_Pid.Target = Center_Angle;
        Angle_Pid.OutMax = 255;
        Angle_Pid.OutMin = -255;
        Angle_Pid.Kp = 0.5;
        Angle_Pid.Ki = 0.00035;
        Angle_Pid.Kd = 0.85;


    //外环，位置环
    
        Location_Pid.Target = 0;
        Location_Pid.OutMax = 75;
        Location_Pid.OutMin = -75;
        Location_Pid.Kp = 0.3;
        Location_Pid.Ki = 0.0;
        Location_Pid.Kd = 100;

    int Angle;
    int Location;
    float set_location_target;
    logdata_t logdata;

    uint16_t count_stop_log = 0;

    bool request = true;  

    while(true)
    {
        xQueueSend(angle_request_handle, &request, portMAX_DELAY);
        xQueueSend(motor_encoder_request_handle, &request, portMAX_DELAY);

        xQueueReceive(motor_encoder_com_handle, &Location, portMAX_DELAY);
        xQueueReceive(angle_com_handle, &Angle, portMAX_DELAY);
        xQueueReceive(location_set_com_handle, &set_location_target, 0);

        Location_Pid.Target = set_location_target;

        logdata.Angle = Angle;
        logdata.Location = Location;
        logdata.State = RunState;
        
        if(xQueueReceive(pid_set_com_handle, &receive_Pid, 0) == pdTRUE)
        {
            Angle_Pid.Kp = receive_Pid.Kp;
            Angle_Pid.Ki = receive_Pid.Ki;
            Angle_Pid.Kd = receive_Pid.Kd;
            ESP_LOGI(TAG, "更新PID参数：Kp=%f, Ki=%f, Kd=%f", Angle_Pid.Kp, Angle_Pid.Ki, Angle_Pid.Kd);
        }    

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
                Angle_Pid.Error0 = 0;
                Angle_Pid.Error1 = 0;
                Angle_Pid.ErrorInt = 0;
                Angle_Pid.Out = 0;
                Location_Pid.Error0 = 0;
                Location_Pid.Error1 = 0;
                Location_Pid.ErrorInt = 0;
                Location_Pid.Out = 0;
                break;
                
            case WAITING_FOR_START: // 等待起摆检测状态
                /**
                 * 检测摆杆是否到达摆动极限位置：
                 * 右侧最高点检测: 连续3个角度值都在中心角度+调控区间以上，且中间值为最低点
                 * 左侧最高点检测: 连续3个角度值都在中心角度-调控区间以下，且中间值为最高点
                 * 进入平衡条件: 连续2个角度值都在调控区间内
                 */
                ESP_LOGI(TAG, "等待状态，当前角度：%d，位置：%d", Angle, Location);
                vTaskDelay(20 / portTICK_PERIOD_MS);

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
                if (Angle0 > Center_Angle - 0.75*Center_Range && Angle0 < Center_Angle + 0.75*Center_Range
                && Angle1 > Center_Angle - 0.75*Center_Range && Angle1 < Center_Angle + 0.75*Center_Range)
                {   // 区间乘0.8是为了能起摆到更小的调节区间，提高起摆成功率
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
                [[fallthrough]];

            case SWINGING_UP_LEFT_DELAY:    // 保持脉冲一定时间(START_TIME)
                vTaskDelay(START_TIME / portTICK_PERIOD_MS);
                RunState = SWINGING_UP_LEFT_BACK;
                ESP_LOGI(TAG, "左侧等待");
                [[fallthrough]];

            case SWINGING_UP_LEFT_BACK:
                motor_set_duty(-START_PWM);
                RunState = SWINGING_UP_LEFT_JUDGE;
                ESP_LOGI(TAG, "左侧回转");
                [[fallthrough]];

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
                [[fallthrough]];

            case SWINGING_UP_RIGHT_DELAY:    // 保持脉冲
                vTaskDelay(START_TIME / portTICK_PERIOD_MS);
                RunState = SWINGING_UP_RIGHT_BACK;
                ESP_LOGI(TAG, "右侧等待");
                [[fallthrough]];

            case SWINGING_UP_RIGHT_BACK:    // 施加正向PWM脉冲
                motor_set_duty(START_PWM);
                RunState = SWINGING_UP_RIGHT_JUDGE;   
                ESP_LOGI(TAG, "右侧回转");
                [[fallthrough]];

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
                // ESP_LOGI(TAG, "平衡控制，角度：%d，位置：%d", Angle, Location);
                if ( !(Center_Angle - Center_Range < Angle && Angle < Center_Angle + Center_Range) )//倒立摆不在可调控区间
                {
                    RunState = STOPPED;
                    ESP_LOGI(TAG, "超出调节范围");
                }
                if(xQueueReceive(figure_request_handle, &request, 0) == pdTRUE)
                    xQueueSend(figure_com_handle, &Angle_Pid, 0);

                // 角度环(内环): 5ms周期，直接控制电机维持角度平衡
                Angle_Pid.Actual = Angle;
                PID_Update(&Angle_Pid);
                motor_set_duty(Angle_Pid.Out);  

                Count++;
                if (Count >= 10)
                {
                    // 位置环(外环): 50ms周期，通过调整角度目标值来控制位置
                    Count = 0;
                    Location_Pid.Actual = Location;
                    PID_Update(&Location_Pid);
                    Angle_Pid.Target = Center_Angle - Location_Pid.Out;//位置环输出通过改变中心角度，从而实现位置的固定
                }
            
            break;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief PID参数设置任务
 * @warning 仅调试使用，PID调节完成后可不创建该任务
 * @param[out] 更新后的pid值
 */
void PIDset_task(void *arg)
{
    PCNT kp_set_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 10*1000, 
        P_SET_ENCODER_A_GPIO_NUM, P_SET_ENCODER_B_GPIO_NUM, 
        P_SET_ENCODER_B_GPIO_NUM, P_SET_ENCODER_A_GPIO_NUM, NULL, "kp_set_encoder");
    PCNT ki_set_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 10*1000, 
        I_SET_ENCODER_A_GPIO_NUM, I_SET_ENCODER_B_GPIO_NUM, 
        I_SET_ENCODER_B_GPIO_NUM, I_SET_ENCODER_A_GPIO_NUM, NULL, "ki_set_encoder");
    PCNT kd_set_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 10*1000, 
        D_SET_ENCODER_A_GPIO_NUM, D_SET_ENCODER_B_GPIO_NUM, 
        D_SET_ENCODER_B_GPIO_NUM, D_SET_ENCODER_A_GPIO_NUM, NULL, "kd_set_encoder");

    PID_t pid_send; // 要发送的pid
    pid_send = Location_Pid;

    int last_kp_location = 0;
    int last_ki_location = 0;
    int last_kd_location = 0;

    int kp_location = 0;
    int ki_location = 0;
    int kd_location = 0;
    
    bool should_send = false;   // 是否发送

    
    while(true)
    {
        kp_location = kp_set_encoder.location();
        ki_location = ki_set_encoder.location();
        kd_location = ki_set_encoder.location();

        if(last_kp_location != kp_location)
        {  
            last_kp_location = kp_location;
            pid_send.Kp = kp_location / 200.0f;
            should_send = true;
            ESP_LOGI("PIDset_task", "更新Kp: %f", pid_send.Kp);
        }
        if(last_ki_location != ki_location)
        {  
            last_ki_location = ki_location;
            pid_send.Ki = ki_location / 200.0f;
            should_send = true;
            ESP_LOGI("PIDset_task", "更新Ki: %f", pid_send.Ki);
        }
        if(last_kd_location != kd_location)
        {  
            last_kd_location = kd_location;
            pid_send.Kd = kd_location / 200.0f;
            should_send = true;
            ESP_LOGI("PIDset_task", "更新Kd: %f", pid_send.Kd);
        }
        if(should_send)
        {
            xQueueSend(pid_set_com_handle, &pid_send, portMAX_DELAY);
            should_send = false;
        }
            
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}

/**
 * @brief 日志任务
 * @param[in]   来自控制任务的日志请求
 * @param[out]  当前系统状态日志
 */
void log_task(void *arg)    // 日志任务
{
    constexpr uint16_t LOG_INTERVAL_MS = 1000; // 日志打印间隔时间，单位毫秒
    logdata_t logdata;   

    bool request = true;
    const char *TAG = "log_task";
    char state_str[30];
    while(true)
    {
        xQueueSend(log_request_handle, &request, portMAX_DELAY);
        xQueueReceive(log_runstate_com_handle, &logdata, portMAX_DELAY);

        switch(logdata.State)
        {
            case STOPPED:
                strcpy(state_str, "停止状态");
                break;
            case WAITING_FOR_START:
                strcpy(state_str, "等待起摆检测状态");
                break;
            case SWINGING_UP_LEFT:
                strcpy(state_str, "左侧启摆");
                break;
            case SWINGING_UP_LEFT_DELAY:
                strcpy(state_str, "左侧启摆等待");
                break;
            case SWINGING_UP_LEFT_BACK:
                strcpy(state_str, "左侧回转");
                break;
            case SWINGING_UP_LEFT_JUDGE:
                strcpy(state_str, "左侧回转判断");
                break;
            case SWINGING_UP_RIGHT:
                strcpy(state_str, "右侧启摆");
                break;
            case SWINGING_UP_RIGHT_DELAY:
                strcpy(state_str, "右侧启摆等待");
                break;
            case SWINGING_UP_RIGHT_BACK:
                strcpy(state_str, "右侧回转");
                break;
            case SWINGING_UP_RIGHT_JUDGE:
                strcpy(state_str, "右侧回转判断");
                break;
            case BALANCING:
                strcpy(state_str, "平衡控制状态");
                break;
        }
        ESP_LOGI(TAG, "Angle: %d, Location: %d, State: %s", logdata.Angle, logdata.Location, state_str);
        vTaskDelay(LOG_INTERVAL_MS / portTICK_PERIOD_MS);
    }
    
}

/**
 * @brief 串口曲线打印任务
 * @param[in]   来自控制任务的曲线打印请求
 */
void figure_task(void *arg) // 串口曲线打印任务
{
    constexpr char *TAG = "figure_task";
    constexpr bool request = true;
    constexpr uint16_t FIGURE_INTERVAL_MS = 50; // 曲线打印间隔时间，单位毫秒
    PID_t angle_pid;
    while(true)
    {
        xQueueSend(figure_request_handle, &request, 0);
        xQueueReceive(figure_com_handle, &angle_pid, portMAX_DELAY);
        printf("%f,%f,%f\n", angle_pid.Target, angle_pid.Actual, angle_pid.Out);
        vTaskDelay(FIGURE_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

/**
 * @brief 位置目标值设置任务
 * @param[in]   来自控制任务的位置目标值设置请求
 * @param[out]  当前位置目标值
 */
void location_set_task(void *arg)
{
    constexpr char *TAG = "location_set_task";
    bool request = false;
    float location = 0;
    float last_location = location;
    PCNT location_set_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, 10*1000, 
        I_SET_ENCODER_A_GPIO_NUM, I_SET_ENCODER_B_GPIO_NUM, 
        I_SET_ENCODER_B_GPIO_NUM, I_SET_ENCODER_A_GPIO_NUM, NULL, "location_set_encoder");
    while(true)
    {
        location = location_set_encoder.location() * -20.0f;
        if(last_location != location)
        {
            last_location = location;
            ESP_LOGI(TAG, "位置目标值改变为：%f", location);
            xQueueSend(location_set_com_handle, &location, portMAX_DELAY);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
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
    motor_encoder_com_handle = xQueueCreate(1, sizeof(int));
    angle_com_handle = xQueueCreate(1, sizeof(int));
    log_runstate_com_handle = xQueueCreate(1, sizeof(logdata_t));
    pid_set_com_handle = xQueueCreate(1, sizeof(PID_t));
    figure_com_handle = xQueueCreate(1, sizeof(PID_t));
    location_set_com_handle = xQueueCreate(1, sizeof(float));

    motor_encoder_request_handle = xQueueCreate(1, sizeof(bool));
    angle_request_handle = xQueueCreate(1, sizeof(bool));

    log_request_handle = xQueueCreate(1, sizeof(bool));
    figure_request_handle = xQueueCreate(1, sizeof(bool));

    xTaskCreate(motor_encoder_task, "motor_encoder_task", 4096, NULL, 9, NULL);
    xTaskCreate(angle_task, "angle", 4096, NULL, 9, NULL); 
    xTaskCreate(motor_task, "motor_task", 4096, NULL, 9, NULL);
    xTaskCreate(key_task, "key_task", 4096, NULL, 9, NULL);
    xTaskCreate(control_task, "control_task", 8192, NULL, 10, NULL);
    xTaskCreate(log_task, "log_task", 8192, NULL, 5, NULL);
    xTaskCreate(PIDset_task, "PIDset_task", 8192, NULL, 5, NULL);
    // xTaskCreate(figure_task, "figure_task", 4096, NULL, 5, NULL);
    // xTaskCreate(location_set_task, "location_set_task", 4096, NULL, 5, NULL);
}