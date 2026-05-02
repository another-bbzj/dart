/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

int PB11_GPIO = 0;
int PB10_GPIO = 0;


float test_yaw_angle_mm = 100.0f;

int enable_yaw_calibration = 0;


// PB11 中断锁存：按下一次即记住，直到状态机消费（Yaw 微动开关）
static volatile bool pb11_press_event_latched = false;
volatile uint32_t pb11_exti_irq_count = 0;
volatile uint32_t pb11_event_consumed_count = 0;

extern "C" void Gimbal_On_PB11_Exti(void)
{
    pb11_exti_irq_count++;
    pb11_press_event_latched = true;
}

bool Consume_PB11_Press_Event()
{
    __disable_irq();
    bool has_event = pb11_press_event_latched;//读取flag
    pb11_press_event_latched = false;//清零flag
    __enable_irq();
    if (has_event)
    {
        pb11_event_consumed_count++;
    }
    return has_event;
}

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief 计算Yaw丝杆位置（mm）
 * @param now_angle_deg 当前角度（度）
 * @param ref_angle_deg 校准参考角度（度）
 * @param lead_mm_per_rev 每转丝杆行程（mm）
 * @param travel_mm 总行程（mm）
 * @return 丝杆位置（mm）
 */
static float Yaw_LeadScrew_Position_mm(float now_angle_deg, float ref_angle_deg, float lead_mm_per_rev, float travel_mm)
{
    //转换逻辑：圈数 * 丝杆行程
    float position_mm = (now_angle_deg - ref_angle_deg) / 360.0f * lead_mm_per_rev;

    if (position_mm < 0.0f)
    {
        position_mm = 0.0f;
    }
    if (position_mm > travel_mm)
    {
        position_mm = travel_mm;
    }

    return position_mm;
}

/*
 * @brief 从丝杆位置更新Yaw角度
 * @return 当前Yaw轴丝杆位置（mm）
 */
float Class_Gimbal::Update_Yaw_Transform_From_Screw()
{
    float now_yaw_mm = Yaw_LeadScrew_Position_mm(Motor_Yaw.Get_Now_Angle(),
                                                 FSM_Yaw_Calibration.Angle_Ref,
                                                 Yaw_Screw_Lead_mm_per_rev,
                                                 Yaw_Screw_Total_Travel_mm);
    Motor_Yaw.Set_Transform_Angle(now_yaw_mm);
    return now_yaw_mm;
}

void Class_Gimbal::Update_MiniPC_Command()
{
    if (MiniPC == nullptr)
    {
        MiniPC_Command_Flag = 0;
        MiniPC_Command_Speed_Raw = 0;
        MiniPC_Target_Yaw_Omega = 0.0f;
        return;
    }

    MiniPC_Command_Flag = MiniPC->Get_CAN_Command_Flag();
    MiniPC_Command_Speed_Raw = MiniPC->Get_CAN_Command_Speed();

    float target_omega = (float)MiniPC_Command_Speed_Raw * MiniPC_Speed_To_Yaw_Omega_Scale;
    if (target_omega > MiniPC_Yaw_Omega_Max)
    {
        target_omega = MiniPC_Yaw_Omega_Max;
    }
    else if (target_omega < -MiniPC_Yaw_Omega_Max)
    {
        target_omega = -MiniPC_Yaw_Omega_Max;
    }

    MiniPC_Target_Yaw_Omega = target_omega;
}

/* Function prototypes -------------------------------------------------------*/
void Class_FSM_Yaw_Calibration::Yaw_Calibration_TIM_Status_PeriodElapsedCallback()
{
    
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
     case(0)://向左转
     {
        Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Gimbal->Motor_Yaw.Set_Target_Omega_Radian(-speed);
        
        // 进入该状态第一帧清除旧事件，避免跨状态误触发
        if(Status[Now_Status_Serial].Time == 1)
        {
           (void)Consume_PB11_Press_Event();
        }

        bool evt = Consume_PB11_Press_Event();
        bool pressed = (PB11_GPIO == 1); 
        //微动开关 事件+电平触发
        if (evt && pressed) 
        {
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Gimbal->Motor_Yaw.Set_Target_Omega_Radian(0.0f);
            Angle_Ref = Gimbal->Motor_Yaw.Get_Now_Angle();
            Set_Status(1);
        }
     }
     break;
     case(1)://记录这一个位置就可以了 这里就算校准完成
     {
        // 计算当前丝杆位置
        float now_yaw_mm = Gimbal->Update_Yaw_Transform_From_Screw();
        //更新PID输入值
        Gimbal->Motor_Yaw.Set_Transform_Angle(now_yaw_mm);

        // 首次进入校准完成态后切入正常控制
        if(Status[Now_Status_Serial].Time == 1)
        {
            Gimbal->Yaw_Calibrated = true;
            Gimbal->Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            Gimbal->Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Gimbal->Motor_Yaw.Set_Target_Radian(test_yaw_angle_mm);
        }
     }
     break;
    }
}

float Motor_Yaw_Omega_P_test = 1900.0f;
float Motor_Yaw_Omega_I_test = 700.0f;  
float Motor_Yaw_Omega_D_test = 0.0f;

float Motor_Yaw_Angle_P_test = 30.0f;
float Motor_Yaw_Angle_I_test = 0.0f;
float Motor_Yaw_Angle_D_test = 0.0f;

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    //Boardc_BMI.Init();

    FSM_Yaw_Calibration.Gimbal = this;
    FSM_Pitch_Calibration.Gimbal = this;

    FSM_Yaw_Calibration.Init(6,0);
    // FSM_Pitch_Calibration.Init(9,0);

    Motor_Yaw.PID_Angle.Init(Motor_Yaw_Angle_P_test, Motor_Yaw_Angle_I_test, Motor_Yaw_Angle_D_test, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Yaw.PID_Omega.Init(Motor_Yaw_Omega_P_test, Motor_Yaw_Omega_I_test, Motor_Yaw_Omega_D_test, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);
}


/**
 * @brief 输出到电机
 *
 */
// float test_yaw_omega = -5.0f;


int my_allow = 0;
int minipc_flag = 0;
int normal_to_minipc_delay_cnt = 0;

void Class_Gimbal::Output()
{
    //持续更新当前角度对应的丝杆位置
    float now_yaw_mm = Update_Yaw_Transform_From_Screw();

    //限制距离
    const float yaw_limit_guard_mm = 220.0f;

    // Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Yaw.Set_Target_Omega_Radian(test_yaw_omega);

    // Motor_Yaw.PID_Omega.Set_K_P(Motor_Yaw_Omega_P_test);
    // Motor_Yaw.PID_Omega.Set_K_I(Motor_Yaw_Omega_I_test);
    // Motor_Yaw.PID_Omega.Set_K_D(Motor_Yaw_Omega_D_test);

    // Motor_Yaw.PID_Angle.Set_K_P(Motor_Yaw_Angle_P_test);
    // Motor_Yaw.PID_Angle.Set_K_I(Motor_Yaw_Angle_I_test);
    // Motor_Yaw.PID_Angle.Set_K_D(Motor_Yaw_Angle_D_test);

/*----------------前置赋值------------------*/
    if(Yaw_Calibrated == true)//校准完成后才允许切换到MINIPC模式
    {
        if(minipc_flag == 1)
        {
            Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
        }
        else if(minipc_flag == 0)
        {
            if(Yaw_Calibrated)
            {
             Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
            }
            else
            {
                Set_Gimbal_Control_Type(Gimbal_Control_Type_YAW_UNCALIBRATION);
            }
        }
    }
    
/*---------------------------------*/

    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        normal_to_minipc_delay_cnt = 0;
        // Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        // Motor_Yaw.Set_Target_Omega_Radian(0.0f);
    }
    else if(Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
    {
        // if (minipc_flag == 0)
        // {
        //     if (normal_to_minipc_delay_cnt < 9000)
        //     {
        //         normal_to_minipc_delay_cnt++;
        //     }
        //     if (normal_to_minipc_delay_cnt >= 9000)
        //     {
        //         minipc_flag = 1;
        //     }
        // }
        // else
        // {
        //     normal_to_minipc_delay_cnt = 0;
        // }

        // if(my_allow)
        // {
        //     Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        //     Motor_Yaw.Set_Target_Radian(test_yaw_angle_mm);
        // }
        
    }
    else if (Gimbal_Control_Type == Gimbal_Control_Type_MINIPC)
    {
        normal_to_minipc_delay_cnt = 0;
        float yaw_omega_cmd = MiniPC_Yaw_Direction * MiniPC_Target_Yaw_Omega;

        if (Yaw_Calibrated)
        {
            // 将运动范围限制在左侧区间 [5, yaw_limit_guard_mm]
            if (now_yaw_mm <= 5.0f && yaw_omega_cmd < 0.0f)
            {
                yaw_omega_cmd = 0.0f;
            }
            if (now_yaw_mm >= yaw_limit_guard_mm && yaw_omega_cmd > 0.0f)
            {
                yaw_omega_cmd = 0.0f;
            }
        }

        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Yaw.Set_Target_Omega_Radian(yaw_omega_cmd);
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{

    PB11_GPIO = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET ? 1 : 0;
    PB10_GPIO = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET ? 1 : 0;

    Update_MiniPC_Command();

    if(enable_yaw_calibration == 1)
    {
        FSM_Yaw_Calibration.Yaw_Calibration_TIM_Status_PeriodElapsedCallback();
    }

    //控制模式
    Output();

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    // Motor_Pitch_L.TIM_PID_PeriodElapsedCallback();
    // Motor_Pitch_R.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
