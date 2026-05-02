/**
 * @file dvc_minipc.cpp
 * @author cjw by yssickjgd
 * @brief 迷你主机
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* includes ------------------------------------------------------------------*/

#include "dvc_minipc.h"

/* private macros ------------------------------------------------------------*/

/* private types -------------------------------------------------------------*/

/* private variables ---------------------------------------------------------*/

/* private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 迷你主机初始化
 *
 * @param __frame_header 数据包头标
 * @param __frame_rear 数据包尾标
 */
void Class_MiniPC::Init(Struct_USB_Manage_Object* __USB_Manage_Object, Struct_UART_Manage_Object* __UART_Manage_Object, Struct_CAN_Manage_Object* __CAN_Manage_Object, uint8_t __frame_header, uint8_t __frame_rear)
{
	  USB_Manage_Object = __USB_Manage_Object;
    UART_Manage_Object = __UART_Manage_Object;
    CAN_Manage_Object = __CAN_Manage_Object;
    Frame_Header = __frame_header;
    Frame_Rear = __frame_rear;
}
/**
 * @brief 数据处理过程
 *
 */
int head = 0;
uint8_t temp_data[128];
int PACKET_LEN  = sizeof(Struct_MiniPC_Rx_Data);
static const uint16_t MINIPC_CAN_RX_ID = 0x106;

void Class_MiniPC::Data_Process(Enum_MiniPC_Data_Source Data_Source)
{
  if (Data_Source == CAN)
  {
    // 单ID协议: 仅处理来自上位机的 0x106
    if (CAN_Manage_Object->Rx_Buffer.Header.Identifier == MINIPC_CAN_RX_ID)
    {
      const uint8_t *can_data = CAN_Manage_Object->Rx_Buffer.Data;

      CAN_Command_Flag = can_data[0];
      CAN_Command_Speed = (int16_t)(((uint16_t)can_data[1] << 8) | can_data[2]);
      CAN_Command_Calibration = can_data[3];

      // 兼容旧接口，便于上层直接沿用现有 getter
      Data_NUC_To_MCU.Control_Type = CAN_Command_Flag;
      Data_NUC_To_MCU.MiniPC_To_Chassis_Target_Velocity_X = CAN_Command_Speed;
      Data_NUC_To_MCU.Device_Mode = CAN_Command_Calibration;
    }
  }
}

/*MINIPC解包出
  速度1
  速度2
  发射标志位
  */
/*下位机发出
  比赛阶段
  下位机是否准备好
*/


/**
 * @brief 迷你主机发送数据输出到usb发送缓冲区
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
extern Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
extern Referee_Rx_C_t CAN3_Chassis_Rx_Data_C;
extern Referee_Rx_D_t CAN3_Chassis_Rx_Data_D;
extern Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
extern Referee_Rx_F_t CAN3_Chassis_Rx_Data_F;
extern Referee_Rx_G_t CAN3_Chassis_Rx_Data_G;
extern bool Push_Calibration_Finished;
extern bool Pull_Calibration_Finished;
extern bool Referee_Allow_Shoot;
volatile int index = 0;
uint8_t  test_p = 0; 
void Class_MiniPC::Output()
{
  const uint8_t calibration_finished = (Push_Calibration_Finished && Pull_Calibration_Finished) ? 1u : 0u;
  const uint8_t allow_shoot = Referee_Allow_Shoot ? 1u : 0u;

  uint8_t game_started = 0u;
  uint8_t hatch_open = 0u;
  if (Referee != NULL)
  {
    game_started = (Referee->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE) ? 1u : 0u;
    hatch_open = (Referee->Get_Dart_Command_Status() == Referee_Data_Robot_Dart_Command_Status_OPEN) ? 1u : 0u;
  }

  // 统一CAN协议(8字节): [0..2]上位机->下位机, [3..6]下位机->上位机, [7]保留
  CAN3_MiniPC_Tx_Data_C[0] = CAN_Command_Flag;
  CAN3_MiniPC_Tx_Data_C[1] = (uint8_t)(((uint16_t)CAN_Command_Speed >> 8) & 0xff);
  CAN3_MiniPC_Tx_Data_C[2] = (uint8_t)((uint16_t)CAN_Command_Speed & 0xff);
  CAN3_MiniPC_Tx_Data_C[3] = calibration_finished;
  CAN3_MiniPC_Tx_Data_C[4] = allow_shoot;
  CAN3_MiniPC_Tx_Data_C[5] = game_started;
  CAN3_MiniPC_Tx_Data_C[6] = hatch_open;
  CAN3_MiniPC_Tx_Data_C[7] = 0x00;
}

/**
 * @brief tim定时器中断增加数据到发送缓冲区
 *
 */
void Class_MiniPC::TIM_Write_PeriodElapsedCallback()
{
  Output();
}

/**
 * @brief usb通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::USB_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(USB);
}

/**
 * @brief uart通信接收回调函数
 *
 * @param rx_data 接收的数据
 */
void Class_MiniPC::UART_RxCpltCallback(uint8_t *rx_data)
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(UART);
}

/**
 * @brief can通信接收回调函数
 *
 * @param 
 */
void Class_MiniPC::CAN_RxCpltCallback()
{
  //滑动窗口, 判断迷你主机是否在线
  Flag += 1;
  Data_Process(CAN);
}



/**
 * @brief tim定时器中断定期检测迷你主机是否存活
 *
 */
void Class_MiniPC::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    //判断该时间段内是否接收过迷你主机数据
    if (Flag == Pre_Flag)
    {
        //迷你主机断开连接
        MiniPC_Status =  MiniPC_Status_DISABLE;
    }
    else
    {
        //迷你主机保持连接
        MiniPC_Status =  MiniPC_Status_ENABLE ;
    }

    Pre_Flag = Flag;
}

/**
  * @brief CRC16 Caculation function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
  * @return : CRC16 checksum
  */
uint16_t Class_MiniPC::Get_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength, uint16_t wCRC)
{
  uint8_t ch_data;

  if (pchMessage == NULL) return 0xFFFF;
  while (dwLength--) {
    ch_data = *pchMessage++;
    wCRC = (wCRC >> 8) ^ W_CRC_TABLE[(wCRC ^ ch_data) & 0x00ff];
  }

  return wCRC;
}

uint8_t Class_MiniPC::Get_CAN_MiniPC_Frame_Sum(const uint8_t *Frame_Data) const
{
  if (Frame_Data == NULL) return 0;

  uint16_t sum = 0;
  for (uint8_t i = 0; i < 6; i++)
  {
    sum += Frame_Data[i];
  }

  return (uint8_t)sum;
}


/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */

bool Class_MiniPC::Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_expected = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return false;

  w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
  return (
    (w_expected & 0xff) == pchMessage[dwLength - 2] &&
    ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**

@brief Append CRC16 value to the end of the buffer
@param[in] pchMessage : Data to Verify,
@param[in] dwLength : Stream length = Data + checksum
@return none
*/
void Class_MiniPC::Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength)
{
  uint16_t w_crc = 0;

  if ((pchMessage == NULL) || (dwLength <= 2)) return;

  w_crc = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);

  pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
  pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}

/**
 * 计算给定向量的偏航角（yaw）。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量（未使用）
 * @return 计算得到的偏航角（以角度制表示）
 */
float Class_MiniPC::calc_yaw(float x, float y, float z) 
{
    // 使用 atan2f 函数计算反正切值，得到弧度制的偏航角
    float yaw = atan2f(y, x);

    // 将弧度制的偏航角转换为角度制
    yaw = (yaw * 180 / PI); // 向左为正，向右为负

    return yaw;
}

/**
 * 计算给定向量的欧几里德距离。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的欧几里德距离
 */
float Class_MiniPC::calc_distance(float x, float y, float z) 
{
    // 计算各分量的平方和，并取其平方根得到欧几里德距离
    float distance = sqrtf(x * x + y * y + z * z);

    return distance;
}

/**
 * 计算给定向量的俯仰角（pitch）。
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的俯仰角（以角度制表示）
 */
//extern Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
float Class_MiniPC::calc_pitch(float x, float y, float z) 
{
  // 根据 x、y 分量计算的平面投影的模长和 z 分量计算的反正切值，得到弧度制的俯仰角
  float pitch = atan2f(z, sqrtf(x * x + y * y));
  //使用重力加速度模型迭代更新俯仰角
  for (size_t i = 0; i < 20; i++) {
    float v_x = bullet_v * cosf(pitch);
    float v_y = bullet_v * sinf(pitch);
    // 计算子弹飞行时间
    float t = sqrtf(x * x + y * y) / v_x;
    float h = v_y * t - 0.5 * g * t * t;
    float dz = z - h;

    if (abs(dz) < 0.01) 
    {
      break;
    }
    // 根据 dz 和向量的欧几里德距离计算新的俯仰角的变化量，进行迭代更新
    pitch += asinf(dz / calc_distance(x, y, z));
  }

  // 将弧度制的俯仰角转换为角度制
  pitch = -(pitch * 180 / PI); // 向上为负，向下为正

  return pitch;
}

/**
 * 计算当前瞄准点与目标点的偏差 (映射到同一球面的弦长)
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @param now_yaw   实际yaw轴角度（弧度制）
 * @param now_pitch 实际pitch轴角度（弧度制）
 * @return 偏差值
 */
float Class_MiniPC::Calc_Error(float x, float y, float z, float now_yaw, float now_pitch)
{
    float dis = sqrtf(x*x + y*y + z*z);
    float x0 = dis*cos(now_pitch)*cos(now_yaw);
    float y0 = dis*cos(now_pitch)*sin(now_yaw);
    float z0 = dis*sin(now_pitch);
    float err = sqrtf(pow(x-x0,2) + pow(y-y0,2) + pow(z-z0,2));
    return err;
}

/**
 * 计算计算yaw，pitch
 * 
 * @param x 向量的x分量
 * @param y 向量的y分量
 * @param z 向量的z分量
 * @return 计算得到的目标角（以角度制表示）
 */
void Class_MiniPC::Auto_aim(float x, float y, float z, float *yaw, float *pitch, float *distance)
{
    *yaw = calc_yaw(x, y, z);//第一次标- 现改为正
    *pitch = calc_pitch(x, y, z);//第一次标- 现改为正
    *distance = calc_distance(x, y, z);
     //这里的z为上位机直接发的z，不是弹道解算后的z1，判断时存在一定误差（瞄准误差允许范围为5cm时，不影响10m内打弹）
}

float Class_MiniPC::meanFilter(float input) 
{
    static float buffer[5] = {0};
    static uint64_t index = 0;
    float sum = 0;

    // Replace the oldest value with the new input value
    buffer[index] = input;

    // Increment the index, wrapping around to the start of the array if necessary
    index = (index + 1) % 5;

    // Calculate the sum of the buffer's values
    for(int i = 0; i < 5; i++) {
        sum += buffer[i];
    }

    // Return the mean of the buffer's values
    return sum / 5.0;
}

/************************ copyright(c) ustc-robowalker **************************/
