
#include "bsp_sbus.h"
#include "bsp.h"
#include "bsp_motion.h"
#include "string.h"


#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00

// Parameters related to receiving data  接收数据相关参数
uint8_t sbus_start = 0;
uint8_t sbus_buf_index = 0;
uint8_t sbus_new_cmd = 0;

// data-caching mechanism  数据缓存
uint8_t inBuffer[SBUS_RECV_MAX] = {0};
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t sbus_data[SBUS_RECV_MAX] = {0};
int16_t g_sbus_channels[18] = {0};
#define RC_MODE_THRESHOLD 1000   // 阈值，根据你开关的三段数值调整


ctrl_mode_t g_ctrl_mode = CTRL_MODE_PC;

// Parses SBUS data into channel values  解析SBUS的数据，转化成通道数值。
static int SBUS_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif

    // 安全检测，检测是否失联或者数据错误
    // Security detection to check for lost connections or data errors
    failsafe_status = SBUS_SIGNAL_OK;
    if (sbus_data[23] & (1 << 2))
    {
        failsafe_status = SBUS_SIGNAL_LOST;
        //printf("SBUS_SIGNAL_LOST\n");
        // lost contact errors  遥控器失联错误
    }
    else if (sbus_data[23] & (1 << 3))
    {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
        //printf("SBUS_SIGNAL_FAILSAFE\n");
        // data loss error  数据丢失错误
    }
    return failsafe_status;
}

// Receives SBUS cache data  接收SBUS的缓存数据
void SBUS_Reveive(uint8_t data)
{
    // If the protocol start flag is met, data is received  如果符合协议开始标志，则开始接收数据
    if (sbus_start == 0 && data == SBUS_START)
    {
        sbus_start = 1;
        sbus_new_cmd = 0;
        sbus_buf_index = 0;
        inBuffer[sbus_buf_index] = data;
        inBuffer[SBUS_RECV_MAX - 1] = 0xff;
    }
    else if (sbus_start)
    {
        sbus_buf_index++;
        inBuffer[sbus_buf_index] = data;
    }

    // Finish receiving a frame of data  完成接收一帧数据
    if (sbus_start & (sbus_buf_index >= (SBUS_RECV_MAX - 1)))
    {
        sbus_start = 0;
        if (inBuffer[SBUS_RECV_MAX - 1] == SBUS_END)
        {
            memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
            sbus_new_cmd = 1;
        }
    }
}

// SBUS receives and processes data handle  SBUS接收处理数据句柄
int16_t MapChannel2ToVz(uint16_t ch2_value)
{
    const int16_t vz_max = 500;    // 最大转角速度
    const int16_t deadzone = 50;   // 中立区阈值
    const int16_t center = 992;    // 中立值

    int16_t vz = 0;

    // 中立附近不动
    if (ch2_value > center - deadzone && ch2_value < center + deadzone)
        return 0;

    // 上下极限映射到 -vz_max ~ +vz_max
    if (ch2_value > center)
        vz = ((int32_t)(ch2_value - center) * vz_max) / (1809 - center);
    else
        vz = ((int32_t)(ch2_value - center) * vz_max) / (center - 181);

    return vz;
}

int16_t MapChannel3ToVx(uint16_t ch3_value)
{
    const int16_t vx_max = 500;    // 最大速度 mm/s
    const int16_t deadzone = 50;   // 中立区阈值
    const int16_t center = 997;    // 中立值

    // 中立附近不动
    if (ch3_value > center - deadzone && ch3_value < center + deadzone)
        return 0;

    // 上下极限映射到 -vx_max ~ +vx_max
    int16_t vx = 0;
    if (ch3_value > center)
        vx = ((int32_t)(ch3_value - center) * vx_max) / (1798 - center);
    else
        vx = ((int32_t)(ch3_value - center) * vx_max) / (center - 173);

    return vx;
}


void SBUS_Handle(void)
{

    if (sbus_new_cmd)
    {
        int res = SBUS_Parse_Data();
        sbus_new_cmd = 0;
        if (res) return;
        #if SBUS_ALL_CHANNELS
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
			   g_sbus_channels[3], g_sbus_channels[4], g_sbus_channels[5],
			   g_sbus_channels[6], g_sbus_channels[7], g_sbus_channels[8],
			   g_sbus_channels[9], g_sbus_channels[10], g_sbus_channels[11],
			   g_sbus_channels[12], g_sbus_channels[13], g_sbus_channels[14],
			   g_sbus_channels[15]);
        #else
        if (g_sbus_channels[4] < RC_MODE_THRESHOLD) {
            g_ctrl_mode = CTRL_MODE_PC;   // 遥控器控制
        } else {
            g_ctrl_mode = CTRL_MODE_RC;   // 上位机控制
        }
        if (g_ctrl_mode == CTRL_MODE_RC)
        {
            int16_t vx = MapChannel3ToVx(g_sbus_channels[1]); // 前后速度
            int16_t vz = MapChannel2ToVz(g_sbus_channels[3]); // 左右转角
            int16_t vy = 0; // 横向速度，如果底盘支持可映射

            Motion_Ctrl(vx, vy, vz);
        }

      //   printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
      //          g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
			   // g_sbus_channels[3], g_sbus_channels[4],g_sbus_channels[5],
			   // g_sbus_channels[6], g_sbus_channels[7]);
        #endif
    }
}

