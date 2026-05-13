#include "bsp_imu.h"
#include "spi.h"

// ==================== 寄存器地址定义 ====================
#define MPU6500_PWR_MGMT_1       0x6B
#define MPU6500_PWR_MGMT_2       0x6C
#define MPU6500_CONFIG           0x1A
#define MPU6500_GYRO_CONFIG      0x1B
#define MPU6500_ACCEL_CONFIG     0x1C
#define MPU6500_ACCEL_CONFIG_2   0x1D
#define MPU6500_USER_CTRL        0x6A
#define MPU6500_SIGNAL_PATH_RESET 0x68
#define MPU6500_WHO_AM_I         0x75
#define MPU6500_ACCEL_XOUT_H     0x3B
#define MPU6500_ID               0x70

// ==================== SPI 硬件操作宏 ====================
extern SPI_HandleTypeDef hspi5;
#define MPU_HSPI hspi5
#define MPU_NSS_LOW  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

// 全局变量
imu_data_t imu_data;

// 内部使用的原始数据与零偏结构体
typedef struct {
    int32_t gx_offset, gy_offset, gz_offset;
} mpu_offset_t;

static mpu_offset_t mpu_offset;
static uint8_t tx, rx;
static uint8_t mpu_buff[14];

// ==================== 底层通信函数 ====================

static uint8_t mpu_write_reg(uint8_t const reg, uint8_t const data) {
    MPU_NSS_LOW;
    tx = reg & 0x7F; // 写操作，最高位为 0
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

static uint8_t mpu_read_reg(uint8_t const reg) {
    MPU_NSS_LOW;
    tx = reg | 0x80; // 读操作，最高位为 1
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

static uint8_t mpu_read_regs(uint8_t const regAddr, uint8_t *pData, uint8_t len) {
    MPU_NSS_LOW;
    tx = regAddr | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, pData, pData, len, 55); // 注意这里用 dummy 覆盖发送
    MPU_NSS_HIGH;
    return 0;
}

// ==================== 上电校准函数 ====================
#define ZERO_LEN 300
static void mpu_offset_cal(void) {
    int i;
    mpu_offset.gx_offset = 0;
    mpu_offset.gy_offset = 0;
    mpu_offset.gz_offset = 0;

    for (i = 0; i < ZERO_LEN; i++) {
        mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
        
        // 只累加陀螺仪数据求零偏
        mpu_offset.gx_offset += (int16_t)(mpu_buff[8] << 8 | mpu_buff[9]);
        mpu_offset.gy_offset += (int16_t)(mpu_buff[10] << 8 | mpu_buff[11]);
        mpu_offset.gz_offset += (int16_t)(mpu_buff[12] << 8 | mpu_buff[13]);
        
        HAL_Delay(2);
    }
    
    // 【已修复原代码的 BUG】
    mpu_offset.gx_offset = mpu_offset.gx_offset / ZERO_LEN;
    mpu_offset.gy_offset = mpu_offset.gy_offset / ZERO_LEN; 
    mpu_offset.gz_offset = mpu_offset.gz_offset / ZERO_LEN;
}

// ==================== 接口：初始化 ====================
uint8_t BSP_IMU_Init(void) {
    uint8_t i = 0;
    uint8_t MPU6500_Init_Data[7][2] = {
        { MPU6500_PWR_MGMT_1,     0x03 }, // 时钟源：Z轴陀螺仪锁相环
        { MPU6500_PWR_MGMT_2,     0x00 }, // 全部使能
        { MPU6500_CONFIG,         0x00 }, // 陀螺仪带宽 250Hz
        { MPU6500_GYRO_CONFIG,    0x10 }, // 陀螺仪量程 +-1000dps (对应 32.8 LSB/dps)
        { MPU6500_ACCEL_CONFIG,   0x10 }, // 加速度计量程 +-8G (对应 4096 LSB/g)
        { MPU6500_ACCEL_CONFIG_2, 0x00 }, // 加速度计带宽 250Hz
        { MPU6500_USER_CTRL,      0x10 }, // 【关键】禁用 I2C 接口，仅使用 SPI
    };

    // 1. 硬件复位
    mpu_write_reg(MPU6500_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    mpu_write_reg(MPU6500_SIGNAL_PATH_RESET, 0x07);
    HAL_Delay(100);

    // 2. 身份校验防呆！
    if (MPU6500_ID != mpu_read_reg(MPU6500_WHO_AM_I)) {
        return 1; // 初始化失败（SPI不通或者芯片坏了）
    }
    
    // 3. 写入参数
    for (i = 0; i < 7; i++) {
        mpu_write_reg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        HAL_Delay(5);
    }
    
    // 4. 陀螺仪静态零偏校准 (耗时约 600ms，开机时请勿晃动云台！)
    mpu_offset_cal();
    
    return 0; // 成功
}

// ==================== 接口：高频更新 ====================
void BSP_IMU_Update(void) {
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    // 连续读取 14 字节
    mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    // 拼接数据
    raw_ax = (mpu_buff[0] << 8) | mpu_buff[1];
    raw_ay = (mpu_buff[2] << 8) | mpu_buff[3];
    raw_az = (mpu_buff[4] << 8) | mpu_buff[5];
    
    // 【关键】扣除开机时校准出的零偏，消除静态漂移！
    raw_gx = ((mpu_buff[8] << 8) | mpu_buff[9])   - mpu_offset.gx_offset;
    raw_gy = ((mpu_buff[10] << 8) | mpu_buff[11]) - mpu_offset.gy_offset;
    raw_gz = ((mpu_buff[12] << 8) | mpu_buff[13]) - mpu_offset.gz_offset;

    // 1. 加速度转换 (g): +-8g 对应 4096 LSB/g
    imu_data.ax = (float)raw_ax / 4096.0f; 
    imu_data.ay = (float)raw_ay / 4096.0f;
    imu_data.az = (float)raw_az / 4096.0f;

    // 2. 陀螺仪转换 (rad/s)
    // 根据 +-1000dps 的量程: 1 dps = 32.8 LSB. 
    // 所以: (raw / 32.8) 得到 dps（度/秒）
    // 再乘 (PI / 180) 即约除以 57.3f，转换为 rad/s 给 Mahony 解算
    imu_data.gx = ((float)raw_gx) / 32.8f / 57.3f;
    imu_data.gy = ((float)raw_gy) / 32.8f / 57.3f;
    imu_data.gz = ((float)raw_gz) / 32.8f / 57.3f;
}