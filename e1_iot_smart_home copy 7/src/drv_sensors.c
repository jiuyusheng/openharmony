#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "drv_sensors.h"
#include "smart_home.h"
#include "iot_i2c.h"
#include "iot_adc.h"
#include "iot_gpio.h"
#include "stdint.h"
#include "iot_errno.h"
#include <math.h>

#define I2C_HANDLE EI2C0_M2
#define SHT30_I2C_ADDRESS 0x44
#define BH1750_I2C_ADDRESS 0x23
#define MPU6050_I2C_ADDRESS 0x68

// ADC通道定义
#define MQ2_ADC_CHANNEL 0  // MQ2气体传感器ADC通道

// GPIO定义
#define PIR_GPIO_PIN 3     // PIR人体红外传感器GPIO引脚，A3（GPIO0_PA3）

// MPU6050寄存器地址
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_ACCEL_YOUT_H  0x3D
#define MPU6050_ACCEL_ZOUT_H  0x3F
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C

// MQ2校准参数
static double mq2_ro_clean_air = 76.63;  // 清洁空气中的传感器电阻值

// 全局变量定义
double g_temp = 25.0;
double g_humi = 50.0;
double g_lux = 100.0;
double g_gas = 0.0;
bool g_pir_state = false;
double g_accel_x = 0.0;
double g_accel_y = 0.0;
double g_accel_z = 0.0;

// 报警状态
bool g_temp_alarm = false;
bool g_humi_alarm = false;
bool g_lux_alarm = false;
bool g_gas_alarm = false;
bool g_pir_alarm = false;
bool g_accel_alarm = false;

/***************************************************************
 * 函数名称: sht30_init
 * 说    明: sht30初始化
 * 参    数: 无
 * 返 回 值: uint32_t IOT_SUCCESS表示成功 IOT_FAILURE表示失败
 ***************************************************************/
static uint32_t sht30_init(void)
{
    uint32_t ret = 0;
    uint8_t send_data[2] = {0x22, 0x36};
    uint32_t send_len = 2;

    ret = IoTI2cWrite(I2C_HANDLE, SHT30_I2C_ADDRESS, send_data, send_len); 
    if (ret != IOT_SUCCESS)
    {
        printf("SHT30 I2c write failure.\r\n");
        return IOT_FAILURE;
    }

    return IOT_SUCCESS;
}

/***************************************************************
 * 函数名称: bh1750_init
 * 说    明: bh1750初始化
 * 参    数: 无
 * 返 回 值: uint32_t IOT_SUCCESS表示成功 IOT_FAILURE表示失败
 ***************************************************************/
static uint32_t bh1750_init(void)
{
    uint32_t ret = 0;
    uint8_t send_data[1] = {0x10};
    uint32_t send_len = 1;

    ret = IoTI2cWrite(I2C_HANDLE, BH1750_I2C_ADDRESS, send_data, send_len); 
    if (ret != IOT_SUCCESS)
    {
        printf("BH1750 I2c write failure.\r\n");
        return IOT_FAILURE;
    }

    return IOT_SUCCESS;
}

/***************************************************************
 * 函数名称: mpu6050_init
 * 说    明: MPU6050初始化
 * 参    数: 无
 * 返 回 值: uint32_t IOT_SUCCESS表示成功 IOT_FAILURE表示失败
 ***************************************************************/
static uint32_t mpu6050_init(void)
{
    uint32_t ret = 0;
    uint8_t send_data[2];
    
    // 唤醒MPU6050
    send_data[0] = MPU6050_PWR_MGMT_1;
    send_data[1] = 0x00;
    ret = IoTI2cWrite(I2C_HANDLE, MPU6050_I2C_ADDRESS, send_data, 2);
    if (ret != IOT_SUCCESS) {
        printf("MPU6050 wake up failure.\r\n");
        return IOT_FAILURE;
    }
    
    // 设置加速度计量程为±2g
    send_data[0] = MPU6050_ACCEL_CONFIG;
    send_data[1] = 0x00;
    ret = IoTI2cWrite(I2C_HANDLE, MPU6050_I2C_ADDRESS, send_data, 2);
    if (ret != IOT_SUCCESS) {
        printf("MPU6050 accel config failure.\r\n");
        return IOT_FAILURE;
    }
    
    // 设置低通滤波器
    send_data[0] = MPU6050_CONFIG;
    send_data[1] = 0x06;
    ret = IoTI2cWrite(I2C_HANDLE, MPU6050_I2C_ADDRESS, send_data, 2);
    if (ret != IOT_SUCCESS) {
        printf("MPU6050 config failure.\r\n");
        return IOT_FAILURE;
    }
    
    printf("MPU6050 init success.\r\n");
    return IOT_SUCCESS;
}

/***************************************************************
 * 函数名称: mq2_dev_init
 * 说    明: MQ2气体传感器初始化
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
void mq2_dev_init(void)
{
    // 初始化ADC
    IoTAdcInit(MQ2_ADC_CHANNEL);
    printf("MQ2 sensor init success.\r\n");
}

/***************************************************************
 * 函数名称: pir_dev_init
 * 说    明: PIR人体红外传感器初始化
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
void pir_dev_init(void)
{
    IoTGpioInit(PIR_GPIO_PIN);
    IoTGpioSetDir(PIR_GPIO_PIN, IOT_GPIO_DIR_IN);
    // IoTGpioSetPull(PIR_GPIO_PIN, IOT_GPIO_PULL_UP); // 你的SDK没有此函数，注释掉
    printf("PIR sensor init success on GPIO%d.\r\n", PIR_GPIO_PIN);
}

/***************************************************************
 * 函数名称: mpu6050_dev_init
 * 说    明: MPU6050震动传感器初始化
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
void mpu6050_dev_init(void)
{
    mpu6050_init();
}

/***************************************************************
* 函数名称: sht30_calc_RH
* 说    明: 湿度计算
* 参    数: u16sRH：读取到的湿度原始数据
* 返 回 值: 计算后的湿度数据
***************************************************************/
static float sht30_calc_RH(uint16_t u16sRH)
{
    float humidityRH = 0;

    /*clear bits [1..0] (status bits)*/
    u16sRH &= ~0x0003;
    /*calculate relative humidity [%RH]*/
    /*RH = rawValue / (2^16-1) * 10*/
    humidityRH = (100 * (float)u16sRH / 65535);

    return humidityRH;
}

/***************************************************************
* 函数名称: sht30_calc_temperature
* 说    明: 温度计算
* 参    数: u16sT：读取到的温度原始数据
* 返 回 值: 计算后的温度数据
***************************************************************/
static float sht30_calc_temperature(uint16_t u16sT)
{
    float temperature = 0;

    /*clear bits [1..0] (status bits)*/
    u16sT &= ~0x0003;
    /*calculate temperature [℃]*/
    /*T = -45 + 175 * rawValue / (2^16-1)*/
    temperature = (175 * (float)u16sT / 65535 - 45);

    return temperature;
}

/***************************************************************
* 函数名称: sht30_check_crc
* 说    明: 检查数据正确性
* 参    数: data：读取到的数据
            nbrOfBytes：需要校验的数量
            checksum：读取到的校对比验值
* 返 回 值: 校验结果，0-成功 1-失败
***************************************************************/
static uint8_t sht30_check_crc(uint8_t *data, uint8_t nbrOfBytes, uint8_t checksum)
{
    uint8_t crc = 0xFF;
    uint8_t bit = 0;
    uint8_t byteCtr ;
    const int16_t POLYNOMIAL = 0x131;

    /*calculates 8-Bit checksum with given polynomial*/
    for(byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
    {
        crc ^= (data[byteCtr]);
        for ( bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else crc = (crc << 1);
        }
    }

    if(crc != checksum)
        return 1;
    else
        return 0;
}

/***************************************************************
* 函数名称: sht30_read_data
* 说    明: 读取温度、湿度
* 参    数: temp,humi：读取到的数据,通过指针返回 
* 返 回 值: 无
***************************************************************/
void sht30_read_data(double *temp, double *humi)
{
    /*checksum verification*/
    uint8_t data[3];
    uint16_t tmp;
    uint8_t rc;
    /*byte 0,1 is temperature byte 4,5 is humidity*/
    uint8_t SHT30_Data_Buffer[6];
    memset(SHT30_Data_Buffer, 0, 6);
    uint8_t send_data[2] = {0xE0, 0x00};

    uint32_t send_len = 2;
    IoTI2cWrite(I2C_HANDLE, SHT30_I2C_ADDRESS, send_data, send_len);

    uint32_t receive_len = 6;
    IoTI2cRead(I2C_HANDLE, SHT30_I2C_ADDRESS, SHT30_Data_Buffer, receive_len);

    /*check temperature*/
    data[0] = SHT30_Data_Buffer[0];
    data[1] = SHT30_Data_Buffer[1];
    data[2] = SHT30_Data_Buffer[2];
    rc = sht30_check_crc(data, 2, data[2]);
    if(!rc)
    {
        tmp = ((uint16_t)data[0] << 8) | data[1];
        *temp = sht30_calc_temperature(tmp);
    }
    
    /*check humidity*/
    data[0] = SHT30_Data_Buffer[3];
    data[1] = SHT30_Data_Buffer[4];
    data[2] = SHT30_Data_Buffer[5];
    rc = sht30_check_crc(data, 2, data[2]);
    if(!rc)
    {
        tmp = ((uint16_t)data[0] << 8) | data[1];
        *humi = sht30_calc_RH(tmp);
    }
}

/***************************************************************
* 函数名称: bh1750_read_data
* 说    明: 读取光照强度
* 参    数: dat：读取到的数据
* 返 回 值: 无
***************************************************************/
void bh1750_read_data(double *dat)
{
    uint8_t send_data[1] = {0x10};
    uint32_t send_len = 1;

    IoTI2cWrite(I2C_HANDLE, BH1750_I2C_ADDRESS, send_data, send_len); 

    uint8_t recv_data[2] = {0};
    uint32_t receive_len = 2;   

    IoTI2cRead(I2C_HANDLE, BH1750_I2C_ADDRESS, recv_data, receive_len);
    *dat = (float)(((recv_data[0] << 8) + recv_data[1]) / 1.2);
}

/***************************************************************
* 函数名称: mq2_read_data
* 说    明: 读取MQ2气体传感器数据
* 参    数: gas_ppm：气体浓度(ppm)
* 返 回 值: 无
***************************************************************/
void mq2_read_data(double *gas_ppm)
{
    uint32_t adc_value = 0;
    double voltage = 0.0;
    double rs = 0.0;
    double ratio = 0.0;
    // 读取ADC值
    IoTAdcGetVal(MQ2_ADC_CHANNEL, &adc_value);
    // 转换为电压值 (假设ADC参考电压为3.3V，12位ADC)
    voltage = (double)adc_value * 3.3 / 4095.0;
    // 计算传感器电阻值 (假设负载电阻为10K)
    rs = (3.3 - voltage) / voltage * 10000.0;
    // 计算与清洁空气的比值
    ratio = rs / mq2_ro_clean_air;
    // 用线性近似代替pow(ratio, -1.409) * 1000.0
    // 经验公式：ppm ≈ (1/ratio) * 1000
    if (ratio > 0.0001)
        *gas_ppm = (1.0 / ratio) * 1000.0;
    else
        *gas_ppm = 0;
    // 限制范围
    if (*gas_ppm < 0) *gas_ppm = 0;
    if (*gas_ppm > 10000) *gas_ppm = 10000;
}

/***************************************************************
* 函数名称: pir_read_state
* 说    明: 读取PIR人体红外传感器状态
* 参    数: 无
* 返 回 值: bool true表示检测到人体，false表示未检测到
***************************************************************/
bool pir_read_state(void)
{
    unsigned int gpio_value = 0;
    IoTGpioGetInputVal(PIR_GPIO_PIN, &gpio_value);
    // 调试输出：每10次读取输出一次PIR状态
    static int debug_count = 0;
    debug_count++;
    if (debug_count >= 10) {
        printf("PIR GPIO%d = %d\r\n", PIR_GPIO_PIN, gpio_value);
        debug_count = 0;
    }
    return (gpio_value == 1);
}

/***************************************************************
* 函数名称: mpu6050_read_data
* 说    明: 读取MPU6050三轴加速度数据
* 参    数: accel_x, accel_y, accel_z：三轴加速度值(g)
* 返 回 值: 无
***************************************************************/
void mpu6050_read_data(double *accel_x, double *accel_y, double *accel_z)
{
    uint8_t send_data[1];
    uint8_t recv_data[6];
    int16_t raw_x, raw_y, raw_z;
    
    // 读取加速度数据
    send_data[0] = MPU6050_ACCEL_XOUT_H;
    IoTI2cWrite(I2C_HANDLE, MPU6050_I2C_ADDRESS, send_data, 1);
    IoTI2cRead(I2C_HANDLE, MPU6050_I2C_ADDRESS, recv_data, 6);
    
    // 组合16位数据
    raw_x = (recv_data[0] << 8) | recv_data[1];
    raw_y = (recv_data[2] << 8) | recv_data[3];
    raw_z = (recv_data[4] << 8) | recv_data[5];
    
    // 转换为g值 (±2g量程)
    *accel_x = (double)raw_x / 16384.0;
    *accel_y = (double)raw_y / 16384.0;
    *accel_z = (double)raw_z / 16384.0;
}

/***************************************************************
* 函数名称: check_alarm_conditions
* 说    明: 检查所有报警条件
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void check_alarm_conditions(void)
{
    // 温度报警检测
    g_temp_alarm = (g_temp > TEMP_ALARM_THRESHOLD);
    
    // 湿度报警检测
    g_humi_alarm = (g_humi > HUMI_ALARM_THRESHOLD);
    
    // 光照报警检测 (小于50Lux或大于300Lux)
    g_lux_alarm = (g_lux < LUX_ALARM_THRESHOLD_LOW || g_lux > LUX_ALARM_THRESHOLD_HIGH);
    
    // 气体报警检测
    g_gas_alarm = (g_gas > GAS_ALARM_THRESHOLD);
    
    // PIR报警检测 (连续30秒有人才报警)
    static int pir_count = 0;
    if (g_pir_state) {
        pir_count++;
        if (pir_count >= 30) {  // 连续30次（约30秒）才报警
            g_pir_alarm = true;
            printf("PIR报警触发！检测到人体（30秒）\r\n");
        }
    } else {
        if (pir_count > 0) {
            printf("PIR状态：人体离开\r\n");
        }
        pir_count = 0;
        g_pir_alarm = false;
    }
    
    // 震动报警检测 (根据是否有人动态调整阈值)
    double accel_magnitude = sqrt(g_accel_x * g_accel_x + g_accel_y * g_accel_y + g_accel_z * g_accel_z);
    double threshold = g_pir_state ? 1.0 : 2.0; // 有人时1报警，无人时2报警
    g_accel_alarm = (accel_magnitude > threshold);
}

/***************************************************************
* 函数名称: clear_all_alarms
* 说    明: 清除所有报警状态
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void clear_all_alarms(void)
{
    g_temp_alarm = false;
    g_humi_alarm = false;
    g_lux_alarm = false;
    g_gas_alarm = false;
    g_pir_alarm = false;
    g_accel_alarm = false;
}

/***************************************************************
* 函数名称: is_any_alarm_active
* 说    明: 检查是否有任何报警激活
* 参    数: 无
* 返 回 值: bool true表示有报警，false表示无报警
***************************************************************/
bool is_any_alarm_active(void)
{
    return (g_temp_alarm || g_humi_alarm || g_lux_alarm || 
            g_gas_alarm || g_pir_alarm || g_accel_alarm);
}

/***************************************************************
* 函数名称: i2c_dev_init
* 说    明: i2c设备初始化
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void i2c_dev_init(void)
{
    IoTI2cInit(I2C_HANDLE, EI2C_FRE_400K);
    sht30_init();
    bh1750_init();
    mpu6050_init();
    printf("All I2C sensors init success.\r\n");
}

