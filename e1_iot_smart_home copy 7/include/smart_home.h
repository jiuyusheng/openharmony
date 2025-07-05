/*
 * Copyright (c) 2024 iSoftStone Education Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __SMART_HOME_H__
#define __SMART_HOME_H__

#include <stdint.h>
#include <stdbool.h>

// 全局环境数据变量
extern double g_temp;      // 温度 (°C)
extern double g_humi;      // 湿度 (%)
extern double g_lux;       // 光照强度 (Lux)
extern double g_gas;       // 气体浓度 (ppm)
extern bool g_pir_state;   // PIR人体检测状态
extern double g_accel_x;   // X轴加速度 (g)
extern double g_accel_y;   // Y轴加速度 (g)
extern double g_accel_z;   // Z轴加速度 (g)
extern bool has_person;

// 报警阈值设置
#define TEMP_ALARM_THRESHOLD    35.0    // 温度报警阈值 (°C)
#define HUMI_ALARM_THRESHOLD    80.0    // 湿度报警阈值 (%)
#define LUX_ALARM_THRESHOLD_LOW     50.0    // 光照报警阈值下限 (Lux)
#define LUX_ALARM_THRESHOLD_HIGH    300.0   // 光照报警阈值上限 (Lux)
#define GAS_ALARM_THRESHOLD     100.0   // 气体报警阈值 (ppm)
#define ACCEL_ALARM_THRESHOLD_HUMAN  1.0    // 有人时震动报警阈值 (g)
#define ACCEL_ALARM_THRESHOLD_EMPTY  2.0    // 无人时震动报警阈值 (g)

// 报警状态
extern bool g_temp_alarm;   // 温度报警状态
extern bool g_humi_alarm;   // 湿度报警状态
extern bool g_lux_alarm;    // 光照报警状态
extern bool g_gas_alarm;    // 气体报警状态
extern bool g_pir_alarm;    // PIR报警状态
extern bool g_accel_alarm;  // 震动报警状态

// 手动优先标志
extern bool motor_manual_override;  // 电机手动优先标志

// 传感器初始化函数
void i2c_dev_init(void);
void mq2_dev_init(void);
void pir_dev_init(void);
void mpu6050_dev_init(void);

// 传感器数据读取函数
void bh1750_read_data(double *dat);
void sht30_read_data(double *temp, double *humi);
void mq2_read_data(double *gas_ppm);
bool pir_read_state(void);
void mpu6050_read_data(double *accel_x, double *accel_y, double *accel_z);

// 报警检测函数
void check_alarm_conditions(void);
void clear_all_alarms(void);
bool is_any_alarm_active(void);

// 执行器控制函数
void light_dev_init(void);
void light_set_pwm(unsigned int duty);
void light_set_state(bool state);
bool get_light_state(void);

void motor_dev_init(void);
void motor_set_pwm(unsigned int duty);
void motor_set_state(bool state);
bool get_motor_state(void);

void buzzer_dev_init(void);
void buzzer_set_frequency(unsigned int freq);
void buzzer_set_state(bool state);

void alarm_led_init(void);
void alarm_led_set_state(bool state);

// LCD显示函数
void lcd_dev_init(void);
void lcd_show_ui(void);
void lcd_set_temperature(double temperature);
void lcd_set_humidity(double humidity);
void lcd_set_illumination(double illumination);
void lcd_set_gas_concentration(double gas_ppm);
void lcd_set_pir_state(bool state);
void lcd_set_accel_data(double x, double y, double z);
void lcd_set_light_state(bool state);
void lcd_set_motor_state(bool state);
void lcd_set_auto_state(bool state);
void lcd_set_alarm_state(bool alarm_active);

// 命令处理函数
void smart_home_su03t_cmd_process(int su03t_cmd);
void smart_home_iot_cmd_process(int iot_cmd);
void smart_home_key_press_process(uint8_t key_no);

// 数据存储函数
void save_environment_data(void);
void load_environment_data(void);

#endif