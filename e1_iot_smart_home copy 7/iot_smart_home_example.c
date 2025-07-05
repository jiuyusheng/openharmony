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

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "los_task.h"
#include "ohos_init.h"
#include "cmsis_os.h"
#include "config_network.h"
#include "smart_home.h"
#include "smart_home_event.h"
#include "su_03t.h"
#include "iot.h"
#include "lcd.h"
#include "picture.h"
#include "adc_key.h"
#include "iot_gpio.h"
#include "iot_pwm.h"

/* RGB对应的PWM通道 */
#define LED_R_PORT EPWMDEV_PWM1_M1
#define LED_G_PORT EPWMDEV_PWM7_M1
#define LED_B_PORT EPWMDEV_PWM0_M1

#define ROUTE_SSID      "jiu"          // WiFi账号
#define ROUTE_PASSWORD "1023174147"       // WiFi密码

#define MSG_QUEUE_LENGTH                                16
#define BUFFER_LEN                                      50

// 函数声明补全
void smart_home_key_process(int key_no);
void buzzer_alarm_pattern(int pattern);
void alarm_led_blink_pattern(int pattern);
void alarm_led_update(void);
void lcd_set_network_state(bool state);

extern bool motor_manual_override;

/***************************************************************
 * 函数名称: iot_thread
 * 说    明: iot线程
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
void iot_thread(void *args) {
  uint8_t mac_address[12] = {0x00, 0xdc, 0xb6, 0x90, 0x01, 0x00,0};

  char ssid[32]=ROUTE_SSID;
  char password[32]=ROUTE_PASSWORD;
  char mac_addr[32]={0};

  FlashDeinit();
  FlashInit();

  VendorSet(VENDOR_ID_WIFI_MODE, (unsigned char*)"STA", 3); // 配置为Wifi STA模式
  VendorSet(VENDOR_ID_MAC, mac_address, 6); // 多人同时做该实验，请修改各自不同的WiFi MAC地址
  VendorSet(VENDOR_ID_WIFI_ROUTE_SSID, (unsigned char*)ssid, sizeof(ssid));
  VendorSet(VENDOR_ID_WIFI_ROUTE_PASSWD, (unsigned char*)password, sizeof(password));

reconnect:
  SetWifiModeOff();
  int ret = SetWifiModeOn();
  if(ret != 0){
    printf("wifi connect failed,please check wifi config and the AP!\n");
    return;
  }
  mqtt_init();

  while (1) {
    if (!wait_message()) {
      goto reconnect;
    }
    LOS_Msleep(1);
  }
}


/***************************************************************
 * 函数名称: smart_home_thread
 * 说    明: 智慧家居主线程
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
void smart_home_thread(void *arg)
{
    e_iot_data iot_data = {0};

    i2c_dev_init();
    lcd_dev_init();
    motor_dev_init();
    light_dev_init();
    su03t_init();
    mq2_dev_init();
    pir_dev_init();
    mpu6050_dev_init();
    buzzer_dev_init();
    alarm_led_init();

    lcd_show_ui();

    while(1)
    {
        event_info_t event_info = {0};
        int ret = smart_home_event_wait(&event_info,3000);
        if(ret == LOS_OK){
            switch (event_info.event)
            {
                case event_key_press:
                    smart_home_key_process(event_info.data.key_no);
                    break;
                case event_iot_cmd:
                    smart_home_iot_cmd_process(event_info.data.iot_data);
                    break;
                case event_su03t:
                    smart_home_su03t_cmd_process(event_info.data.su03t_data);
                    break;
               default:break;
            }
        }

        // 采集所有传感器数据
        sht30_read_data(&g_temp, &g_humi);
        bh1750_read_data(&g_lux);
        mq2_read_data(&g_gas);
        g_pir_state = pir_read_state();
        extern void lcd_show_ui(void);
        extern bool has_person;
        has_person = g_pir_state ? true : false;
        lcd_show_ui();
        mpu6050_read_data(&g_accel_x, &g_accel_y, &g_accel_z);

        // 实时串口输出所有传感器数据
        printf("实时数据: 温度=%.2f°C, 湿度=%.2f%%, 光照=%.2fLux, 气体=%.2fppm, PIR=%d, 加速度=%.2f\n",
            g_temp, g_humi, g_lux, g_gas, g_pir_state,
            sqrt(g_accel_x*g_accel_x + g_accel_y*g_accel_y + g_accel_z*g_accel_z));

        // 报警判断
        check_alarm_conditions();

        // 有异常报警时串口输出异常类型
        if (is_any_alarm_active()) {
            printf("【报警】");
            if (g_temp_alarm)  printf("温度异常(%.2f°C) ", g_temp);
            if (g_humi_alarm)  printf("湿度异常(%.2f%%) ", g_humi);
            if (g_lux_alarm)   printf("光照异常(%.2fLux) ", g_lux);
            if (g_gas_alarm)   printf("气体异常(%.2fppm) ", g_gas);
            if (g_pir_alarm)   printf("人体入侵 ");
            if (g_accel_alarm) printf("震动异常 ");
            printf("\n");
        }

        // 联动控制
        if (is_any_alarm_active()) {
            // 有报警时 - 先关闭绿灯
            IoTPwmStop(LED_G_PORT); // 关闭绿灯PWM
            buzzer_set_state(true);
            alarm_led_set_state(true);
            
            // 根据报警类型设置不同的蜂鸣和LED模式
            if (g_gas_alarm) {
                buzzer_alarm_pattern(2);  // 气体报警：间歇
                alarm_led_blink_pattern(3);  // 急促闪
            } else if (g_temp_alarm || g_humi_alarm) {
                buzzer_alarm_pattern(1);  // 温湿度报警：长鸣
                alarm_led_blink_pattern(2);  // 快闪
            } else if (g_lux_alarm) {
                buzzer_alarm_pattern(1);  // 光照异常：长鸣
                alarm_led_blink_pattern(1);  // 慢闪
            } else if (g_pir_alarm) {
                buzzer_alarm_pattern(0);  // PIR报警：短促
                alarm_led_blink_pattern(1);  // 慢闪
            } else if (g_accel_alarm) {
                buzzer_alarm_pattern(2);  // 震动报警：间歇
                alarm_led_blink_pattern(2);  // 快闪
            }
            
            // 温度过高时自动启动风扇
            if (g_temp_alarm && !get_motor_state()) {
                motor_set_state(true);
                lcd_set_motor_state(true);
            }
        } else {
            // 无报警时
            buzzer_set_state(false);
            alarm_led_set_state(false);
            // 红灯PWM停止，绿灯PWM常亮
            IoTPwmStop(LED_R_PORT); // 红灯PWM停止
            IoTPwmStart(LED_G_PORT, 99, 1000); // 绿灯99%占空比常亮
            // 温度正常时关闭风扇，但手动优先时除外
            if (!g_temp_alarm && get_motor_state() && !motor_manual_override) {
                motor_set_state(false);
                lcd_set_motor_state(false);
            }
        }

        // 更新报警LED状态
        alarm_led_update();

        // LCD显示
        lcd_set_temperature(g_temp);
        lcd_set_humidity(g_humi);
        lcd_set_illumination(g_lux);

        // MQTT数据上报
        if (mqtt_is_connected()) 
        {
            iot_data.illumination = g_lux;
            iot_data.temperature = g_temp;
            iot_data.humidity = g_humi;
            iot_data.light_state = get_light_state();
            iot_data.motor_state = get_motor_state();
            // 可扩展：iot_data.gas = g_gas; ...
            send_msg_to_mqtt(&iot_data);
            lcd_set_network_state(true);
        }else{  
            lcd_set_network_state(false);
        }

        lcd_show_ui();
    }
}

/***************************************************************
 * 函数名称: device_read_thraed
 * 说    明: 设备读取线程
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
// void device_read_thraed(void *arg)
// {
//     double read_data[3] = {0};

//     i2c_dev_init();

//     while(1)
//     {
//         bh1750_read_data(&read_data[0]);
//         sht30_read_data(&read_data[1]);
//         LOS_QueueWrite(m_msg_queue, (void *)&read_data, sizeof(read_data), LOS_WAIT_FOREVER);
//         LOS_QueueWrite(m_su03_msg_queue, (void *)&read_data, sizeof(read_data), LOS_WAIT_FOREVER);
//         LOS_Msleep(500);
//     }
// }

/***************************************************************
 * 函数名称: iot_smart_hone_example
 * 说    明: 开机自启动调用函数
 * 参    数: 无
 * 返 回 值: 无
 ***************************************************************/
void iot_smart_home_example()
{
    unsigned int thread_id_1;
    unsigned int thread_id_2;
    unsigned int thread_id_3;
    TSK_INIT_PARAM_S task_1 = {0};
    TSK_INIT_PARAM_S task_2 = {0};
    TSK_INIT_PARAM_S task_3 = {0};
    unsigned int ret = LOS_OK;
    
    smart_home_event_init();
    
    // ret = LOS_QueueCreate("su03_queue", MSG_QUEUE_LENGTH, &m_su03_msg_queue, 0, BUFFER_LEN);
    // if (ret != LOS_OK)
    // {
    //     printf("Falied to create Message Queue ret:0x%x\n", ret);
    //     return;
    // }

    task_1.pfnTaskEntry = (TSK_ENTRY_FUNC)smart_home_thread;
    task_1.uwStackSize = 2048;
    task_1.pcName = "smart hone thread";
    task_1.usTaskPrio = 24;
    
    ret = LOS_TaskCreate(&thread_id_1, &task_1);
    if (ret != LOS_OK)
    {
        printf("Falied to create task ret:0x%x\n", ret);
        return;
    }

    task_2.pfnTaskEntry = (TSK_ENTRY_FUNC)adc_key_thread;
    task_2.uwStackSize = 2048;
    task_2.pcName = "key thread";
    task_2.usTaskPrio = 24;
    ret = LOS_TaskCreate(&thread_id_2, &task_2);
    if (ret != LOS_OK)
    {
        printf("Falied to create task ret:0x%x\n", ret);
        return;
    }

    task_3.pfnTaskEntry = (TSK_ENTRY_FUNC)iot_thread;
    task_3.uwStackSize = 20480*5;
    task_3.pcName = "iot thread";
    task_3.usTaskPrio = 24;
    ret = LOS_TaskCreate(&thread_id_3, &task_3);
    if (ret != LOS_OK)
    {
        printf("Falied to create task ret:0x%x\n", ret);
        return;
    }
}

APP_FEATURE_INIT(iot_smart_home_example);
