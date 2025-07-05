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

#include "smart_home.h"
#include "iot_pwm.h"
#include "iot_gpio.h"

#define BUZZER_PWM_HANDLE EPWMDEV_PWM5_M0
#define BUZZER_GPIO_HANDLE GPIO0_PC5

static bool g_buzzer_state = false;
static unsigned int g_buzzer_freq = 1000;  // 默认频率1KHz

/***************************************************************
* 函数名称: buzzer_dev_init
* 说    明: 蜂鸣器设备初始化
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void buzzer_dev_init(void)
{
    // 初始化PWM
    IoTPwmInit(BUZZER_PWM_HANDLE);
    
    // 初始化GPIO作为备用控制
    IoTGpioInit(BUZZER_GPIO_HANDLE);
    IoTGpioSetDir(BUZZER_GPIO_HANDLE, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(BUZZER_GPIO_HANDLE, IOT_GPIO_VALUE0);
    
    printf("Buzzer init success.\r\n");
}

/***************************************************************
* 函数名称: buzzer_set_frequency
* 说    明: 设置蜂鸣器频率
* 参    数: unsigned int freq 频率(Hz)
* 返 回 值: 无
***************************************************************/
void buzzer_set_frequency(unsigned int freq)
{
    if (freq < 100 || freq > 10000) {
        printf("Invalid frequency: %d Hz, using default 1000 Hz\r\n", freq);
        freq = 1000;
    }
    
    g_buzzer_freq = freq;
    
    if (g_buzzer_state) {
        // 如果蜂鸣器当前是开启状态，重新设置频率
        IoTPwmStart(BUZZER_PWM_HANDLE, 50, g_buzzer_freq);  // 50%占空比
    }
}

/***************************************************************
* 函数名称: buzzer_set_state
* 说    明: 控制蜂鸣器状态
* 参    数: bool state true：打开 false：关闭
* 返 回 值: 无
***************************************************************/
void buzzer_set_state(bool state)
{
    if (state == g_buzzer_state) {
        return;
    }

    if (state) {
        // 开启蜂鸣器
        IoTPwmStart(BUZZER_PWM_HANDLE, 50, g_buzzer_freq);  // 50%占空比
        IoTGpioSetOutputVal(BUZZER_GPIO_HANDLE, IOT_GPIO_VALUE1);
    } else {
        // 关闭蜂鸣器
        IoTPwmStop(BUZZER_PWM_HANDLE);
        IoTGpioSetOutputVal(BUZZER_GPIO_HANDLE, IOT_GPIO_VALUE0);
    }
    
    g_buzzer_state = state;
}

/***************************************************************
* 函数名称: buzzer_alarm_pattern
* 说    明: 蜂鸣器报警模式
* 参    数: int pattern 报警模式 0:短促 1:长鸣 2:间歇
* 返 回 值: 无
***************************************************************/
void buzzer_alarm_pattern(int pattern)
{
    switch (pattern) {
        case 0:  // 短促报警
            buzzer_set_frequency(2000);
            buzzer_set_state(true);
            break;
        case 1:  // 长鸣报警
            buzzer_set_frequency(1000);
            buzzer_set_state(true);
            break;
        case 2:  // 间歇报警
            buzzer_set_frequency(1500);
            buzzer_set_state(true);
            break;
        default:
            buzzer_set_state(false);
            break;
    }
} 