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
#include "iot_gpio.h"
#include "iot_pwm.h"

#define ALARM_LED_GPIO_HANDLE GPIO0_PB7
#define ALARM_LED_PWM_HANDLE EPWMDEV_PWM1_M1

static bool g_alarm_led_state = false;
static bool g_alarm_led_blink = false;
static unsigned int g_blink_count = 0;

/***************************************************************
* 函数名称: alarm_led_init
* 说    明: 报警LED初始化
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void alarm_led_init(void)
{
    // 初始化GPIO
    IoTGpioInit(ALARM_LED_GPIO_HANDLE);
    IoTGpioSetDir(ALARM_LED_GPIO_HANDLE, IOT_GPIO_DIR_OUT);
    IoTGpioSetOutputVal(ALARM_LED_GPIO_HANDLE, IOT_GPIO_VALUE0);
    
    // 初始化PWM用于闪烁控制
    IoTPwmInit(ALARM_LED_PWM_HANDLE);
    
    printf("Alarm LED init success.\r\n");
}

/***************************************************************
* 函数名称: alarm_led_set_state
* 说    明: 控制报警LED状态
* 参    数: bool state true：打开 false：关闭
* 返 回 值: 无
***************************************************************/
void alarm_led_set_state(bool state)
{
    if (state == g_alarm_led_state) {
        return;
    }

    if (state) {
        // 开启报警LED
        IoTGpioSetOutputVal(ALARM_LED_GPIO_HANDLE, IOT_GPIO_VALUE1);
        IoTPwmStart(ALARM_LED_PWM_HANDLE, 50, 2);  // 2Hz闪烁
    } else {
        // 关闭报警LED
        IoTGpioSetOutputVal(ALARM_LED_GPIO_HANDLE, IOT_GPIO_VALUE0);
        IoTPwmStop(ALARM_LED_PWM_HANDLE);
        g_alarm_led_blink = false;
        g_blink_count = 0;
    }
    
    g_alarm_led_state = state;
}

/***************************************************************
* 函数名称: alarm_led_blink_pattern
* 说    明: 报警LED闪烁模式
* 参    数: int pattern 闪烁模式 0:常亮 1:慢闪 2:快闪 3:急促闪
* 返 回 值: 无
***************************************************************/
void alarm_led_blink_pattern(int pattern)
{
    if (!g_alarm_led_state) {
        return;
    }
    
    switch (pattern) {
        case 0:  // 常亮
            IoTPwmStop(ALARM_LED_PWM_HANDLE);
            IoTGpioSetOutputVal(ALARM_LED_GPIO_HANDLE, IOT_GPIO_VALUE1);
            g_alarm_led_blink = false;
            break;
        case 1:  // 慢闪 (1Hz)
            IoTPwmStart(ALARM_LED_PWM_HANDLE, 50, 1);
            g_alarm_led_blink = true;
            break;
        case 2:  // 快闪 (2Hz)
            IoTPwmStart(ALARM_LED_PWM_HANDLE, 50, 2);
            g_alarm_led_blink = true;
            break;
        case 3:  // 急促闪 (5Hz)
            IoTPwmStart(ALARM_LED_PWM_HANDLE, 30, 5);
            g_alarm_led_blink = true;
            break;
        default:
            break;
    }
}

/***************************************************************
* 函数名称: alarm_led_update
* 说    明: 更新报警LED状态（在主循环中调用）
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void alarm_led_update(void)
{
    if (g_alarm_led_blink) {
        g_blink_count++;
        // 可以在这里添加更复杂的闪烁逻辑
    }
} 