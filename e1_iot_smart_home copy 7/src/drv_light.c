#include "drv_light.h"
#include "iot_gpio.h"
#include "lz_hardware.h"
#include "iot_pwm.h"

/* RGB对应的PWM通道 */
#define LED_R_PORT EPWMDEV_PWM1_M1
#define LED_G_PORT EPWMDEV_PWM7_M1
#define LED_B_PORT EPWMDEV_PWM0_M1

static bool g_light_state = false;


/***************************************************************
* 函数名称: light_dev_init
* 说    明: rgb灯设备初始化
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void light_dev_init(void)
{
    unsigned int ret;

    /* 初始化PWM */
    ret = IoTPwmInit(LED_R_PORT);
    if (ret != 0) {
        printf("IoTPwmInit failed(%d)\n", LED_R_PORT);
    }

    ret = IoTPwmInit(LED_G_PORT);
    if (ret != 0) {
        printf("IoTPwmInit failed(%d)\n", LED_G_PORT);
    }

    ret = IoTPwmInit(LED_B_PORT);
    if (ret != 0) {
        printf("IoTPwmInit failed(%d)\n", LED_B_PORT);
    }

    printf("RGB LED PWM initialized successfully\n");
}

/***************************************************************
* 函数名称: light_set_state
* 说    明: 控制灯状态
* 参    数: bool state true：打开 false：关闭
* 返 回 值: 无
***************************************************************/
void light_set_state(bool state)
{
    if (state == g_light_state)
    {
        return;
    }

    if (state)
    {
        // 点亮RGB灯（可根据需要只点亮某一路）
        IoTPwmStart(LED_R_PORT, 99, 1000);
        IoTPwmStart(LED_G_PORT, 99, 1000);
        IoTPwmStart(LED_B_PORT, 99, 1000);
    }
    else
    {
        IoTPwmStop(LED_R_PORT);
        IoTPwmStop(LED_G_PORT);
        IoTPwmStop(LED_B_PORT);
    }
    g_light_state = state;
}



int get_light_state(void)
{
    return g_light_state;
}
