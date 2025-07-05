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

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "iot_errno.h"

#include "iot_pwm.h"
#include "iot_gpio.h"
#include "su_03t.h"
#include "iot.h"
#include "lcd.h"
#include "picture.h"
#include "adc_key.h"
#include "components.h"
#include "lcd.h"

void su03t_send_double_msg(uint8_t index, double dat);
void lcd_show_chinese(uint16_t x, uint16_t y, uint8_t *str, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode);
void lcd_menu_update(lcd_menu_t **menus, int menu_number, int select_index);
void lcd_db_show(lcd_display_board_t **dbs, int dbs_number);

static bool auto_state = false;
static bool network_state = false;
static bool motor_auto_on_by_lux = false;  // 电机是否因光照异常而自动开启
bool motor_manual_override = false;  // 重新添加手动优先标志
bool has_person = false; // 允许外部引用

void light_menu_entry(lcd_menu_t *menu);
void fan_menu_entry(lcd_menu_t *menu);

/* 风扇菜单的数据初始化*/
lcd_menu_t fan_menu={
    .img={
        .img=(const uint8_t *)img_fan_off,
        .height=64,
        .width=64,
    },
    .is_selected=false,
    .text={
        .fc=LCD_MAGENTA,
        .bc=LCD_WHITE,
        .font_size=24,
        .name="风扇关",
    },
    .enterFunc=fan_menu_entry,
    .exitFunc=NULL,
    .base_x=100,
    .base_y=64,
 
};

/* 照明灯的菜单初始化数据*/
lcd_menu_t light_menu={
    .img={
        .img=(const uint8_t *)img_light_off,
        .height=64,
        .width=64,
    },
    .is_selected=false,
    .text={
        .fc=LCD_MAGENTA,
        .bc=LCD_WHITE,
        .font_size=24,
        .name="灯光关",
    },
    .base_x=20,
    .base_y=64,
    .enterFunc=light_menu_entry,
    .exitFunc=NULL,
};

/* 温度面板的初始化数据*/
lcd_display_board_t temp_db={
    .img={
        .img=(const uint8_t *)img_temp_normal,
        .height=48,
        .width=48,
    },
    .text={
        .fc=LCD_MAGENTA,
        .bc=LCD_WHITE,
        .font_size=24,
        .name="25.5°C",
    },
    .base_x=180,
    .base_y=64,
};

/* 湿度面板的初始化数据*/
lcd_display_board_t humi_db={
    .img={
        .img=(const uint8_t *)img_humi,
        .height=48,
        .width=48,
    },
    .text={
        .fc=LCD_MAGENTA,
        .bc=LCD_WHITE,
        .font_size=24,
        .name="25.5°C",
    },
    .base_x=180,
    .base_y=120,
};

/* 亮度面板的初始化数据*/
lcd_display_board_t lum_db={
    .img={
        .img=(const uint8_t *)img_lum,
        .height=48,
        .width=48,
    },
    .text={
        .fc=LCD_MAGENTA,
        .bc=LCD_WHITE,
        .font_size=24,
        .name="1234Lx",
    },
    .base_x=180,
    .base_y=174,
};
/* 所有的面板集合数组,方便遍历查询*/
lcd_display_board_t *lcd_dbs[] ={&temp_db,&humi_db,&lum_db};
/* 所有的菜单集合数组,方便遍历查询*/
lcd_menu_t *lcd_menus[] = {&light_menu,&fan_menu};
/* 菜单的个数*/
static int lcd_menu_number =  sizeof(lcd_menus)/sizeof(lcd_menu_t *);
/* 菜单的当前选中索引,在数组中的位置*/
static int menu_select_index = 0;

//菜单左和右的处理,需要考虑菜单个数的边界
void lcd_menu_selected_move_left()
{
    if(menu_select_index > 0){
        menu_select_index--;
    }
}

void lcd_menu_selected_move_right()
{
    if(menu_select_index <lcd_menu_number-1){
        menu_select_index++;
    }
}

/**
 * @brief 照明灯菜单按下确认按键
 * 
 * @param menu 
 */
void light_menu_entry(lcd_menu_t *menu)
{
    int light_state = get_light_state();
    if(light_state)
    {
        light_set_state(false);
        lcd_set_light_state(false);
    }else{
        light_set_state(true);
        lcd_set_light_state(true);
    }
}
/**
 * @brief  风扇菜单按下确认按键
 * 
 * @param menu 
 */
void fan_menu_entry(lcd_menu_t *menu)
{
   int motor_state = get_motor_state();
   if(motor_state)
   {
       motor_set_state(false);
       lcd_set_motor_state(false);
       motor_manual_override = false;  // 按键关闭时重置手动优先标志
       motor_auto_on_by_lux = false;   // 清除光照自动开启标记
   }else{
       motor_set_state(true);
       lcd_set_motor_state(true);
       motor_manual_override = true;   // 按键打开时设置手动优先标志
       motor_auto_on_by_lux = false;   // 清除光照自动开启标记
   }
}

/***************************************************************
* 函数名称: lcd_dev_init
* 说    明: lcd初始化
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void lcd_dev_init(void)
{
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, LCD_WHITE);
    
    // 显示欢迎页面
    extern const unsigned char gImage_hy[129280];
    // 居中显示欢迎页面，假设LCD宽度为320，高度为240
    int welcome_x = (LCD_W - 320) / 2;  // 水平居中
    int welcome_y = (LCD_H - 202) / 2;  // 垂直居中
    lcd_show_picture(welcome_x, welcome_y, 320, 202, gImage_hy);
    
    // 延时2秒显示欢迎页面
    LOS_TaskDelay(2000);  // 延时2000ms
    
    // 清屏准备显示主界面
    lcd_fill(0, 0, LCD_W, LCD_H, LCD_WHITE);
}

/**
 * @brief 按键处理函数
 * 
 * @param key_no 按键号
 */
void smart_home_key_process(int key_no)
{
    printf("smart_home_key_process:%d\n",key_no);
    if(key_no == KEY_UP){

    }else if(key_no == KEY_DOWN){
        lcd_menu_entry(lcd_menus[menu_select_index]);

    }else if(key_no == KEY_LEFT){
        
        lcd_menu_selected_move_left();
    }else if(key_no == KEY_RIGHT){
        lcd_menu_selected_move_right();
    }
}
/**
 * @brief 物联网的指令处理函数
 * 
 * @param iot_cmd iot的指令
 */
void smart_home_iot_cmd_process(int iot_cmd)
{
    switch (iot_cmd)
    {
        case IOT_CMD_LIGHT_ON:
            light_set_state(true);
            lcd_set_light_state(true);
            break;
        case IOT_CMD_LIGHT_OFF:
            light_set_state(false);
            lcd_set_light_state(false);
            break;
        case IOT_CMD_MOTOR_ON:
            motor_set_state(true);
            lcd_set_motor_state(true);
            motor_manual_override = true;  // 物联网指令打开时设置手动优先
            motor_auto_on_by_lux = false;  // 清除光照自动开启标记
            break;
        case IOT_CMD_MOTOR_OFF:
            motor_set_state(false);
            lcd_set_motor_state(false);
            motor_manual_override = false;  // 物联网指令关闭时重置
            motor_auto_on_by_lux = false;  // 清除光照自动开启标记
            break;
    }
}

/**
 * @brief 语音管家发出的指令
 * 
 * @param su03t_cmd 语音管家的指令
 */
void smart_home_su03t_cmd_process(int su03t_cmd)
{
    switch (su03t_cmd)
    {
        case light_state_on:
            light_set_state(true);
            lcd_set_light_state(true);
            break;
        case light_state_off:
            light_set_state(false);
            lcd_set_light_state(false);
            break;
        case motor_state_on:
            motor_set_state(true);
            lcd_set_motor_state(true);
            motor_manual_override = true;  // 语音指令打开时设置手动优先
            motor_auto_on_by_lux = false;  // 清除光照自动开启标记
            break;
        case motor_state_off:
            motor_set_state(false);
            lcd_set_motor_state(false);
            motor_manual_override = false;  // 语音指令关闭时重置
            motor_auto_on_by_lux = false;  // 清除光照自动开启标记
            break;
        case temperature_get:
        {
            double temp,humi;

            sht30_read_data(&temp, &humi);
            su03t_send_double_msg(1, temp);
        }
            
            break;
        case humidity_get:
        {
            double temp,humi;
            sht30_read_data(&temp, &humi);

            su03t_send_double_msg(2, humi);
        }
            break;
         case gas_get:
        {
            double gas;
            mq2_read_data(&gas);
            su03t_send_double_msg(3, gas);}
             break;
        case illumination_get:
        {
            double lum;
            bh1750_read_data(&lum);
            su03t_send_double_msg(4, lum);
        }
            
        
       
            break;
        case alarm_led_on:
            alarm_led_set_state(true);
            break;
        case alarm_led_off:
            alarm_led_set_state(false);
            break;
        default:
            break;
    }
}

/***************************************************************
* 函数名称: lcd_load_ui
* 说    明: 加载lcd ui
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void lcd_show_ui(void)
{
    // 动态调整纵向布局参数
    #define GAS_IMG_TOP_Y 20
    #define PANEL_V_GAP 4
    int gas_img_y = GAS_IMG_TOP_Y;
    int temp_img_y = gas_img_y + 48 + PANEL_V_GAP;
    temp_db.base_y = temp_img_y;
    humi_db.base_y = temp_img_y + temp_db.img.height + PANEL_V_GAP;
    lum_db.base_y = humi_db.base_y + humi_db.img.height + PANEL_V_GAP;
    lcd_show_chinese(64, 0, (uint8_t*)"环境检测与人体感应报警系统", LCD_RED, LCD_WHITE, 16, 0);
    lcd_show_picture(0, 0, 32, 32, network_state ? img_wifi_on : img_wifi_off);
    
    // 在灯光图片上方显示logo，避免压到标题和WiFi图片
    extern const unsigned char gImage_logo[4000];
    lcd_show_picture(light_menu.base_x, 35, 100, 20, gImage_logo);
    
    lcd_menu_update(lcd_menus ,lcd_menu_number,menu_select_index);
    lcd_menu_show(lcd_menus ,  lcd_menu_number);
    lcd_db_show(lcd_dbs, sizeof(lcd_dbs)/sizeof(lcd_display_board_t *));
    // 只显示有人/无人图片，不再显示开门图片
    const unsigned char* person_img = has_person ? gImage_km : gImage_gm;
    int person_img_x = light_menu.base_x;
    int person_img_y = light_menu.base_y + light_menu.img.height + 3 + light_menu.text.font_size + 5;
    lcd_show_picture(
        person_img_x,
        person_img_y,
        63,
        64,
        person_img
    );
    // 在图片右侧显示"有人"或"无人"文字
    const char* person_text = has_person ? "有人" : "无人";
    int text_x = person_img_x + 63 + 8;
    int text_y = person_img_y + (64 - light_menu.text.font_size) / 2;
    lcd_show_chinese(text_x, text_y, (uint8_t*)person_text, light_menu.text.fc, LCD_WHITE, light_menu.text.font_size, 0);

    // 气体图片在温度图片正上方，气体数据在气体图片右方
    extern const unsigned char gImage_gas[4512];
    extern double g_gas;
    int gas_img_x = temp_db.base_x;
    // int gas_img_y = temp_db.base_y - 48 - 8; // 已在上方自动调整
    lcd_show_picture(gas_img_x, gas_img_y, 47, 48, gImage_gas);
    // 气体数据在气体图片右方，垂直居中
    char gas_str[20];
    snprintf(gas_str, sizeof(gas_str), "%.1fppm", g_gas);
    int gas_text_x = gas_img_x + 47 + 8; // 图片右侧8像素间隔
    int gas_text_y = gas_img_y + (48 - temp_db.text.font_size) / 2; // 垂直居中
    
    // 根据气体浓度设置颜色
    uint16_t gas_text_color;
    if(g_gas > GAS_ALARM_THRESHOLD)
    {
        gas_text_color = LCD_RED;  // 浓度异常时显示红色
    }
    else
    {
        gas_text_color = LCD_MAGENTA;  // 浓度正常时显示洋红色
    }
    
    lcd_show_string(gas_text_x, gas_text_y, (uint8_t*)gas_str, gas_text_color, LCD_WHITE, temp_db.text.font_size, 0);
}

/***************************************************************
* 函数名称: lcd_set_temperature
* 说    明: 设置温度显示
* 参    数: double temperature 温度
* 返 回 值: 无
***************************************************************/
void lcd_set_temperature(double temperature)
{
    sprintf(temp_db.text.name, "%.01f℃ ", temperature);
    /* 对温度做高温和正常的区分*/
    if(temperature > 35)
    {
        temp_db.text.fc = LCD_RED;
        temp_db.img.img = img_temp_high;
    }
    else
    {
       temp_db.text.fc = LCD_MAGENTA;
       temp_db.img.img = img_temp_normal;
    }
}

/***************************************************************
* 函数名称: lcd_set_humidity
* 说    明: 设置湿度显示
* 参    数: double humidity 湿度
* 返 回 值: 无
***************************************************************/
void lcd_set_humidity(double humidity)
{
    sprintf(humi_db.text.name, "%.01f%% ", humidity);

}

/***************************************************************
* 函数名称: lcd_set_illumination
* 说    明: 设置光照强度显示
* 参    数: double illumination 光照强度
* 返 回 值: 无
***************************************************************/
void lcd_set_illumination(double illumination)
{
    sprintf(lum_db.text.name, "%.01fLx ", illumination);
    
    /* 对光照强度做异常和正常的区分，异常时自动打开电机*/
    if(illumination < LUX_ALARM_THRESHOLD_LOW || illumination > LUX_ALARM_THRESHOLD_HIGH)
    {
        lum_db.text.fc = LCD_RED;
        // 光照异常时自动打开电机
        if(!get_motor_state())  // 如果电机当前是关闭状态
        {
            motor_set_state(true);
            lcd_set_motor_state(true);
            motor_auto_on_by_lux = true;  // 标记为光照异常自动开启
            if(illumination < LUX_ALARM_THRESHOLD_LOW) {
                printf("光照异常(%.1fLx < %.1fLx)，自动打开电机\n", illumination, LUX_ALARM_THRESHOLD_LOW);
            } else {
                printf("光照异常(%.1fLx > %.1fLx)，自动打开电机\n", illumination, LUX_ALARM_THRESHOLD_HIGH);
            }
        }
    }
    else
    {
       lum_db.text.fc = LCD_MAGENTA;
       // 光照正常时，只有电机是由光照异常自动开启的才自动关闭
       if(get_motor_state() && motor_auto_on_by_lux)  // 电机开启且是由光照异常自动开启的
       {
           motor_set_state(false);
           lcd_set_motor_state(false);
           motor_auto_on_by_lux = false;  // 清除自动开启标记
           printf("光照正常(%.1fLx在%.1f-%.1fLx范围内)，自动关闭电机\n", illumination, LUX_ALARM_THRESHOLD_LOW, LUX_ALARM_THRESHOLD_HIGH);
       }
    }
}



void lcd_set_network_state(int state){
    network_state = state;
}

/***************************************************************
* 函数名称: lcd_set_light_state
* 说    明: 设置灯状态显示
* 参    数: bool state true：显示"打开" false：显示"关闭"
* 返 回 值: 无
***************************************************************/
void lcd_set_light_state(bool state)
{
    
    strcpy(light_menu.text.name,state? "灯光开" :"灯光关");
    light_menu.img.img = state? img_light_on : img_light_off;
}

/***************************************************************
* 函数名称: lcd_set_motor_state
* 说    明: 设置电机状态显示
* 参    数: bool state true：显示"打开" false：显示"关闭"
* 返 回 值: 无
***************************************************************/
void lcd_set_motor_state(bool state)
{

    strcpy(fan_menu.text.name,state? "风扇开" :"风扇关");
    fan_menu.img.img = state? img_fan_on : img_fan_off;

}

/***************************************************************
* 函数名称: lcd_set_auto_state
* 说    明: 设置自动模式状态显示
* 参    数: bool state true：显示"打开" false：显示"关闭"
* 返 回 值: 无
***************************************************************/
void lcd_set_auto_state(bool state)
{



    // if (state)
    // {
    //     lcd_show_chinese(77, 204, "开启", LCD_RED, LCD_WHITE, 24, 0);
    // }
    // else
    // {
    //     lcd_show_chinese(77, 204, "关闭", LCD_RED, LCD_WHITE, 24, 0);
    // }
}
