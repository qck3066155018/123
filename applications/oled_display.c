/*
 * 专属的环境数据 OLED 显示模块
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include "ssd1306.h"
#include <stdlib.h>
#include <board.h>
#include <time.h>
#include "ccm_alloc.h"

#define PIN_KEY_TOGGLE  GET_PIN(C, 1)

/* 引入外部全局变量 */
extern uint32_t g_adc_voltage_mv;
extern int32_t g_ds18b20_temp;
extern int32_t g_can_current_ma;
extern uint32_t g_adc_voltage_mv_ch1;
extern uint8_t g_sys_alarm_status;
extern uint8_t g_bl0972_measure_mode;
/* 静态控制块 */
static struct rt_thread oled_ui_thread;
static struct rt_thread key_ctrl_thread;

/* ================= 12x12 中文字模数组 ================= */
/* "时" */
const uint8_t CN_Shi[] = {
    0x7F,0xE0,0x44,0x40,0x44,0x40,0x7F,0xE0,0x00,0x00,0x10,0x00,0x12,0x00,0x11,0x90,
    0x10,0x10,0xFF,0xF0,0x10,0x00,0x00,0x00
};
/* "间" */
const uint8_t CN_Jian[] = {
    0x00,0x00,0x9F,0xF0,0x40,0x00,0x1F,0xC0,0x92,0x40,0x92,0x40,0x92,0x40,0x92,0x40,
    0x9F,0xD0,0x80,0x10,0xFF,0xF0,0x00,0x00
};
/* "电" */
const uint8_t CN_Dian[] = {
    0x3F,0xC0,0x24,0x80,0x24,0x80,0x24,0x80,0xFF,0xE0,0x24,0x90,0x24,0x90,0x24,0x90,
    0x3F,0x90,0x00,0x10,0x00,0x70,0x00,0x00
};
/* "压" */
const uint8_t CN_Ya[] = {
    0x00,0x10,0x7F,0xE0,0x40,0x10,0x42,0x10,0x42,0x10,0x42,0x10,0x5F,0xF0,0x42,0x10,
    0x42,0x90,0x42,0x50,0x40,0x10,0x00,0x00
};
/* "温" */
const uint8_t CN_Wen[] = {
    0x44,0x20,0x22,0x40,0x00,0x90,0x03,0xF0,0xFA,0x10,0xAB,0xF0,0xAA,0x10,0xAB,0xF0,
    0xFA,0x10,0x03,0xF0,0x00,0x10,0x00,0x00
};
/* "度" */
const uint8_t CN_Du[] = {
    0x00,0x10,0x7F,0xE0,0x50,0x00,0x51,0x10,0x7D,0x90,0x55,0x50,0xD5,0x20,0x55,0x20,
    0x7D,0x50,0x51,0x90,0x50,0x10,0x00,0x00
};
/* "流" */
const uint8_t CN_Liu[] = {
    0x44,0x20,0x22,0x40,0x00,0x10,0x24,0x20,0x2D,0xC0,0x34,0x00,0xA5,0xF0,0x64,0x00,
    0x25,0xE0,0x2C,0x10,0x26,0x70,0x00,0x00
};
/* "警" */
const uint8_t CN_Jing[] = {
    0x49,0x00,0xFD,0x40,0x55,0x70,0x5D,0x50,0xF3,0x50,0x5D,0x50,0x23,0x50,0xD5,0x50,
    0x49,0x70,0x75,0x40,0x43,0x00,0x00,0x00
};
/* "告" */
const uint8_t CN_Gao[] = {
    0x04,0x00,0x15,0xF0,0x65,0x20,0x25,0x20,0x25,0x20,0xFD,0x20,0x25,0x20,0x25,0x20,
    0x25,0x20,0x25,0xF0,0x04,0x00,0x00,0x00
};

/* ================= 绘制 12x12 汉字函数 (适配逐列式、逆向/高位在前) ================= */
static void ssd1306_DrawChinese12x12(uint8_t x, uint8_t y, const uint8_t *chArray)
{
    uint8_t j, byte_top, byte_bottom;

    for (j = 0; j < 12; j++)
    {
        /* 12x12 逐列式：每列由上下两个字节组成 */
        byte_top = chArray[j * 2];       /* 上半部分 8 像素 */
        byte_bottom = chArray[j * 2 + 1];  /* 下半部分 4 像素 */

        for (int k = 0; k < 8; k++)
        {
            if (byte_top & (0x80 >> k))
            {
                ssd1306_DrawPixel(x + j, y + k, White);
            }
        }

        for (int k = 0; k < 4; k++)
        {
            /* 下半部分只用到了高 4 位 */
            if (byte_bottom & (0x80 >> k))
            {
                ssd1306_DrawPixel(x + j, y + 8 + k, White);
            }
        }
    }
}

static void oled_display_entry(void *parameter)
{
    char buf[32];
    time_t now;
    struct tm *tm_info;

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();

    while (1)
    {
        ssd1306_Fill(Black);
        now = time(RT_NULL);
        tm_info = localtime(&now);

        /* ================= 1. 第一行：时间 (Y=0) ================= */
        ssd1306_DrawChinese12x12(2, 0, CN_Shi);
        ssd1306_DrawChinese12x12(16, 0, CN_Jian); /* X坐标间距缩小为 14 (12像素字体+2像素间距) */
        snprintf(buf, sizeof(buf), ":%02d:%02d:%02d",
                tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
        ssd1306_SetCursor(30, 1); /* Y轴+1，让 10px 的英文和 12px 的中文垂直居中对齐 */
        ssd1306_WriteString(buf, Font_7x10, White);

        snprintf(buf, sizeof(buf), "%s", (g_bl0972_measure_mode == 1) ? "DC" : "AC");
        ssd1306_SetCursor(112, 1);
        ssd1306_WriteString(buf, Font_7x10, White);

        /* ================= 2. 第二行：电压 (Y=16) ================= */
                ssd1306_DrawChinese12x12(2, 16, CN_Dian);
                ssd1306_DrawChinese12x12(16, 16, CN_Ya);
                /* 寄存器值已是电压(V)，直接显示整数 */
                int v0 = (int)g_adc_voltage_mv;

                int v1 = (int)g_adc_voltage_mv_ch1;

                /* 格式化并打印 */
                snprintf(buf, sizeof(buf), ":%d  %d", v0, v1);
                ssd1306_SetCursor(30, 17);
                ssd1306_WriteString(buf, Font_7x10, White);

        /* ================= 3. 第三行：温度 (Y=32) ================= */
        ssd1306_DrawChinese12x12(2, 32, CN_Wen);
        ssd1306_DrawChinese12x12(16, 32, CN_Du);
        if (g_ds18b20_temp >= 0) {
            snprintf(buf, sizeof(buf), ":%d.%d C", (int)(g_ds18b20_temp / 10), (int)(g_ds18b20_temp % 10));
        } else {
            snprintf(buf, sizeof(buf), ":-%d.%d C", (int)abs(g_ds18b20_temp / 10), (int)abs(g_ds18b20_temp % 10));
        }
        ssd1306_SetCursor(30, 33);
        ssd1306_WriteString(buf, Font_7x10, White);

        /* ================= 4. 第四行：电流 (Y=48) ================= */
        /* 左侧画电流文字和数值 */
        ssd1306_DrawChinese12x12(2, 48, CN_Dian);
        ssd1306_DrawChinese12x12(16, 48, CN_Liu);
        snprintf(buf, sizeof(buf), ":%d mA", (int)g_can_current_ma);
        ssd1306_SetCursor(30, 49);
        ssd1306_WriteString(buf, Font_7x10, White);

//        /* ================= 5. 异常状态闪烁提示 ================= */
//        if (g_sys_alarm_status != 0)
//        {
//            if (tm_info->tm_sec % 2 == 0)
//            {
//                /* 警告闪烁放在右上角，因为字体小了，稍微靠右调到了102 */
//                ssd1306_DrawChinese12x12(102, 0, CN_Jing);
//                ssd1306_DrawChinese12x12(116, 0, CN_Gao);
//            }
//        }

        ssd1306_UpdateScreen();
        rt_thread_mdelay(1000);
    }
}

/* 按键轮询线程保持不变 */
static void key_thread_entry(void *parameter)
{
    rt_pin_mode(PIN_KEY_TOGGLE, PIN_MODE_INPUT_PULLUP);
    uint8_t key_pressed = 0;
    while (1)
    {
        if (rt_pin_read(PIN_KEY_TOGGLE) == PIN_LOW)
        {
            rt_thread_mdelay(20);
            if (rt_pin_read(PIN_KEY_TOGGLE) == PIN_LOW && key_pressed == 0)
            {
                key_pressed = 1;
                if (ssd1306_GetDisplayOn() == 1)
                {
                    ssd1306_SetDisplayOn(0);
                    rt_kprintf("OLED Display OFF\n");
                }
                else
                {
                    ssd1306_SetDisplayOn(1);
                    rt_kprintf("OLED Display ON\n");
                }
            }
        }
        else
        {
            key_pressed = 0;
        }
        rt_thread_mdelay(50);
    }
}

int oled_display_init(void)
{
    void *stack = rt_ccm_malloc(1024);
    if (stack == RT_NULL) return -RT_ERROR;

    rt_err_t res = rt_thread_init(&oled_ui_thread, "oled_ui", oled_display_entry, RT_NULL, stack, 1024, 21, 10);
    if (res == RT_EOK)
    {
        rt_thread_startup(&oled_ui_thread);
        return RT_EOK;
    }
    return -RT_ERROR;
}
INIT_APP_EXPORT(oled_display_init);

int key_control_init(void)
{
    void *stack = rt_ccm_malloc(1024);
    if (stack == RT_NULL) return -RT_ERROR;

    rt_err_t res = rt_thread_init(&key_ctrl_thread, "key_ctrl", key_thread_entry, RT_NULL, stack, 1024, 24, 10);
    if (res == RT_EOK)
    {
        rt_thread_startup(&key_ctrl_thread);
        return RT_EOK;
    }
    return -RT_ERROR;
}
INIT_APP_EXPORT(key_control_init);
