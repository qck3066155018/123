/*
 * File      : alarm_monitor.c
 * 说明      : 异常数据监控与报警处理线程 (仅负责报警滤波、锁存与硬件驱动)
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "ccm_alloc.h"

#define DBG_TAG "app.alarm"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ================= 引入外部全局变量 ================= */
extern int32_t g_ds18b20_temp;
extern int32_t g_can_current_ma;
/* 核心联动：引入由 adc_read.c 预处理好的绝缘故障标志 */
extern uint8_t g_insul_fault_flag;

static struct rt_thread alarm_mon_thread;

/* 全局报警状态标志 (0为正常，非0为异常) */
uint8_t g_sys_alarm_status = 0;

/* ================== 报警阈值全局变量 ================== */
int32_t  g_th_temp_max = 700;   /* 温度阈值 70.0 °C */
int32_t  g_th_curr_max = 1000;    /* 电流阈值 1000 mA */
/* 注意：绝缘阈值判定已经在 adc_read.c 中完成，此处不再需要判定阻值 */

/* 报警输出引脚 */
#define PIN_ALARM_OUT   GET_PIN(B, 14)
#define PIN_ALARM_OUT1  GET_PIN(B, 0)//蜂鸣器
#define PIN_KEY_STOP    GET_PIN(C, 0)
#define ALARM_STOP_LONG_PRESS_COUNT 4

static void alarm_monitor_entry(void *parameter)
{
    rt_pin_mode(PIN_ALARM_OUT, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_ALARM_OUT, PIN_LOW);
    rt_pin_mode(PIN_ALARM_OUT1, PIN_MODE_OUTPUT);
    rt_pin_write(PIN_ALARM_OUT1, PIN_LOW);
    rt_pin_mode(PIN_KEY_STOP, PIN_MODE_INPUT_PULLUP);

    uint8_t last_alarm_status = 0;
    uint8_t alarm_latched = 0;

    /* 监控项滤波计数器 */
    uint8_t over_count_temp = 0;
    uint8_t over_count_insul = 0;
    uint8_t over_count_cur  = 0;
    uint8_t key_stop_press_count = 0;
    uint8_t key_stop_handled = 0;

    /* 滤波深度：持续 5 次（约 2.5 秒）异常才触发报警 */
#define ALARM_FILTER_MAX 5

    while (1)
    {
        uint8_t current_alarm = 0;

        /* 1. 温度判定 */
        if (g_ds18b20_temp > g_th_temp_max) {
            if (over_count_temp < ALARM_FILTER_MAX) over_count_temp++;
            if (over_count_temp >= ALARM_FILTER_MAX) current_alarm |= 0x01;
        } else {
            over_count_temp = 0;
        }

        /* 2. 绝缘判定 (直接读取 adc_read.c 计算出的结果) */
        if (g_insul_fault_flag == 1) {
            if (over_count_insul < ALARM_FILTER_MAX) over_count_insul++;
            if (over_count_insul >= ALARM_FILTER_MAX) current_alarm |= 0x02;
        } else {
            over_count_insul = 0;
        }

        /* 3. CAN 电流判定 */
        if (g_can_current_ma > g_th_curr_max) {
            if (over_count_cur < ALARM_FILTER_MAX) over_count_cur++;
            if (over_count_cur >= ALARM_FILTER_MAX) current_alarm |= 0x08;
        } else {
            over_count_cur = 0;
        }

        g_sys_alarm_status = current_alarm;

        /* 报警锁存逻辑 */
        if ((current_alarm & ~last_alarm_status) != 0)
        {
            alarm_latched = 1;
            LOG_W("ALARM TRIGGERED! Code: 0x%02X", current_alarm);
        }
        last_alarm_status = current_alarm;

        /* 按键解除报警 */
        if (rt_pin_read(PIN_KEY_STOP) == PIN_LOW)
        {
            if (key_stop_press_count < ALARM_STOP_LONG_PRESS_COUNT)
            {
                key_stop_press_count++;
            }

            if (key_stop_press_count >= ALARM_STOP_LONG_PRESS_COUNT &&
                key_stop_handled == 0 &&
                alarm_latched == 1)
            {
                alarm_latched = 0;
                key_stop_handled = 1;
                LOG_I("ALARM STOPPED BY USER LONG PRESS.");
            }
        }
        else
        {
            key_stop_press_count = 0;
            key_stop_handled = 0;
        }

        /* 驱动报警硬件 */
        if (alarm_latched != 0)
        {
            rt_pin_write(PIN_ALARM_OUT, PIN_HIGH);
            //rt_pin_write(PIN_ALARM_OUT1, PIN_HIGH);
        }
        else
        {
            rt_pin_write(PIN_ALARM_OUT, PIN_LOW);
           // rt_pin_write(PIN_ALARM_OUT1, PIN_LOW);
        }

        rt_thread_mdelay(500);
    }
}

int alarm_monitor_init(void)
{
    void *stack = rt_ccm_malloc(512);
    if (stack == RT_NULL) return -RT_ERROR;

    rt_err_t res = rt_thread_init(&alarm_mon_thread, "alarm_mon", alarm_monitor_entry, RT_NULL, stack, 512, 18, 10);
    if (res == RT_EOK)
    {
        rt_thread_startup(&alarm_mon_thread);
        return RT_EOK;
    }
    return -RT_ERROR;
}
INIT_APP_EXPORT(alarm_monitor_init);
