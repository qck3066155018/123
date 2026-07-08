/*
 * File      : rs485_app.c
 * 说明      : RS485 数码管持续显示 CAN 电流值
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "rs485.h"
#include <stdio.h>
#include <string.h>

#define DBG_TAG "app.rs485"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ================= 硬件参数配置 ================= */
#define APP_RS485_SERIAL       "uart6"
#define APP_RS485_BAUDRATE     9600
#define APP_RS485_PARITY       0
#define APP_RS485_PIN          104
#define APP_RS485_LVL          1
#define LED_DISPLAY_ADDR       1
#define LED_DISPLAY_PERIOD_MS  500

/* ================= 引入外部传感器数据 ================= */
extern int32_t g_can_current_ma;
/* ================= 收发线程 ================= */
static void rs485_trx_thread_entry(void *parameter)
{
    char send_buf[32];

    rs485_inst_t *hinst = rs485_create(APP_RS485_SERIAL, APP_RS485_BAUDRATE,
            APP_RS485_PARITY, APP_RS485_PIN, APP_RS485_LVL);
    if (hinst == RT_NULL) return;
    if (rs485_connect(hinst) != RT_EOK) { rs485_destory(hinst); return; }

    LOG_I("RS485 LED display ready. Showing CAN current on addr %03d.", LED_DISPLAY_ADDR);

    while (1)
    {
        snprintf(send_buf, sizeof(send_buf), "$%03d,%d#", LED_DISPLAY_ADDR, (int)g_can_current_ma);
        rs485_send(hinst, (void *)send_buf, strlen(send_buf));
        rt_thread_mdelay(LED_DISPLAY_PERIOD_MS);
    }
}

/* ================= 线程初始化 ================= */
static int rs485_app_init(void)
{
    rt_thread_t tid;

    tid = rt_thread_create("rs485_trx", rs485_trx_thread_entry, RT_NULL, 2048, 14, 10);

    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return RT_EOK;
    }

    LOG_E("Failed to create RS485 thread.");
    return -RT_ERROR;
}
INIT_APP_EXPORT(rs485_app_init);
