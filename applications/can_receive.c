/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-03-10     Y8314       the first version
 */

#include <rtthread.h>
#include "rtdevice.h"
#include "ccm_alloc.h"

#define CAN_DEV_NAME       "can1"      /* CAN 设备名称 */
#define TARGET_CAN_ID      0x3C2       /* 目标传感器 ID */

static struct rt_thread can_thread_obj;
static struct rt_semaphore rx_sem;     /* 用于接收消息的信号量 */
static rt_device_t can_dev;            /* CAN 设备句柄 */

/* 新增全局变量，供 OLED 显示线程读取 */
int32_t g_can_current_ma = 0;

/* 接收数据回调函数 */
static rt_err_t can_rx_call(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

/* CAN 接收与解析线程 */
static void can_rx_thread(void *parameter)
{
    struct rt_can_msg rxmsg = {0};

    rt_device_set_rx_indicate(can_dev, can_rx_call);

    while (1)
    {
        rxmsg.hdr = -1;

        /* 如果超过 2000 毫秒没有收到目标 CAN 数据，就认为掉线。*/
        if (rt_sem_take(&rx_sem, rt_tick_from_millisecond(2000)) != RT_EOK)
        {
            /* 发生超时，说明传感器断开或者停止发送，归零全局变量 */
            g_can_current_ma = 0;
            continue; /* 跳过本次数据读取，继续下一轮等待 */
        }

        rt_device_read(can_dev, 0, &rxmsg, sizeof(rxmsg));

        if (rxmsg.id == TARGET_CAN_ID && rxmsg.len >= 5)
        {
            uint32_t raw_value = ((uint32_t)rxmsg.data[0] << 24) |
                    ((uint32_t)rxmsg.data[1] << 16) |
                    ((uint32_t)rxmsg.data[2] << 8)  |
                    ((uint32_t)rxmsg.data[3]);

            int32_t current_mA = (int32_t)(raw_value - 0x80000000);
            uint8_t error_flag = rxmsg.data[4];

            /* ================= 新增：零点底噪过滤（死区钳位）================= */
            /* 假设 2mA 以内的波动都视为没电流（噪声），强行归零 */
            if (current_mA >= -2 && current_mA <= 2)
            {
                current_mA = 0;
            }
            /* ============================================================= */

            if (error_flag == 0)
            {
                /* 正常情况：更新到全局变量供 OLED 读取 */
                g_can_current_ma = current_mA;
                // rt_kprintf("[CAN] ID:0x%03X | Current: %d mA\n", rxmsg.id, current_mA);
            }
            else
            {
                rt_kprintf("[CAN] ID:0x%03X | Sensor Error! Code: 0x%02X\n", rxmsg.id, error_flag);
            }
        }
    }
}

int can_app_init(void)
{
    rt_err_t res;

    can_dev = rt_device_find(CAN_DEV_NAME);
    if (!can_dev) return RT_ERROR;

    res = rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN250kBaud);
    if (res != RT_EOK) return RT_ERROR;

    struct rt_can_filter_item filter_item = {TARGET_CAN_ID, 0, 0, 1, 0x7FF, RT_NULL, RT_NULL};
    struct rt_can_filter_config filter_cfg = {1, 1, &filter_item};
    rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &filter_cfg);

    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);

    res = rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);
    if (res != RT_EOK) return res;

    /* 使用 CCM RAM 创建线程 */
    void *stack = rt_ccm_malloc(512);
    if (stack == RT_NULL) return -RT_ERROR;

    res = rt_thread_init(&can_thread_obj, "can_rx", can_rx_thread, RT_NULL, stack, 512, 13, 10);
    if (res == RT_EOK)
    {
        rt_thread_startup(&can_thread_obj);
        return RT_EOK;
    }
    return RT_ERROR;
}
INIT_APP_EXPORT(can_app_init);
