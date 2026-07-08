/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2019-07-24     WillianChan      the first version
 * 2020-07-28     WillianChan      add the inclusion of the board.h
 * 2026-xx-xx     User             modify to auto-start thread
 */

#include <stdlib.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "dallas_ds18b20_sensor_v1.h"
#include "ccm_alloc.h"

/* Modify this pin according to the actual wiring situation */
#define DS18B20_DATA_PIN    GET_PIN(E, 11)

/* 新增全局变量，供 OLED 显示线程读取 */
int32_t g_ds18b20_temp = 0;
static struct rt_thread ds18b20_thread_obj;

static void read_temp_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;

    /* 用于突跳过滤的静态变量 */
    static int32_t last_valid_temp = 0;
    static uint8_t is_first_read = 1;
    /* 延时等待底层传感器设备注册完成 */
    rt_thread_mdelay(1000);

    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        rt_kprintf("Can't find device:%s\n", parameter);
        return;
    }

    if (rt_device_open(dev, RT_DEVICE_FLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("open device failed!\n");
        return;
    }
    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);

    /* 改为死循环，实现自动持续触发读取 */
    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1)
        {
            rt_kprintf("read data failed!size is %d\n", res);
            rt_thread_mdelay(1000); /* 读取失败时增加延时，防止死循环刷屏 */
            continue;
        }


        int32_t current_temp = sensor_data.data.temp;
        /* ========== 1. 经典硬件错误过滤 ========== */
        if (current_temp == 850)
        {
            rt_kprintf("[DS18B20] Reset error 85.0C detected, ignore.\n");
            rt_thread_mdelay(1000);
            continue;
        }

        /* ========== 2. 突跳（跳变）过滤 ========== */
        if (!is_first_read)
        {
            /* 计算与上次有效温度的绝对差值 */
            int32_t diff = abs(current_temp - last_valid_temp);

            /* 设定突跳阈值为 15.0℃ (即150)。1秒内温度跳变超过此值纯属扯淡，直接拦截 */
            if (diff > 150)
            {
                rt_kprintf("[DS18B20] Jump Filter Blocked! Cur:%d.%dC, Last:%d.%dC\n",
                        current_temp / 10, abs(current_temp) % 10,
                        last_valid_temp / 10, abs(last_valid_temp) % 10);

                rt_thread_mdelay(1000);
                continue; /* 丢弃该异常数据，不更新全局变量，直接等下一轮 */
            }
        }

        /* ========== 3. 数据安全，更新全局状态 ========== */
        last_valid_temp = current_temp; /* 更新历史有效值 */
        is_first_read = 0;              /* 标记已完成首次读取 */

        g_ds18b20_temp = current_temp;  /* 将干净的数据交给 OLED 和监控模块 */

        /* 保留串口打印，方便调试核对数据 */
        //        if (sensor_data.data.temp >= 0)
        //        {
        //            rt_kprintf("[DS18B20] temp:%3d.%dC, timestamp:%5d\n",
        //                       sensor_data.data.temp / 10,
        //                       sensor_data.data.temp % 10,
        //                       sensor_data.timestamp);
        //        }
        //        else
        //        {
        //            rt_kprintf("[DS18B20] temp:-%2d.%dC, timestamp:%5d\n",
        //                       abs(sensor_data.data.temp / 10),
        //                       abs(sensor_data.data.temp % 10),
        //                       sensor_data.timestamp);
        //        }

        /* 调整读取频率为 1000ms，与 OLED 的刷新频率对齐以节省 CPU 资源 */
        rt_thread_mdelay(1000);
    }
}

/* 自动启动线程的应用层初始化函数 */
int ds18b20_app_init(void)
{
    void *stack = rt_ccm_malloc(1024);
    if (stack == RT_NULL) return -RT_ERROR;

    rt_err_t res = rt_thread_init(&ds18b20_thread_obj,
            "18b20tem",
            read_temp_entry,
            "temp_ds18b20",
            stack,
            1024,
            25,
            20);
    if (res == RT_EOK)
    {
        rt_thread_startup(&ds18b20_thread_obj);
        return RT_EOK;
    }
    rt_kprintf("init ds18b20 thread failed!\n");
    return -RT_ERROR;
}
INIT_APP_EXPORT(ds18b20_app_init);

/* 底层硬件端口初始化保持不变 */
static int rt_hw_ds18b20_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.user_data = (void *)DS18B20_DATA_PIN;
    rt_hw_ds18b20_init("ds18b20", &cfg);

    return RT_EOK;
}
/* 在组件阶段完成硬件接口的注册，确保先于应用层初始化 */
INIT_COMPONENT_EXPORT(rt_hw_ds18b20_port);
