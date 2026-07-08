/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-03-28     Y8314       the first version
 * 2026-05-14     Y8314       add 4G RTU fallback
 */
/*
 * File      : wifi_onenet_app.c
 * 说明      : RW007 Wi-Fi 连接与 4G数据上报线程
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <wlan_mgnt.h>
#include <wlan_prot.h>
#include <wlan_cfg.h>
#include <arpa/inet.h>
#include <netdev.h>
#include <stdlib.h>
#include "onenet.h"

#define DBG_TAG "app.cloud"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* =================  Wi-Fi 账号密码 ================= */
#define WIFI_SSID       "123"
#define WIFI_PASSWORD   "12345678"
/* ================= 4G 模块串口配置 ================= */
#define UART_4G_NAME    "uart3"  //  ML307R 实际连接的串口号
static rt_device_t uart_4g_dev = RT_NULL;

/* ================= 引入安防系统的传感器全局变量 ================= */
extern int32_t g_ds18b20_temp;
extern uint32_t g_adc_voltage_mv;
extern uint32_t g_adc_voltage_mv_ch1;
extern int32_t g_can_current_ma;
extern uint8_t g_sys_alarm_status;

/*全局硬件资源互斥锁 */
rt_mutex_t g_hw_mutex = RT_NULL;

/* 检查 Wi-Fi 网卡是否获取到 IP 并处于连网状态 */
static rt_bool_t is_wifi_available(void)
{
    struct netdev *netdev = netdev_get_by_name("w0");
    if (netdev != RT_NULL && netdev_is_internet_up(netdev))
    {
        return RT_TRUE;
    }
    return RT_FALSE;
}

/* 初始化 4G 模块所在的串口 */
static int init_4g_uart(void)
{
    uart_4g_dev = rt_device_find(UART_4G_NAME);
    if (uart_4g_dev != RT_NULL)
    {
        /* 以中断接收和轮询发送模式打开串口 */
        return rt_device_open(uart_4g_dev, RT_DEVICE_FLAG_INT_RX);
    }
    LOG_E("Can't find 4G UART device: %s", UART_4G_NAME);
    return -RT_ERROR;
}

/* 通过 4G RTU 透传发送 JSON 数据 */
static void upload_via_4g_rtu(int32_t temp, uint32_t v0, uint32_t v1, int32_t current, uint8_t alarm)
{
    if (uart_4g_dev == RT_NULL) return;

    char json_buf[300];

    /* 处理温度浮点数转换 (避免在某些精简 C 库中使用 %f 导致死机) */
    int temp_int = temp;

    /* * 构建包含多个属性的 JSON 字符串，一次性打包发送节省流量和时间。
     * 变量标识符(temperature, voltage_0等)与 OneNET 平台上的物模型名称保持一致。
     */
    rt_snprintf(json_buf, sizeof(json_buf),
                "{\"id\":\"1\",\"params\":{\"temp_value\":{\"value\":%d},\"voltage_0\":{\"value\":%lu},\"voltage_1\":{\"value\":%lu},\"current\":{\"value\":%ld},\"alarm_status\":{\"value\":%u}}}\r\n",
                temp_int,  v0, v1, current, alarm);

    /* 写入串口 */
    rt_device_write(uart_4g_dev, 0, json_buf, rt_strlen(json_buf));

    //LOG_I("[4G Link] TX: %s", json_buf);
}

static void onenet_upload_entry(void *parameter)
{
    /* 1. 给系统和驱动一点启动时间 */
    rt_thread_mdelay(3000);

    /* 2. 初始化 4G 串口 */
    init_4g_uart();

    /* 3. 主动连接 Wi-Fi 路由器 (非阻塞尝试) */
    LOG_I("Connecting to Wi-Fi: %s ...", WIFI_SSID);
    if (rt_wlan_connect(WIFI_SSID, WIFI_PASSWORD) != RT_EOK)
    {
        LOG_W("Wi-Fi connect failed! Will use 4G as fallback.");
    }

    /* 记录 OneNET MQTT 是否已经初始化过的标志位 */
    rt_bool_t wifi_mqtt_inited = RT_FALSE;

    /* 4. 进入数据上报主循环 */
    while (1)
    {
        /* 统一在循环开头获取一次所有传感器数据 */
        /* 【修复重点】：变量必须声明在 if/else 的外部，这样 Wi-Fi 和 4G 链路才能都访问到它们 */
        if (g_hw_mutex != RT_NULL) rt_mutex_take(g_hw_mutex, RT_WAITING_FOREVER);


        uint32_t temp = g_ds18b20_temp / 10; // 得到整数部分：31
        uint32_t temp_dec = g_ds18b20_temp % 10; // 得到小数部分：1
        uint32_t v0   = g_adc_voltage_mv;
        uint32_t v1   = g_adc_voltage_mv_ch1;
        int32_t cur   = g_can_current_ma;
        //uint8_t alarm = g_sys_alarm_status; // 恢复 alarm 变量
        uint8_t alarm = 1;
        if (g_hw_mutex != RT_NULL) rt_mutex_release(g_hw_mutex);

        /* ================= 主备链路切换逻辑 ================= */
        if (is_wifi_available())
        {
            if (!wifi_mqtt_inited)
            {
                LOG_I("Wi-Fi is ready! Starting OneNET initialization...");
                if (g_hw_mutex != RT_NULL) rt_mutex_take(g_hw_mutex, RT_WAITING_FOREVER);
                onenet_mqtt_init();
                if (g_hw_mutex != RT_NULL) rt_mutex_release(g_hw_mutex);
                wifi_mqtt_inited = RT_TRUE;
                rt_thread_mdelay(3000); // 握手延时
            }

            /* 1. 定义足够大的缓冲区 */
            char pub_payload[512];
            rt_snprintf(pub_payload, sizeof(pub_payload),
                        "{\"id\":\"1\",\"version\":\"1.0\",\"params\":{\"temp_value\":{\"value\":\"%lu.%lu\"},\"voltage_0\":{\"value\":\"%lu\"},\"voltage_1\":{\"value\":\"%lu\"},\"current\":{\"value\":\"%ld\"}}}",
                        temp, temp_dec, v0, v1, alarm);

            /* 3. 构造 Topic */
            char pub_topic[128];
            rt_snprintf(pub_topic, sizeof(pub_topic), "$sys/%s/%s/thing/property/post", "Qai608dU21", "vol");

            /* 4. 打印出来确认 */
            LOG_I("Target Topic: %s", pub_topic);
            LOG_I("Upload Payload: %s", pub_payload);

            /* 5. 执行上传 */
            rt_err_t res = RT_ERROR;
            if (g_hw_mutex != RT_NULL) rt_mutex_take(g_hw_mutex, RT_WAITING_FOREVER);
            res = onenet_mqtt_publish(pub_topic, (uint8_t *)pub_payload, rt_strlen(pub_payload));
            if (g_hw_mutex != RT_NULL) rt_mutex_release(g_hw_mutex);

            if (res == RT_EOK) {
                LOG_I("MQTT Publish SUCCESS! Check OneNET Web.");
            } else {
                LOG_E("MQTT Publish FAILED! error code: %d", res);
            }
        }
        else
        {
            /* 如果 Wi-Fi 没网，切换到 4G 串口透传 */
            upload_via_4g_rtu(temp, v0, v1, cur, alarm);
        }

        /* ================= 延时控制 ================= */
        /* 如果系统触发报警，加快上报频率至 2 秒；正常情况下 10 秒一报 */
        if (alarm != 0)
        {
            rt_thread_mdelay(2000);
        }
        else
        {
            rt_thread_mdelay(10000);
        }
    }
}

int wifi_onenet_app_init(void)
{
    rt_thread_t tid;

    /* 创建互斥锁 */
    if (g_hw_mutex == RT_NULL)
    {
        g_hw_mutex = rt_mutex_create("hw_mux", RT_IPC_FLAG_PRIO);
    }

    /* 注意：涉及 Wi-Fi 握手、DNS 解析、TLS 加密和 MQTT 协议，栈空间给到 4096 */
    tid = rt_thread_create("cloud_up", onenet_upload_entry, RT_NULL, 4096, 23, 10);
    if (tid != RT_NULL)
    {
        rt_thread_startup(tid);
        return RT_EOK;
    }
    return -RT_ERROR;
}
INIT_APP_EXPORT(wifi_onenet_app_init);
