/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-5-10      ShiHao       first version
 */
//version=2018-10-31&res=products%2FifK796hQfB%2Fdevices%2F11&et=94654654261354654&method=md5&sign=yFa4xPQzg5oEvgxEKCbR4w%3D%3D
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#define DBG_TAG "main"
#define DBG_LVL         DBG_LOG
#include <rtdbg.h>

/* 配置 LED 灯引脚 */
#define PIN_LED_G              GET_PIN(D, 8)      // D8绿灯
#define PIN_LED_Y              GET_PIN(B, 15)      // B14黄灯
#define PIN_LED_R              GET_PIN(B, 14)      // B14红灯

int main(void)
{
    unsigned int count = 1;

    /* 设置 LED 引脚为输出模式 */
    rt_pin_mode(PIN_LED_G, PIN_MODE_OUTPUT);

    while (count > 0)
    {
        /* LED 灯亮 */
        rt_pin_write(PIN_LED_G, PIN_LOW);
       // LOG_D("led on, count: %d", count);
        rt_thread_mdelay(500);

        /* LED 灯灭 */
        rt_pin_write(PIN_LED_G, PIN_HIGH);
        //LOG_D("led off");
        rt_thread_mdelay(500);

        count++;
    }

    return 0;
}

#include <fcntl.h>
#include <unistd.h>
#include <rtthread.h>

void test_udisk_write(void)
{
    int fd;
    rt_kprintf("Start writing to U-disk...\n");

    // 只打开并创建一个极其简单的文件
    fd = open("/my_test.txt", O_WRONLY | O_CREAT);
    if (fd >= 0)
    {
        write(fd, "OK", 2); // 只写2个字节
        close(fd);
        rt_kprintf("Write Success!\n");
    }
    else
    {
        rt_kprintf("Open Failed!\n");
    }
}
// 将这个函数导出到 MSH 控制台
MSH_CMD_EXPORT(test_udisk_write, a simple write test);

