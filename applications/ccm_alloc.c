/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-04-05     Y8314       the first version
 */
#include <rtthread.h>
#include "ccm_alloc.h"

#define DBG_TAG "ccm_alloc"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* STM32F4 CCM RAM 起始地址和大小 ( 64KB ) */
#define CCM_RAM_START 0x10000000
#define CCM_RAM_SIZE  (64 * 1024)

static rt_size_t ccm_offset = 0;

void* rt_ccm_malloc(rt_size_t size)
{
    rt_base_t level;
    void *ptr = RT_NULL;

    /* 必须保证 4 字节对齐，防止硬件异常 */
    size = RT_ALIGN(size, RT_ALIGN_SIZE);

    /* 关中断，确保多线程下分配安全 */
    level = rt_hw_interrupt_disable();
    if ((ccm_offset + size) <= CCM_RAM_SIZE)
    {
        ptr = (void *)(CCM_RAM_START + ccm_offset);
        ccm_offset += size;
    }
    rt_hw_interrupt_enable(level);

    return ptr;
}

/* 自定义一个名为 free_ccm 的命令 */
void free_ccm(void)
{
    rt_kprintf("================ CCM RAM Status ================\n");
    rt_kprintf("Total CCM RAM : %d Bytes (64 KB)\n", CCM_RAM_SIZE);
    rt_kprintf("Used  CCM RAM : %d Bytes\n", ccm_offset);
    rt_kprintf("Free  CCM RAM : %d Bytes\n", CCM_RAM_SIZE - ccm_offset);

    /* 顺便算一下利用率 */
    int percent = (ccm_offset * 100) / CCM_RAM_SIZE;
    rt_kprintf("Usage Percent : %d %%\n", percent);
    rt_kprintf("================================================\n");
}
/* 导出为 MSH 控制台命令 */
MSH_CMD_EXPORT(free_ccm, show CCM RAM memory usage);
