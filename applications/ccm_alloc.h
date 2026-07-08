/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-04-05     Y8314       the first version
 */
#ifndef APPLICATIONS_CCM_ALLOC_H_
#define APPLICATIONS_CCM_ALLOC_H_

#include <rtthread.h>

/* CCM RAM 专用分配函数 */
void* rt_ccm_malloc(rt_size_t size);

#endif /* APPLICATIONS_CCM_ALLOC_H_ */
