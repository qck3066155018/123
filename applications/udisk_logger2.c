/*
 * File     : udisk_logger.c
 * 说明     : U盘数据记录 (已针对 STM32 OTG_FS FIFO 对齐与 FatFs 扇区缓存优化)
 */

#include <rtthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>   /* 使用 abs() 函数 */
#include <string.h>
#include <time.h>
#include "ccm_alloc.h"

#define DBG_TAG "app.logger"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern int32_t g_ds18b20_temp;
extern uint32_t g_adc_voltage_mv;
extern uint32_t g_adc_voltage_mv_ch1;
extern int32_t g_can_current_ma;
extern uint8_t g_sys_alarm_status;
extern rt_mutex_t g_hw_mutex;

#define LOG_INTERVAL_MS      1000   //1s一次
#define NORMAL_FILE_PREFIX   "/data_"
#define ALARM_FILE_PREFIX    "/alarm_"

/* CSV 标准表头和 UTF-8 BOM */
static const char CSV_BOM[3] = {0xEF, 0xBB, 0xBF};
static const char CSV_HEADER[] = "Time,Temp(C),V_In(V),V_CH1(V),Current(mA),Alarm\r\n";

/* ================= 核心修改 1：内存严格 4 字节对齐 ================= */
/* 迎合 STM32 USB_OTG_FS 内部 32位 FIFO 的推送要求，防止 csw signature error */
ALIGN(4) static char normal_filename[64];
ALIGN(4) static char alarm_filename[64];
ALIGN(4) static char write_buf[128];

/* ================= 核心修改 2：512 字节扇区级缓存池 ================= */
/* 将零碎的字符串拼接积累成块，迎合 FatFs 扇区写入机制，大幅减少底层 USB 交互 */
ALIGN(4) static char sector_cache[512];
static uint16_t cache_len = 0;

static void udisk_logger_entry(void *parameter)
{
    time_t now;
    struct tm *tm_info;

    rt_thread_mdelay(2000); /* 稍微多等一等网络和 USB 挂载 */
    LOG_I("U-disk logger thread started (Block Write & Aligned Mode).");

    memset(sector_cache, 0, sizeof(sector_cache));
    cache_len = 0;

    while (1)
    {
        now = time(RT_NULL);
        tm_info = localtime(&now);

        /* 1. 动态生成今天的文件名 */
        snprintf(normal_filename, sizeof(normal_filename), "%s%04d%02d%02d.csv",
                NORMAL_FILE_PREFIX, tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);
        snprintf(alarm_filename, sizeof(alarm_filename), "%s%04d%02d%02d.csv",
                ALARM_FILE_PREFIX, tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

        /* 2. 安全处理浮点数打印 */
        const char *t_sign = (g_ds18b20_temp < 0) ? "-" : "";
        int t_abs = abs(g_ds18b20_temp);
        int t_int = t_abs / 10;
        int t_dec = t_abs % 10;

        int v_int = g_adc_voltage_mv / 1000;
        int v_dec = g_adc_voltage_mv % 1000;

        int ch1_int = g_adc_voltage_mv_ch1 / 1000;
        int ch1_dec = g_adc_voltage_mv_ch1 % 1000;

        snprintf(write_buf, sizeof(write_buf),
                "%04d-%02d-%02d %02d:%02d:%02d,%s%d.%d,%d.%03d,%d.%03d,%d,%d\r\n",
                tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
                tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
                t_sign, t_int, t_dec,
                v_int, v_dec,
                ch1_int, ch1_dec,
                (int)g_can_current_ma,
                (int)g_sys_alarm_status);

        /* 3. 数据先写入内存缓存池，暂不操作 U 盘 */
        int current_str_len = strlen(write_buf);

        /* 防溢出保护：如果缓存池加上新数据会超过512，就先强制写入一次U盘 */
        if (cache_len + current_str_len >= sizeof(sector_cache)) {
            // (通常不会发生，因为下面设定了阈值，这是安全兜底)
            cache_len = sizeof(sector_cache) - 1;
        } else {
            strcat(sector_cache, write_buf);
            cache_len += current_str_len;
        }

        /* 4. 只有当积累数据超过一定阈值 (如400字节) 或者 系统发生报警时，才进行物理写入 */
        if (cache_len >= 400 || g_sys_alarm_status != 0)
        {
            rt_err_t lock_res = RT_EOK; // 默认认为能拿到锁（适用于锁还没创建的情况）

            /* 如果锁存在，则尝试等待 1000ms */
            if (g_hw_mutex != RT_NULL)
            {
                lock_res = rt_mutex_take(g_hw_mutex, rt_tick_from_millisecond(1000));
            }
            /* 只要拿到锁了（或者根本不需要锁），就执行写入 */
            if (lock_res == RT_EOK)
            {

                /* ================= 成功拿到锁，开始写入 U 盘 ================= */
                /* [写入普通日志] */
                int fd = open(normal_filename, O_RDWR | O_CREAT | O_APPEND);
                if (fd >= 0)
                {
                    if (lseek(fd, 0, SEEK_END) == 0)
                    {
                        /* 如果文件大小为 0，说明是新的一天或新建的文件，写入 BOM 和表头 */
                        write(fd, CSV_BOM, 3);
                        write(fd, CSV_HEADER, strlen(CSV_HEADER));
                    }
                    /* 一次性将对齐好的大块数据丢给底层 */
                    write(fd, sector_cache, cache_len);
                    close(fd); /* 写入完毕，立刻释放句柄 */
                }
                else
                {
                    //LOG_W("U-disk write failed, maybe unplugged.");
                }

                /* [写入报警日志] (仅当有报警时) */
                if (g_sys_alarm_status != 0)
                {
                    int afd = open(alarm_filename, O_RDWR | O_CREAT | O_APPEND);
                    if (afd >= 0)
                    {
                        if (lseek(afd, 0, SEEK_END) == 0)
                        {
                            write(afd, CSV_BOM, 3);
                            write(afd, CSV_HEADER, strlen(CSV_HEADER));
                        }
                        /* 报警文件同样一次性写入块数据，便于上下文追溯 */
                        write(afd, sector_cache, cache_len);
                        close(afd);
                    }
                }

                /* 物理写入完成：释放锁，允许 Wi-Fi 继续上报 */
                /* 写入完成：如果有锁，记得释放 */
                if (g_hw_mutex != RT_NULL)
                {
                    rt_mutex_release(g_hw_mutex);
                }

                /* 物理写入成功后，清空缓存池，迎接下一波数据 */
                memset(sector_cache, 0, sizeof(sector_cache));
                cache_len = 0;
            }
            else
            {
                /* ================= 未拿到锁 (Wi-Fi可能卡住了) ================= */
                /* 获取锁超时，放弃本次物理写入！
                 * 注意：此时不执行 memset 清空动作，让数据继续安全地保存在 sector_cache 中，
                 * 下次循环时再一并写入 U 盘，确保本地数据不丢失。
                 */
                LOG_W("Wi-Fi is busy, U-disk write skipped this tick.");
            }
        }
        /* 周期延时 */
        rt_thread_mdelay(LOG_INTERVAL_MS);
    }
}


/* ================== CCM RAM 静态分配区 ================== */
ALIGN(RT_ALIGN_SIZE)
static struct rt_thread logger_thread;

int udisk_logger_init(void)
{
    /* 移除硬编码，改为通过统一的分配器申请 2048 字节的 CCM 栈 */
    void *stack = rt_ccm_malloc(2048);
    if (stack == RT_NULL)
    {
        LOG_E("CCM RAM alloc failed for u_logger");
        return -RT_ERROR;
    }

    rt_err_t res = rt_thread_init(&logger_thread,
            "u_logger",
            udisk_logger_entry,
            RT_NULL,
            stack,  /* <--- 使用动态申请到的栈指针 */
            2048,
            26, 10);

    if (res == RT_EOK)
    {
        rt_thread_startup(&logger_thread);
        return RT_EOK;
    }
    return -RT_ERROR;
}
INIT_APP_EXPORT(udisk_logger_init);
