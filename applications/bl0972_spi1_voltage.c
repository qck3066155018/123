/*
 * BL0972 AC/DC voltage read example on SPI1.
 *
 * 功能说明：
 *   1. 通过 SPI1 读写 BL0972 的 24bit 寄存器。
 *   2. 支持 AC RMS 与 DC 双路电压两种测量模式。
 *   3. 自动采样线程会周期性更新原系统使用的全局电压变量。
 *   4. 根据母线电压和对地电压估算绝缘电阻，并输出绝缘故障标志。
 *
 * Hardware:
 *   SPI1_SCK  -> PB3
 *   SPI1_MISO -> PB4
 *   SPI1_MOSI -> PB5
 *   BL0972_CS -> PB6
 *   MODE_KEY  -> PC0，低电平按下，用于切换 AC/DC 测量模式
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_spi.h"

#define DBG_TAG "bl0972"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* BL0972 挂在 RT-Thread 的 SPI1 总线上，设备名为 bl0972。
 * rt_hw_spi_device_attach() 会把片选 GPIO 绑定到该 SPI 设备。
 */
#define BL0972_SPI_BUS_NAME     "spi1"
#define BL0972_SPI_DEV_NAME     "bl0972"
#define BL0972_CS_GPIO          GPIOB
#define BL0972_CS_PIN           GPIO_PIN_6
#define BL0972_MODE_KEY_PIN     GET_PIN(C, 0)

/* BL0972 SPI 命令字：写 0x81，读 0x82 */
#define BL0972_CMD_WRITE        0x81
#define BL0972_CMD_READ         0x82

/* 本例用到的寄存器地址。
 * I_RMS / V_RMS 是两个 24bit 有效值数据寄存器。
 * MODE1 用于选择有功波形来源，直流模式下要切到 LPF2。
 * MODE2 / MODE3 用于选择 RMS 输入源和 HPF 旁路等工作模式。
 * USR_WRPROT / SOFT_RESET 是配置寄存器前必须使用的控制寄存器。
 */
#define BL0972_REG_I_RMS        0x10
#define BL0972_REG_V_RMS        0x16
#define BL0972_REG_MODE1        0x96
#define BL0972_REG_MODE2        0x97
#define BL0972_REG_MODE3        0x98
#define BL0972_REG_USR_WRPROT   0x9E
#define BL0972_REG_SOFT_RESET   0x9F

/* 写保护解锁和软件复位命令值。
 * 写 MODE2/MODE3 前需要先按芯片要求执行复位、工作态切换和解锁。
 */
#define BL0972_WRPROT_UNLOCK    0x005555
#define BL0972_SOFT_RESET_START 0x5A5A5A
#define BL0972_SOFT_RESET_WORK  0x55AA55

/* 换算系数：当前采用线性拟合公式换算 BL0972 原始值。
 *
 * 计算关系：
 *   voltage = raw * slope + offset
 *
 * 如果后续需要重新标定两路通道，优先修改下面的斜率和零点偏移。
 */
#define BL0972_V_SLOPE          0.00005259f
#define BL0972_V_OFFSET         0.880843f
#define BL0972_I_SLOPE          0.00005259f
#define BL0972_I_OFFSET         0.880843f

/* ==================== 测量模式选择 ====================
 * AC 模式：正常交流 RMS 路径，适合测交流有效值。
 * DC 模式：旁路 HPF，把 V 通道和 I 通道都当成直流电压通道使用。
 *
 * 当前代码默认 g_bl0972_measure_mode = BL0972_MODE_DC。
 * 运行时按 PC0 按键可以在 AC/DC 之间切换，切换后会重新配置芯片寄存器。
 */
#define BL0972_MODE_AC          0
#define BL0972_MODE_DC          1
#define BL0972_UNIT_NAME        "V"



/* 绝缘检测计算中的内部等效电阻，单位 MOhm。 */
#define R_INTERNAL              2.00f
/* 母线中点附近的死区，单位 V；在中点附近认为绝缘状态正常，不做单边故障计算。 */
#define CENTER_DEADZONE_V       20.0f

/* 原系统里 OLED、云端上报、日志、报警模块都读取这些变量。
 * 这里继续保留原变量名，只是数据来源从 ADC 改为 BL0972。
 */
uint32_t g_adc_voltage_mv = 0;       /* 母线电压，单位 mV */
uint32_t g_adc_voltage_mv_ch1 = 0;   /* 负极对地电压，单位 mV */
float g_insul_res_mw = 999.0f;       /* 绝缘电阻，单位 MOhm */
uint8_t g_insul_fault_flag = 0;      /* 绝缘故障标志：1=异常，0=正常 */
uint8_t g_bl0972_measure_mode = BL0972_MODE_DC;
float g_th_insul_min = 0.45f;        /* 绝缘报警阈值，单位 MOhm */

/* bl0972_spi 保存已挂载的 SPI 设备指针，避免每次读写重复查找。
 * bl0972_thread 是手动调试打印线程，bl0972_sample_thread 是系统自动采样线程。
 */
static struct rt_spi_device *bl0972_spi;
static struct rt_thread bl0972_thread;
static struct rt_thread bl0972_sample_thread;
static rt_uint8_t bl0972_stack[1024];
static rt_uint8_t bl0972_sample_stack[1024];
static rt_uint8_t bl0972_mode_ready;
static rt_uint8_t bl0972_key_inited;

static const char *bl0972_get_mode_name(void)
{
    /* 统一模式名称，便于日志和 MSH 命令输出保持一致。 */
    return (g_bl0972_measure_mode == BL0972_MODE_DC) ? "DC" : "AC RMS";
}

static void bl0972_key_init(void)
{
    /* PC0 使用上拉输入：未按下时为高电平，按下时被拉低。 */
    if (!bl0972_key_inited)
    {
        rt_pin_mode(BL0972_MODE_KEY_PIN, PIN_MODE_INPUT_PULLUP);
        bl0972_key_inited = 1;
    }
}

static void bl0972_toggle_measure_mode(void)
{
    /* 修改模式后必须清除 ready 标志，下一次读寄存器时会重新写 MODE2/MODE3。 */
    g_bl0972_measure_mode = (g_bl0972_measure_mode == BL0972_MODE_DC) ?
                            BL0972_MODE_AC : BL0972_MODE_DC;
    bl0972_mode_ready = 0;
    rt_kprintf("BL0972 measure mode switched to %s.\n", bl0972_get_mode_name());
}

static void bl0972_key_scan(void)
{
    static rt_uint8_t key_last = PIN_HIGH;
    rt_uint8_t key_now;

    bl0972_key_init();
    key_now = rt_pin_read(BL0972_MODE_KEY_PIN);

    /* 只在高到低的边沿触发一次模式切换，并延时 20ms 做简单按键消抖。 */
    if (key_now == PIN_LOW && key_last == PIN_HIGH)
    {
        rt_thread_mdelay(20);
        if (rt_pin_read(BL0972_MODE_KEY_PIN) == PIN_LOW)
        {
            bl0972_toggle_measure_mode();
            key_last = PIN_LOW;
            return;
        }
    }

    key_last = key_now;
}

static rt_err_t bl0972_spi_init(void)
{
    struct rt_spi_configuration cfg;
    rt_device_t dev;

    /* 已初始化过则直接返回，后续读写共用同一个 SPI 设备对象。 */
    if (bl0972_spi != RT_NULL)
    {
        return RT_EOK;
    }

    /* PB6 作为 BL0972 的片选脚，使用前先打开 GPIOB 时钟。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 如果设备还没有挂载到 RT-Thread SPI 框架，则动态挂载到 spi1 总线。 */
    dev = rt_device_find(BL0972_SPI_DEV_NAME);
    if (dev == RT_NULL)
    {
        rt_err_t ret = rt_hw_spi_device_attach(BL0972_SPI_BUS_NAME,
                                               BL0972_SPI_DEV_NAME,
                                               BL0972_CS_GPIO,
                                               BL0972_CS_PIN);
        if (ret != RT_EOK)
        {
            LOG_E("attach %s to %s failed: %d",
                  BL0972_SPI_DEV_NAME, BL0972_SPI_BUS_NAME, ret);
            return ret;
        }
    }

    bl0972_spi = (struct rt_spi_device *)rt_device_find(BL0972_SPI_DEV_NAME);
    if (bl0972_spi == RT_NULL)
    {
        LOG_E("cannot find spi device %s", BL0972_SPI_DEV_NAME);
        return -RT_ERROR;
    }

    /* BL0972 要求 SPI Mode 1：CPOL=0，CPHA=1，MSB first。
     * 时钟先设置为 1 MHz，保证通信稳定；如硬件余量足够可按手册再提高。
     */
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_1 | RT_SPI_MSB;
    cfg.data_width = 8;
    cfg.max_hz = 1000 * 1000;
    rt_spi_configure(bl0972_spi, &cfg);

    return RT_EOK;
}

static rt_uint8_t bl0972_checksum(rt_uint8_t cmd, rt_uint8_t reg,
                                  const rt_uint8_t data[3])
{
    /* 校验和 = ~(CMD + ADDR + DATA_H + DATA_M + DATA_L)。
     * 读写帧都使用同一套校验规则，数据区固定 3 字节。
     */
    rt_uint8_t sum = cmd + reg + data[0] + data[1] + data[2];

    return (rt_uint8_t)(~sum);
}

static rt_err_t bl0972_write24(rt_uint8_t reg, rt_uint32_t value)
{
    rt_uint8_t data[3];
    rt_uint8_t tx[6];

    if (bl0972_spi_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* BL0972 的配置寄存器按 24bit 大端顺序传输：高字节先发。 */
    data[0] = (rt_uint8_t)((value >> 16) & 0xFF);
    data[1] = (rt_uint8_t)((value >> 8) & 0xFF);
    data[2] = (rt_uint8_t)(value & 0xFF);

    /* 写帧格式：0x81 + REG + DATA_H + DATA_M + DATA_L + CHECKSUM */
    tx[0] = BL0972_CMD_WRITE;
    tx[1] = reg;
    tx[2] = data[0];
    tx[3] = data[1];
    tx[4] = data[2];
    tx[5] = bl0972_checksum(BL0972_CMD_WRITE, reg, data);

    if (rt_spi_transfer(bl0972_spi, tx, RT_NULL, sizeof(tx)) != sizeof(tx))
    {
        LOG_E("spi write 0x%02x failed", reg);
        return -RT_ERROR;
    }

    return RT_EOK;
}

static rt_err_t bl0972_config_measure_mode(void)
{
    rt_uint32_t mode1;
    rt_uint32_t mode2;
    rt_uint32_t mode3;

    /* 当前模式已经成功写入芯片时不重复配置，减少 SPI 写操作。 */
    if (bl0972_mode_ready)
    {
        return RT_EOK;
    }

    /* 先软件复位，再进入工作态，然后解锁写保护，最后写 MODE1/MODE2/MODE3。 */
    if (bl0972_write24(BL0972_REG_SOFT_RESET, BL0972_SOFT_RESET_START) != RT_EOK)
    {
        return -RT_ERROR;
    }
    rt_thread_mdelay(10);

    if (bl0972_write24(BL0972_REG_SOFT_RESET, BL0972_SOFT_RESET_WORK) != RT_EOK)
    {
        return -RT_ERROR;
    }
    rt_thread_mdelay(50);

    if (bl0972_write24(BL0972_REG_USR_WRPROT, BL0972_WRPROT_UNLOCK) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (g_bl0972_measure_mode == BL0972_MODE_DC)
    {
        /* 直流双路电压模式：
         *   MODE1.WAVE_SEL：
         *     bit10 = 1：V 通道有功波形取直流 LPF2。
         *     bit4  = 1：I 通道有功波形取直流 LPF2。
         *   MODE2.WAVE_RMS_SEL：
         *     VWAVE_RMS_SEL[9:8]   = 10：V_RMS 选择直流。
         *     IWAVE_RMS_SEL[21:20] = 10：I_RMS 选择直流。
         *   MODE3.hpf_sel = 1：旁路高通滤波，保留直流分量。
         *
         * 这样可以把 BL0972 的两个差分输入都当作电压采样通道使用：
         *   CH1 = VP/VN，对应 V_RMS 寄存器；
         *   CH2 = IP/IN，对应 I_RMS 寄存器。
         */
        mode1 = ((rt_uint32_t)1 << 10) | ((rt_uint32_t)1 << 4);
        mode2 = ((rt_uint32_t)2 << 8) | ((rt_uint32_t)2 << 20);
        mode3 = ((rt_uint32_t)1 << 14);
    }
    else
    {
        /* 交流有效值模式：
         * MODE1/MODE2/MODE3 写 0，保持软件复位后的标准 AC RMS 测量路径。
         * 此模式下不旁路 HPF，适合读取交流有效值。
         */
        mode1 = 0x000000;
        mode2 = 0x000000;
        mode3 = 0x000000;
    }

    if (bl0972_write24(BL0972_REG_MODE1, mode1) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (bl0972_write24(BL0972_REG_MODE2, mode2) != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (bl0972_write24(BL0972_REG_MODE3, mode3) != RT_EOK)
    {
        return -RT_ERROR;
    }

    bl0972_mode_ready = 1;
    return RT_EOK;
}

static rt_err_t bl0972_read24(rt_uint8_t reg, rt_uint32_t *value)
{
    /* 读帧格式：
     *   MOSI: 0x82 + REG + 0xFF + 0xFF + 0xFF + 0xFF
     *   MISO: 前 2 字节无效，后面依次返回 DATA_H、DATA_M、DATA_L、CHECKSUM
     */
    rt_uint8_t tx[6] = {BL0972_CMD_READ, reg, 0xFF, 0xFF, 0xFF, 0xFF};
    rt_uint8_t rx[6] = {0};
    rt_uint8_t data[3];

    if (bl0972_spi_init() != RT_EOK)
    {
        return -RT_ERROR;
    }

    if (bl0972_config_measure_mode() != RT_EOK)
    {
        return -RT_ERROR;
    }

    /* rt_spi_transfer 会自动拉低/拉高绑定的片选脚，完成一次完整 SPI 帧传输。 */
    if (rt_spi_transfer(bl0972_spi, tx, rx, sizeof(tx)) != sizeof(tx))
    {
        LOG_E("spi transfer failed");
        return -RT_ERROR;
    }

    /* 取出 24bit 原始值，并用最后一个字节校验通信是否正确。 */
    data[0] = rx[2];
    data[1] = rx[3];
    data[2] = rx[4];

    if (rx[5] != bl0972_checksum(BL0972_CMD_READ, reg, data))
    {
        LOG_E("checksum failed, rx=0x%02x", rx[5]);
        return -RT_ERROR;
    }

    *value = ((rt_uint32_t)data[0] << 16) |
             ((rt_uint32_t)data[1] << 8) |
             ((rt_uint32_t)data[2]);

    return RT_EOK;
}

static float bl0972_read_ch1_mv(void)
{
    rt_uint32_t raw;

    if (bl0972_read24(BL0972_REG_V_RMS, &raw) != RT_EOK)
    {
        return -1.0f;
    }

    rt_kprintf("[BL0972] CH1 raw=0x%06x (%u)\n", raw, raw);

    /* CH1 使用电压通道 VP/VN，对应 V_RMS 寄存器。 */
    return BL0972_V_SLOPE * (float)raw + BL0972_V_OFFSET;
}

static float bl0972_read_ch2_mv(void)
{
    rt_uint32_t raw;

    if (bl0972_read24(BL0972_REG_I_RMS, &raw) != RT_EOK)
    {
        return -1.0f;
    }

    rt_kprintf("[BL0972] CH2 raw=0x%06x (%u)\n", raw, raw);

    /* CH2 使用电流通道 IP/IN，当第二路电压通道使用，对应 I_RMS 寄存器。 */
    return 1.032*(BL0972_I_SLOPE * (float)raw + BL0972_I_OFFSET);
}

rt_err_t bl0972_read_voltage_mv(rt_uint32_t *ch1_mv, rt_uint32_t *ch2_mv)
{
    float ch1;
    float ch2;

    if (ch1_mv == RT_NULL || ch2_mv == RT_NULL)
    {
        return -RT_ERROR;
    }

    ch1 = bl0972_read_ch1_mv();
    ch2 = bl0972_read_ch2_mv();
    if (ch1 < 0.0f || ch2 < 0.0f)
    {
        return -RT_ERROR;
    }

    /* 加 0.5f 后强转整数，相当于四舍五入到 1 mV。 */
    *ch1_mv = (rt_uint32_t)(ch1 + 0.5f);
    *ch2_mv = (rt_uint32_t)(ch2 + 0.5f);

    return RT_EOK;
}

static void bl0972_update_insulation(rt_uint32_t bus_voltage, rt_uint32_t pe_voltage)
{
    /* bus_voltage：母线总电压，单位 mV。
     * pe_voltage ：负极对地电压，单位 mV。
     *
     * 计算时先转成 V，便于和 CENTER_DEADZONE_V、R_INTERNAL 的单位保持一致。
     */
    float v1 = (float)bus_voltage;
    float v2 = (float)pe_voltage;
    float v_mid = v1 / 2.0f;
    float rx_calc = 999.0f;
    uint8_t fault = 0;

    /* 母线电压过低时绝缘计算可信度差，直接保持 999 MOhm、无故障。 */
    if (v1 > 50.0f)
    {
        /* v2 明显低于母线中点：说明一侧绝缘可能偏低，按下桥臂泄漏模型估算。 */
        if (v2 < (v_mid - CENTER_DEADZONE_V))
        {
            float den = v1 - 2.0f * v2;
            if (den > 0.01f)
            {
                rx_calc = (v2 / den) * R_INTERNAL;
                if (rx_calc < g_th_insul_min)
                {
                    fault = 1;
                }
            }
        }
        /* v2 明显高于母线中点：说明另一侧绝缘可能偏低，按上桥臂泄漏模型估算。 */
        else if (v2 > (v_mid + CENTER_DEADZONE_V))
        {
            float den = 2.0f * v2 - v1;
            if (den > 0.01f)
            {
                rx_calc = ((v1 - v2) / den) * R_INTERNAL;
                if (rx_calc < g_th_insul_min)
                {
                    fault = 1;
                }
            }
        }
    }

    /* 将结果写回全局变量，供 OLED、云端上报和报警逻辑读取。 */
    g_insul_res_mw = rx_calc;
    g_insul_fault_flag = fault;
}

static void bl0972_sample_entry(void *parameter)
{
    RT_UNUSED(parameter);
    bl0972_key_init();

    while (1)
    {
        rt_uint32_t ch1_mv = 0;
        rt_uint32_t ch2_mv = 0;

        bl0972_key_scan();
        if (bl0972_read_voltage_mv(&ch1_mv, &ch2_mv) == RT_EOK)
        {

            rt_uint32_t bus_voltage = ch1_mv;
            rt_uint32_t pe_voltage = ch2_mv;

            g_adc_voltage_mv_ch1 = pe_voltage;
            /* 原系统中的 g_adc_voltage_mv 表示母线有效电压。
             * 这里用两路测量值相减得到母线对负极参考后的电压，避免下溢。
             */
            if (bus_voltage > pe_voltage)
            {
                g_adc_voltage_mv = bus_voltage - pe_voltage;
            }
            else
            {
                g_adc_voltage_mv = 0;
            }

            bl0972_update_insulation(bus_voltage, pe_voltage);
        }

        rt_thread_mdelay(1000);
    }
}
static int bl0972_sample_init(void)
{
    rt_err_t ret;

    /* 系统启动后自动创建采样线程：
     *   优先级 22，时间片 10 tick。
     * 该线程负责维护全局电压和绝缘状态，是正常业务路径。
     */
    ret = rt_thread_init(&bl0972_sample_thread,
                         "bl0972_v",
                         bl0972_sample_entry,
                         RT_NULL,
                         bl0972_sample_stack,
                         sizeof(bl0972_sample_stack),
                         22,
                         10);
    if (ret == RT_EOK)
    {
        rt_thread_startup(&bl0972_sample_thread);
    }

    return ret;
}
INIT_APP_EXPORT(bl0972_sample_init);

static void bl0972_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
    bl0972_key_init();

    /* 调试打印线程：只在执行 bl0972_start 后运行，每秒打印一次两路原始换算电压。 */
    while (1)
    {
        float ch1 = bl0972_read_ch1_mv();
        float ch2 = bl0972_read_ch2_mv();

        if (ch1 >= 0.0f && ch2 >= 0.0f)
        {
            rt_kprintf("BL0972 %s CH1(VP/VN): %d.%03d %s, CH2(IP/IN): %d.%03d %s\n",
                       bl0972_get_mode_name(),
                       (int)ch1,
                       (int)((ch1 - (int)ch1) * 1000),
                       BL0972_UNIT_NAME,
                       (int)ch2,
                       (int)((ch2 - (int)ch2) * 1000),
                       BL0972_UNIT_NAME);
        }

        bl0972_key_scan();
        rt_thread_mdelay(1000);
    }
}

static int bl0972_start(void)
{
    static rt_uint8_t started = 0;
    rt_err_t ret;

    if (started)
    {
        /* 防止重复执行 MSH 命令导致同一个静态线程控制块被重复初始化。 */
        rt_kprintf("BL0972 sample thread already started.\n");
        return 0;
    }

    ret = bl0972_spi_init();
    if (ret != RT_EOK)
    {
        return ret;
    }

    ret = rt_thread_init(&bl0972_thread,
                         "bl0972",
                         bl0972_thread_entry,
                         RT_NULL,
                         bl0972_stack,
                         sizeof(bl0972_stack),
                         20,
                         10);
    if (ret == RT_EOK)
    {
        started = 1;
        rt_thread_startup(&bl0972_thread);
    }

    return ret;
}
MSH_CMD_EXPORT(bl0972_start, start BL0972 voltage sample thread);

static int bl0972_read_v(void)
{
    rt_uint32_t raw_v;
    rt_uint32_t raw_i;
    float ch1;
    float ch2;

    /* MSH 单次读取命令：直接打印寄存器 raw 值和换算后的 mV，方便标定排查。 */
    if (bl0972_read24(BL0972_REG_V_RMS, &raw_v) != RT_EOK)
    {
        rt_kprintf("BL0972 read CH1 voltage failed.\n");
        return -RT_ERROR;
    }

    if (bl0972_read24(BL0972_REG_I_RMS, &raw_i) != RT_EOK)
    {
        rt_kprintf("BL0972 read CH2 voltage failed.\n");
        return -RT_ERROR;
    }

    ch1 = BL0972_V_SLOPE * (float)raw_v + BL0972_V_OFFSET;
    ch2 = BL0972_I_SLOPE * (float)raw_i + BL0972_I_OFFSET;

    rt_kprintf("BL0972 %s CH1(VP/VN) raw=0x%06x, voltage=%d.%03d %s\n",
               bl0972_get_mode_name(),
               raw_v,
               (int)ch1,
               (int)((ch1 - (int)ch1) * 1000),
               BL0972_UNIT_NAME);
    rt_kprintf("BL0972 %s CH2(IP/IN) raw=0x%06x, voltage=%d.%03d %s\n",
               bl0972_get_mode_name(),
               raw_i,
               (int)ch2,
               (int)((ch2 - (int)ch2) * 1000),
               BL0972_UNIT_NAME);

    return RT_EOK;
}
MSH_CMD_EXPORT(bl0972_read_v, read BL0972 voltage once);

/* 默认不自动启动 BL0972 调试打印线程，避免和系统电压采集线程重复读取。
 * 需要单独调试时，在 FinSH/MSH 输入 bl0972_start 即可。
 */
INIT_APP_EXPORT(bl0972_start);
