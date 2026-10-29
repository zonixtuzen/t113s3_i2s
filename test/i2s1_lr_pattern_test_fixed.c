/*
 * i2s1_lr_pattern_test_fixed.c
 *
 * Allwinner T113-S3 / RT-Thread I2S1 diagnostic.
 *
 * MSH commands:
 *   i2s1_lr_pattern_test start
 *   i2s1_lr_pattern_test stop
 *   i2s1_lr_pattern_test stats
 *   i2s1_lr_pattern_test dump
 *
 * Purpose:
 *   Restore the known-good I2S1 register configuration before feeding
 *   alternating 32-bit words into TX FIFO.
 *
 * Important:
 *   - This file configures I2S controller registers only.
 *   - It does NOT configure CCU clock registers or PG12/PG13/PG15 pinmux.
 *     Run the existing clock and pinmux tests first.
 *   - Do not run another software/DMA producer on I2S1 TX FIFO concurrently.
 *   - Diagnostic code only; not a production audio driver.
 */

#include <rtthread.h>
#include <stdint.h>

#define I2S1_BASE           0x02033000UL

#define I2S1_CTL            (*(volatile uint32_t *)(I2S1_BASE + 0x00))
#define I2S1_FMT0           (*(volatile uint32_t *)(I2S1_BASE + 0x04))
#define I2S1_FMT1           (*(volatile uint32_t *)(I2S1_BASE + 0x08))
#define I2S1_ISTA           (*(volatile uint32_t *)(I2S1_BASE + 0x0C))
#define I2S1_FCTL           (*(volatile uint32_t *)(I2S1_BASE + 0x14))
#define I2S1_FSTA           (*(volatile uint32_t *)(I2S1_BASE + 0x18))
#define I2S1_INT            (*(volatile uint32_t *)(I2S1_BASE + 0x1C))
#define I2S1_TXFIFO         (*(volatile uint32_t *)(I2S1_BASE + 0x20))
#define I2S1_CLKD           (*(volatile uint32_t *)(I2S1_BASE + 0x24))
#define I2S1_TXCNT          (*(volatile uint32_t *)(I2S1_BASE + 0x28))
#define I2S1_CHCFG          (*(volatile uint32_t *)(I2S1_BASE + 0x30))
#define I2S1_TX0CHSEL       (*(volatile uint32_t *)(I2S1_BASE + 0x34))

#define I2S1_TXEN_BIT       (1U << 2)
#define I2S1_TXFIFO_DEPTH   128U

#define LR_LEFT_WORD        0x00000000U
#define LR_RIGHT_WORD       0xFFFFFFFFU

#define LR_STACK_SIZE       2048
#define LR_PRIORITY         (RT_THREAD_PRIORITY_MAX / 2)
#define LR_TICK             10

static rt_thread_t s_lr_thread = RT_NULL;
static volatile rt_bool_t s_lr_running = RT_FALSE;
static volatile rt_uint32_t s_lr_words_written = 0;
static volatile rt_uint32_t s_lr_full_polls = 0;
static volatile rt_uint32_t s_lr_last_free_words = I2S1_TXFIFO_DEPTH;
static volatile rt_uint32_t s_lr_last_phase = 0;
static volatile rt_uint32_t s_lr_last_txcnt = 0;

/*
 * Restore the register values from the previously tested 384-kHz setup.
 *
 * Keep TX disabled while programming the format/channel/divider registers.
 * The CCU module clock and pinmux must already have been configured.
 */
static void i2s1_lr_restore_registers(void)
{
    /* Disable TXEN while retaining the other known-good CTL bits. */
    I2S1_CTL = 0x00060111U;

    I2S1_FMT0     = 0x00001F57U;
    I2S1_FMT1     = 0x00000030U;
    I2S1_FCTL     = 0x000500F0U;
    I2S1_INT      = 0x00000000U;
    I2S1_CLKD     = 0x00000010U;
    I2S1_CHCFG    = 0x00000001U;
    I2S1_TX0CHSEL = 0x00110003U;

    /* Restore the known-good CTL value, including TXEN. */
    I2S1_CTL = 0x00060115U;

    rt_thread_mdelay(1);
}

static void i2s1_lr_dump_registers(void)
{
    uint32_t fsta = I2S1_FSTA;
    uint32_t ctl = I2S1_CTL;

    rt_kprintf("\n=== I2S1 register dump ===\n");
    rt_kprintf("CTL      = 0x%08x\n", (unsigned int)ctl);
    rt_kprintf("FMT0     = 0x%08x\n", (unsigned int)I2S1_FMT0);
    rt_kprintf("FMT1     = 0x%08x\n", (unsigned int)I2S1_FMT1);
    rt_kprintf("ISTA     = 0x%08x\n", (unsigned int)I2S1_ISTA);
    rt_kprintf("FCTL     = 0x%08x\n", (unsigned int)I2S1_FCTL);
    rt_kprintf("FSTA     = 0x%08x\n", (unsigned int)fsta);
    rt_kprintf("TXE_CNT  = %u words free\n",
               (unsigned int)((fsta >> 16) & 0xFFU));
    rt_kprintf("INT      = 0x%08x\n", (unsigned int)I2S1_INT);
    rt_kprintf("CLKD     = 0x%08x\n", (unsigned int)I2S1_CLKD);
    rt_kprintf("TXCNT    = 0x%08x\n", (unsigned int)I2S1_TXCNT);
    rt_kprintf("CHCFG    = 0x%08x\n", (unsigned int)I2S1_CHCFG);
    rt_kprintf("TX0CHSEL = 0x%08x\n", (unsigned int)I2S1_TX0CHSEL);
}

static void i2s1_lr_feed_entry(void *parameter)
{
    uint32_t phase = 0;
    uint32_t poll_count = 0;

    RT_UNUSED(parameter);

    while (s_lr_running)
    {
        uint32_t fsta = I2S1_FSTA;
        uint32_t free_words = (fsta >> 16) & 0xFFU;
        uint32_t n;

        if (free_words > I2S1_TXFIFO_DEPTH)
            free_words = I2S1_TXFIFO_DEPTH;

        s_lr_last_free_words = free_words;

        if (free_words == 0U)
        {
            s_lr_full_polls++;
            rt_thread_yield();
        }
        else
        {
            for (n = 0; n < free_words && s_lr_running; n++)
            {
                I2S1_TXFIFO = (phase == 0U) ? LR_LEFT_WORD : LR_RIGHT_WORD;
                phase ^= 1U;
                s_lr_words_written++;
            }
            s_lr_last_phase = phase;
        }

        poll_count++;
        if ((poll_count & 0x3FFFU) == 0U)
        {
            s_lr_last_txcnt = I2S1_TXCNT;
            rt_thread_yield();
        }
    }

    /*
     * Keep the handle non-NULL until the worker reaches this point.
     * stats/stop can then be used to verify that the worker has stopped.
     */
    s_lr_thread = RT_NULL;
}

static int i2s1_lr_start(void)
{
    rt_thread_t thread;
    rt_err_t err;
    uint32_t fsta;

    if (s_lr_running)
    {
        rt_kprintf("I2S1 LR pattern test is already running.\n");
        return 0;
    }

    if (s_lr_thread != RT_NULL)
    {
        rt_kprintf("Previous LR thread is still exiting; check stats again.\n");
        return -RT_ERROR;
    }

    /*
     * Restore controller configuration first. This fixes the prior failure
     * mode where CTL/FMT0/CLKD/CHCFG/TX0CHSEL were left at reset/other values.
     */
    i2s1_lr_restore_registers();

    /* Verify the key register writes stuck before starting the feeder. */
    if (I2S1_FMT0 != 0x00001F57U ||
        I2S1_FMT1 != 0x00000030U ||
        I2S1_CLKD != 0x00000010U ||
        I2S1_CHCFG != 0x00000001U ||
        I2S1_TX0CHSEL != 0x00110003U ||
        (I2S1_CTL & I2S1_TXEN_BIT) == 0U)
    {
        rt_kprintf("ERROR: I2S1 register configuration readback failed.\n");
        i2s1_lr_dump_registers();
        return -RT_ERROR;
    }

    s_lr_words_written = 0;
    s_lr_full_polls = 0;
    fsta = I2S1_FSTA;
    s_lr_last_free_words = (fsta >> 16) & 0xFFU;
    s_lr_last_phase = 0;
    s_lr_last_txcnt = I2S1_TXCNT;

    thread = rt_thread_create(
        "i2slrpat",
        i2s1_lr_feed_entry,
        RT_NULL,
        LR_STACK_SIZE,
        LR_PRIORITY,
        LR_TICK
    );

    if (thread == RT_NULL)
    {
        rt_kprintf("ERROR: failed to create LR pattern thread.\n");
        return -RT_ENOMEM;
    }

    s_lr_thread = thread;
    s_lr_running = RT_TRUE;

    err = rt_thread_startup(thread);
    if (err != RT_EOK)
    {
        s_lr_running = RT_FALSE;
        s_lr_thread = RT_NULL;
        rt_kprintf("ERROR: failed to start LR thread, err=%d.\n", err);
        return -RT_ERROR;
    }

    rt_kprintf("I2S1 LR pattern test started.\n");
    rt_kprintf("LEFT candidate  = 0x%08x\n", (unsigned int)LR_LEFT_WORD);
    rt_kprintf("RIGHT candidate = 0x%08x\n", (unsigned int)LR_RIGHT_WORD);
    rt_kprintf("I2S controller registers restored to tested configuration.\n");
    rt_kprintf("CCU clock and PG12/PG13/PG15 pinmux were NOT changed.\n");
    rt_kprintf("Do not run another I2S1 TX FIFO producer concurrently.\n");
    i2s1_lr_dump_registers();

    return 0;
}

static int i2s1_lr_stop(void)
{
    if (s_lr_thread == RT_NULL)
    {
        s_lr_running = RT_FALSE;
        rt_kprintf("LR pattern thread is not running.\n");
        return 0;
    }

    s_lr_running = RT_FALSE;
    rt_kprintf("Stop requested. Run stats until thread_handle=NULL.\n");
    rt_kprintf("The worker stops feeding; queued FIFO words may still drain.\n");
    return 0;
}

static int i2s1_lr_stats(void)
{
    uint32_t fsta = I2S1_FSTA;
    uint32_t ctl = I2S1_CTL;

    rt_kprintf("\n=== I2S1 LR pattern stats ===\n");
    rt_kprintf("running             = %u\n", (unsigned int)s_lr_running);
    rt_kprintf("thread_handle       = %s\n",
               (s_lr_thread != RT_NULL) ? "present" : "NULL");
    rt_kprintf("words_written       = %u (low 32 bits)\n",
               (unsigned int)s_lr_words_written);
    rt_kprintf("full_polls          = %u (low 32 bits)\n",
               (unsigned int)s_lr_full_polls);
    rt_kprintf("last_free_words     = %u\n",
               (unsigned int)s_lr_last_free_words);
    rt_kprintf("last_phase          = %u (0=next LEFT, 1=next RIGHT)\n",
               (unsigned int)s_lr_last_phase);
    rt_kprintf("CTL                 = 0x%08x\n", (unsigned int)ctl);
    rt_kprintf("TXEN                = %u\n",
               (unsigned int)((ctl & I2S1_TXEN_BIT) ? 1U : 0U));
    rt_kprintf("TXCNT               = 0x%08x\n", (unsigned int)I2S1_TXCNT);
    rt_kprintf("TXCNT last sample   = 0x%08x\n",
               (unsigned int)s_lr_last_txcnt);
    rt_kprintf("FSTA                = 0x%08x\n", (unsigned int)fsta);
    rt_kprintf("TXE_CNT             = %u words free\n",
               (unsigned int)((fsta >> 16) & 0xFFU));

    return 0;
}

static int i2s1_lr_pattern_test(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("  i2s1_lr_pattern_test start\n");
        rt_kprintf("  i2s1_lr_pattern_test stop\n");
        rt_kprintf("  i2s1_lr_pattern_test stats\n");
        rt_kprintf("  i2s1_lr_pattern_test dump\n");
        return 0;
    }

    if (rt_strcmp(argv[1], "start") == 0)
        return i2s1_lr_start();

    if (rt_strcmp(argv[1], "stop") == 0)
        return i2s1_lr_stop();

    if (rt_strcmp(argv[1], "stats") == 0)
        return i2s1_lr_stats();

    if (rt_strcmp(argv[1], "dump") == 0)
    {
        i2s1_lr_dump_registers();
        return 0;
    }

    rt_kprintf("Unknown argument: %s\n", argv[1]);
    return -RT_ERROR;
}

MSH_CMD_EXPORT_ALIAS(
    i2s1_lr_pattern_test,
    i2s1_lr_pattern_test,
    I2S1 stereo-slot pattern diagnostic
);
