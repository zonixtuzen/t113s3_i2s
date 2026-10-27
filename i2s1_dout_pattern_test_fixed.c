/*
 * i2s1_dout_pattern_test_fixed.c
 *
 * Diagnostic TX FIFO feeder for Allwinner T113-S3 I2S1.
 *
 * Commands:
 *   i2s1_dout_pattern_test start
 *   i2s1_dout_pattern_test stop
 *   i2s1_dout_pattern_test dump
 *   i2s1_dout_pattern_test stats
 *
 * IMPORTANT:
 * - Register values are inherited from the user's previous test setup.
 * - This is diagnostic code, not a production audio driver.
 * - It does not modify CCU clock registers or pin mux.
 * - Confirm the T113-S3 manual's CTL/FCTL/FSTA definitions before using
 *   this with other configurations.
 * - Only this thread should write I2S1 TXFIFO while the test is running.
 */

#include <rtthread.h>
#include <stdint.h>

#define I2S1_BASE       0x02033000UL
#define I2S_CTL         (*(volatile uint32_t *)(I2S1_BASE + 0x00))
#define I2S_FMT0        (*(volatile uint32_t *)(I2S1_BASE + 0x04))
#define I2S_FMT1        (*(volatile uint32_t *)(I2S1_BASE + 0x08))
#define I2S_ISTA        (*(volatile uint32_t *)(I2S1_BASE + 0x0C))
#define I2S_FCTL        (*(volatile uint32_t *)(I2S1_BASE + 0x14))
#define I2S_FSTA        (*(volatile uint32_t *)(I2S1_BASE + 0x18))
#define I2S_INT         (*(volatile uint32_t *)(I2S1_BASE + 0x1C))
#define I2S_TXFIFO      (*(volatile uint32_t *)(I2S1_BASE + 0x20))
#define I2S_CLKD        (*(volatile uint32_t *)(I2S1_BASE + 0x24))
#define I2S_TXCNT       (*(volatile uint32_t *)(I2S1_BASE + 0x28))
#define I2S_CHCFG       (*(volatile uint32_t *)(I2S1_BASE + 0x30))
#define I2S_TX0CHSEL    (*(volatile uint32_t *)(I2S1_BASE + 0x34))

#define I2S_TXE_CNT(v)  (((v) >> 16) & 0xFFU)
#define I2S_TXFIFO_DEPTH 128U
#define FEED_STACK_SIZE 2048
#define FEED_PRIORITY   (RT_THREAD_PRIORITY_MAX / 2)
#define FEED_TICK       10

static rt_thread_t s_feed_thread = RT_NULL;
static volatile rt_uint8_t s_running = 0;

/* Updated by feeder; read by "stats". Use 32-bit counters for portability. */
static volatile rt_uint32_t s_words_written = 0;
static volatile rt_uint32_t s_fifo_full_polls = 0;
static volatile rt_uint32_t s_fifo_empty_observations = 0;
static volatile rt_uint32_t s_last_free_words = I2S_TXFIFO_DEPTH;
static volatile rt_uint32_t s_pattern_phase = 0;
static volatile rt_uint32_t s_last_tx_count = 0;

/*
 * Four blocks, each 256 FIFO words:
 * 0: alternating 0x55555555 / 0xAAAAAAAA
 * 1: all zeros
 * 2: all ones
 * 3: alternating 0x55555555 / 0xAAAAAAAA
 *
 * This pattern is for visible pin activity only; it does not yet validate
 * 24-bit sample alignment or left/right channel mapping.
 */
static uint32_t i2s1_next_pattern_word(uint32_t index)
{
    switch ((index / 256U) % 4U)
    {
    case 0:
    case 3:
        return (index & 1U) ? 0xAAAAAAAAU : 0x55555555U;
    case 1:
        return 0x00000000U;
    default:
        return 0xFFFFFFFFU;
    }
}

static void i2s1_dout_dump(void)
{
    uint32_t fsta = I2S_FSTA;
    rt_kprintf("\n=== I2S1 DOUT test register dump ===\n");
    rt_kprintf("CTL      = 0x%08x\n", I2S_CTL);
    rt_kprintf("FMT0     = 0x%08x\n", I2S_FMT0);
    rt_kprintf("FMT1     = 0x%08x\n", I2S_FMT1);
    rt_kprintf("ISTA     = 0x%08x (raw; decode using T113-S3 manual)\n", I2S_ISTA);
    rt_kprintf("FCTL     = 0x%08x\n", I2S_FCTL);
    rt_kprintf("FSTA     = 0x%08x (TXE_CNT=%u words free)\n",
                fsta, I2S_TXE_CNT(fsta));
    rt_kprintf("INT      = 0x%08x\n", I2S_INT);
    rt_kprintf("CLKD     = 0x%08x\n", I2S_CLKD);
    rt_kprintf("TXCNT    = 0x%08x\n", I2S_TXCNT);
    rt_kprintf("CHCFG    = 0x%08x\n", I2S_CHCFG);
    rt_kprintf("TX0CHSEL = 0x%08x\n", I2S_TX0CHSEL);
}

static void i2s1_dout_stats(void)
{
    rt_kprintf("\n=== I2S1 DOUT feeder stats ===\n");
    rt_kprintf("running                 = %u\n", (unsigned int)s_running);
    rt_kprintf("words_written (low32)   = %u\n", (unsigned int)s_words_written);
    rt_kprintf("fifo_full_polls (low32) = %u\n", (unsigned int)s_fifo_full_polls);
    rt_kprintf("fifo_empty_obs (low32)  = %u\n", (unsigned int)s_fifo_empty_observations);
    rt_kprintf("last_free_words         = %u\n", (unsigned int)s_last_free_words);
    rt_kprintf("pattern_phase           = %u\n", (unsigned int)s_pattern_phase);
    rt_kprintf("TXCNT now               = 0x%08x\n", I2S_TXCNT);
    rt_kprintf("TXCNT at last sample    = 0x%08x\n", (unsigned int)s_last_tx_count);
    rt_kprintf("FSTA now                = 0x%08x\n", I2S_FSTA);
}

static void i2s1_dout_feed_entry(void *parameter)
{
    uint32_t index = 0;
    uint32_t poll_count = 0;
    RT_UNUSED(parameter);

    rt_kprintf("I2S1 DOUT feeder thread running.\n");
    rt_kprintf("Use 'i2s1_dout_pattern_test stats' to inspect counters.\n");

    while (s_running)
    {
        uint32_t fsta = I2S_FSTA;
        uint32_t free_words = I2S_TXE_CNT(fsta);
        uint32_t n;

        s_last_free_words = free_words;

        if (free_words == 0U)
        {
            s_fifo_full_polls++;
            /*
             * Do not sleep for 1 ms: at 384 kHz stereo, a 128-word FIFO
             * can drain in about 167 us. Yield briefly when full.
             */
            rt_thread_yield();
        }
        else
        {
            /*
             * Write up to the number of currently free FIFO words.
             * This assumes this diagnostic thread is the only TXFIFO writer.
             */
            if (free_words > I2S_TXFIFO_DEPTH)
                free_words = I2S_TXFIFO_DEPTH;

            for (n = 0; n < free_words && s_running; n++)
            {
                I2S_TXFIFO = i2s1_next_pattern_word(index);
                index++;
                s_words_written++;
            }

            s_pattern_phase = (index / 256U) % 4U;
        }

        /*
         * Low-frequency sampling only. No console printing in the hot loop.
         * This counter is a diagnostic observation, not proof of underrun.
         */
        poll_count++;
        if ((poll_count & 0x3FFFU) == 0U)
        {
            uint32_t now_fsta = I2S_FSTA;
            s_last_free_words = I2S_TXE_CNT(now_fsta);
            if (I2S_TXE_CNT(now_fsta) == I2S_TXFIFO_DEPTH)
                s_fifo_empty_observations++;
            s_last_tx_count = I2S_TXCNT;
            rt_thread_yield();
        }
    }

    rt_kprintf("I2S1 DOUT feeder stopped.\n");
    s_feed_thread = RT_NULL;
}

static int i2s1_dout_start(void)
{
    rt_err_t err;

    if (s_running)
    {
        rt_kprintf("DOUT pattern test is already running.\n");
        return 0;
    }

    if (s_feed_thread != RT_NULL)
    {
        rt_kprintf("Previous feeder thread is still exiting; retry shortly.\n");
        return -RT_ERROR;
    }

    /* Reset diagnostic counters. Do not touch CCU or pinmux registers. */
    s_words_written = 0;
    s_fifo_full_polls = 0;
    s_fifo_empty_observations = 0;
    s_last_free_words = I2S_TXFIFO_DEPTH;
    s_pattern_phase = 0;
    s_last_tx_count = I2S_TXCNT;

    /*
     * Keep the same register values used in the user's earlier test.
     * These values are not independently validated by this diagnostic.
     */
    I2S_CTL      = 0x00060115;
    I2S_FMT0     = 0x00001F57;
    I2S_FMT1     = 0x00000030;
    I2S_FCTL     = 0x000500F0;
    I2S_INT      = 0x00000000;
    I2S_CLKD     = 0x00000010;
    I2S_CHCFG    = 0x00000001;
    I2S_TX0CHSEL = 0x00110003;

    s_running = 1;
    s_feed_thread = rt_thread_create("i2s1pat",
                                     i2s1_dout_feed_entry,
                                     RT_NULL,
                                     FEED_STACK_SIZE,
                                     FEED_PRIORITY,
                                     FEED_TICK);
    if (s_feed_thread == RT_NULL)
    {
        s_running = 0;
        rt_kprintf("ERROR: failed to create feeder thread.\n");
        return -RT_ENOMEM;
    }

    err = rt_thread_startup(s_feed_thread);
    if (err != RT_EOK)
    {
        s_running = 0;
        s_feed_thread = RT_NULL;
        rt_kprintf("ERROR: failed to start feeder thread, err=%d.\n", err);
        return -RT_ERROR;
    }

    rt_kprintf("Configured I2S1 TX for current 384 kHz diagnostic setup.\n");
    rt_kprintf("CCU clock registers and pinmux were not changed.\n");
    rt_kprintf("Do not run another I2S TX test at the same time.\n");
    return 0;
}

static int i2s1_dout_stop(void)
{
    s_running = 0;
    rt_kprintf("Stop requested. Wait for 'feeder stopped' before restarting.\n");
    return 0;
}

static int i2s1_dout_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage:\n");
        rt_kprintf("  i2s1_dout_pattern_test start\n");
        rt_kprintf("  i2s1_dout_pattern_test stop\n");
        rt_kprintf("  i2s1_dout_pattern_test dump\n");
        rt_kprintf("  i2s1_dout_pattern_test stats\n");
        return 0;
    }

    if (!rt_strcmp(argv[1], "start"))
        return i2s1_dout_start();
    if (!rt_strcmp(argv[1], "stop"))
        return i2s1_dout_stop();
    if (!rt_strcmp(argv[1], "dump"))
    {
        i2s1_dout_dump();
        return 0;
    }
    if (!rt_strcmp(argv[1], "stats"))
    {
        i2s1_dout_stats();
        return 0;
    }

    rt_kprintf("Unknown argument: %s\n", argv[1]);
    return -RT_ERROR;
}

MSH_CMD_EXPORT_ALIAS(i2s1_dout_cmd, i2s1_dout_pattern_test,
                     I2S1 DOUT0 fixed-pattern diagnostic);
