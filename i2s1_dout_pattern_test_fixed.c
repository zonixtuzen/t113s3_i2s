/*
 * i2s1_dout_pattern_test_fixed.c
 *
 * Allwinner T113-S3 I2S1 DOUT pattern test and TXCNT observer.
 *
 * MSH commands:
 *   i2s1_dout_pattern_test start
 *   i2s1_dout_pattern_test stop
 *   i2s1_dout_pattern_test stats
 *   i2s1_dout_pattern_test dump
 *   i2s1_dout_pattern_test observe
 *
 * Fixes:
 *   1. Feeder thread never calls rt_kprintf().
 *   2. Stop requests feeder exit; restart is rejected until exit completes.
 *   3. "observe" samples TXCNT/CTL/FSTA over 100 ms.
 *
 * WARNING:
 *   - Diagnostic code, not a production audio driver.
 *   - Keeps the existing I2S register configuration.
 *   - Does not modify CCU clock registers or pin mux.
 *   - Only one software/DMA producer should write I2S1 TXFIFO.
 */

#include <rtthread.h>
#include <stdint.h>

/* ------------------------------------------------------------ */
/* I2S1 registers                                                 */
/* ------------------------------------------------------------ */

#define I2S1_BASE           0x02033000UL

#define I2S_CTL             (*(volatile uint32_t *)(I2S1_BASE + 0x00))
#define I2S_FMT0            (*(volatile uint32_t *)(I2S1_BASE + 0x04))
#define I2S_FMT1            (*(volatile uint32_t *)(I2S1_BASE + 0x08))
#define I2S_ISTA            (*(volatile uint32_t *)(I2S1_BASE + 0x0C))
#define I2S_FCTL            (*(volatile uint32_t *)(I2S1_BASE + 0x14))
#define I2S_FSTA            (*(volatile uint32_t *)(I2S1_BASE + 0x18))
#define I2S_INT             (*(volatile uint32_t *)(I2S1_BASE + 0x1C))
#define I2S_TXFIFO          (*(volatile uint32_t *)(I2S1_BASE + 0x20))
#define I2S_CLKD            (*(volatile uint32_t *)(I2S1_BASE + 0x24))
#define I2S_TXCNT           (*(volatile uint32_t *)(I2S1_BASE + 0x28))
#define I2S_CHCFG           (*(volatile uint32_t *)(I2S1_BASE + 0x30))
#define I2S_TX0CHSEL        (*(volatile uint32_t *)(I2S1_BASE + 0x34))

#define I2S_TXE_CNT(v)      (((v) >> 16) & 0xFFU)

#define I2S_TXFIFO_DEPTH    128U

/* ------------------------------------------------------------ */
/* Feeder configuration                                          */
/* ------------------------------------------------------------ */

#define FEED_STACK_SIZE     2048
#define FEED_PRIORITY       (RT_THREAD_PRIORITY_MAX / 2)
#define FEED_TICK           10

#define OBSERVE_DELAY_MS    100

/* ------------------------------------------------------------ */
/* State                                                         */
/* ------------------------------------------------------------ */

/*
 * s_running:
 *   1 = feeder should continue
 *   0 = feeder should exit
 *
 * s_feed_thread:
 *   Non-NULL until the feeder has exited.
 *   A new feeder cannot start while this handle remains non-NULL.
 *
 * The feeder thread intentionally does not print to the console.
 */

static rt_thread_t s_feed_thread = RT_NULL;
static volatile rt_uint8_t s_running = 0;

static volatile rt_uint32_t s_words_written = 0;
static volatile rt_uint32_t s_fifo_full_polls = 0;
static volatile rt_uint32_t s_fifo_empty_observations = 0;
static volatile rt_uint32_t s_last_free_words = I2S_TXFIFO_DEPTH;
static volatile rt_uint32_t s_pattern_phase = 0;
static volatile rt_uint32_t s_last_tx_count = 0;

/* ------------------------------------------------------------ */
/* Test pattern                                                   */
/* ------------------------------------------------------------ */

/*
 * Four blocks, each 256 FIFO words:
 *
 *   Block 0: 0x55555555 / 0xAAAAAAAA alternating
 *   Block 1: all zeros
 *   Block 2: all ones
 *   Block 3: 0x55555555 / 0xAAAAAAAA alternating
 *
 * This verifies pin activity only.
 * It does not validate 24-bit alignment or channel mapping.
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

/* ------------------------------------------------------------ */
/* Register dump                                                  */
/* ------------------------------------------------------------ */

static void i2s1_dout_dump(void)
{
    uint32_t ctl;
    uint32_t fmt0;
    uint32_t fmt1;
    uint32_t ista;
    uint32_t fctl;
    uint32_t fsta;
    uint32_t int_reg;
    uint32_t clkd;
    uint32_t txcnt;
    uint32_t chcfg;
    uint32_t tx0chsel;

    /*
     * Snapshot each register once, reducing inconsistent output
     * if hardware state changes while printing.
     */

    ctl       = I2S_CTL;
    fmt0      = I2S_FMT0;
    fmt1      = I2S_FMT1;
    ista      = I2S_ISTA;
    fctl      = I2S_FCTL;
    fsta      = I2S_FSTA;
    int_reg   = I2S_INT;
    clkd      = I2S_CLKD;
    txcnt     = I2S_TXCNT;
    chcfg     = I2S_CHCFG;
    tx0chsel  = I2S_TX0CHSEL;

    rt_kprintf("\n=== I2S1 DOUT register dump ===\n");

    rt_kprintf("CTL      = 0x%08x\n", ctl);
    rt_kprintf("FMT0     = 0x%08x\n", fmt0);
    rt_kprintf("FMT1     = 0x%08x\n", fmt1);
    rt_kprintf("ISTA     = 0x%08x\n", ista);
    rt_kprintf("FCTL     = 0x%08x\n", fctl);

    rt_kprintf("FSTA     = 0x%08x\n", fsta);
    rt_kprintf("TXE_CNT  = %u words free\n",
               (unsigned int)I2S_TXE_CNT(fsta));

    rt_kprintf("INT      = 0x%08x\n", int_reg);
    rt_kprintf("CLKD     = 0x%08x\n", clkd);
    rt_kprintf("TXCNT    = 0x%08x\n", txcnt);
    rt_kprintf("CHCFG    = 0x%08x\n", chcfg);
    rt_kprintf("TX0CHSEL = 0x%08x\n", tx0chsel);
}

/* ------------------------------------------------------------ */
/* Software statistics                                            */
/* ------------------------------------------------------------ */

static void i2s1_dout_stats(void)
{
    uint32_t txcnt;
    uint32_t fsta;

    txcnt = I2S_TXCNT;
    fsta = I2S_FSTA;

    rt_kprintf("\n=== I2S1 DOUT feeder stats ===\n");

    rt_kprintf("running                 = %u\n",
               (unsigned int)s_running);

    rt_kprintf("thread_handle_present   = %u\n",
               (s_feed_thread != RT_NULL) ? 1U : 0U);

    rt_kprintf("words_written (low32)   = %u\n",
               (unsigned int)s_words_written);

    rt_kprintf("fifo_full_polls (low32) = %u\n",
               (unsigned int)s_fifo_full_polls);

    rt_kprintf("fifo_empty_obs (low32)  = %u\n",
               (unsigned int)s_fifo_empty_observations);

    rt_kprintf("last_free_words         = %u\n",
               (unsigned int)s_last_free_words);

    rt_kprintf("pattern_phase           = %u\n",
               (unsigned int)s_pattern_phase);

    rt_kprintf("TXCNT now               = 0x%08x\n",
               txcnt);

    rt_kprintf("TXCNT at last sample    = 0x%08x\n",
               (unsigned int)s_last_tx_count);

    rt_kprintf("FSTA now                = 0x%08x\n",
               fsta);

    rt_kprintf("TXE_CNT now             = %u words free\n",
               (unsigned int)I2S_TXE_CNT(fsta));
}

/* ------------------------------------------------------------ */
/* Independent TXCNT observer                                    */
/* ------------------------------------------------------------ */

/*
 * This command does not write any I2S registers.
 *
 * It samples CTL, FSTA and TXCNT, waits 100 ms, then samples again.
 *
 * TXCNT delta uses unsigned subtraction, so a single 32-bit wrap
 * is handled correctly.
 *
 * The result shows whether TXCNT changed during the interval.
 * It does not, by itself, identify the source of any change.
 */

static int i2s1_dout_observe(void)
{
    uint32_t ctl1;
    uint32_t ctl2;

    uint32_t fsta1;
    uint32_t fsta2;

    uint32_t txcnt1;
    uint32_t txcnt2;
    uint32_t delta;

    ctl1   = I2S_CTL;
    fsta1  = I2S_FSTA;
    txcnt1 = I2S_TXCNT;

    rt_thread_mdelay(OBSERVE_DELAY_MS);

    ctl2   = I2S_CTL;
    fsta2  = I2S_FSTA;
    txcnt2 = I2S_TXCNT;

    delta = (uint32_t)(txcnt2 - txcnt1);

    rt_kprintf("\n=== I2S1 TXCNT observer ===\n");

    rt_kprintf("Interval                = %u ms\n",
               (unsigned int)OBSERVE_DELAY_MS);

    rt_kprintf("Feeder running          = %u\n",
               (unsigned int)s_running);

    rt_kprintf("CTL before              = 0x%08x\n", ctl1);
    rt_kprintf("CTL after               = 0x%08x\n", ctl2);

    rt_kprintf("FSTA before             = 0x%08x\n", fsta1);
    rt_kprintf("FSTA after              = 0x%08x\n", fsta2);

    rt_kprintf("TXE_CNT before          = %u words free\n",
               (unsigned int)I2S_TXE_CNT(fsta1));

    rt_kprintf("TXE_CNT after           = %u words free\n",
               (unsigned int)I2S_TXE_CNT(fsta2));

    rt_kprintf("TXCNT before            = 0x%08x\n", txcnt1);
    rt_kprintf("TXCNT after             = 0x%08x\n", txcnt2);

    rt_kprintf("TXCNT delta             = %u\n",
               (unsigned int)delta);

    /*
     * This is only a rough rate estimate.
     * The actual elapsed time may differ slightly from 100 ms
     * because of RTOS scheduling.
     */

    rt_kprintf("Approx. delta per 100ms = %u\n",
               (unsigned int)delta);

    return 0;
}

/* ------------------------------------------------------------ */
/* Feeder thread                                                  */
/* ------------------------------------------------------------ */

static void i2s1_dout_feed_entry(void *parameter)
{
    uint32_t index = 0;
    uint32_t poll_count = 0;

    RT_UNUSED(parameter);

    /*
     * IMPORTANT:
     * No rt_kprintf() in this thread.
     *
     * Printing from the feeder concurrently with MSH command
     * output was a source of interleaved console text.
     */

    while (s_running)
    {
        uint32_t fsta;
        uint32_t free_words;
        uint32_t n;

        fsta = I2S_FSTA;
        free_words = I2S_TXE_CNT(fsta);

        s_last_free_words = free_words;

        if (free_words == 0U)
        {
            s_fifo_full_polls++;

            /*
             * Yield while FIFO is full.
             * Do not sleep for 1 ms here: at high sample rates,
             * the FIFO may drain much sooner.
             */

            rt_thread_yield();
        }
        else
        {
            if (free_words > I2S_TXFIFO_DEPTH)
            {
                free_words = I2S_TXFIFO_DEPTH;
            }

            for (n = 0;
                 n < free_words && s_running;
                 n++)
            {
                I2S_TXFIFO = i2s1_next_pattern_word(index);

                index++;
                s_words_written++;
            }

            s_pattern_phase = (index / 256U) % 4U;
        }

        /*
         * Periodically sample diagnostics.
         * No console output in the hot loop.
         */

        poll_count++;

        if ((poll_count & 0x3FFFU) == 0U)
        {
            uint32_t now_fsta;

            now_fsta = I2S_FSTA;
            s_last_free_words = I2S_TXE_CNT(now_fsta);

            if (I2S_TXE_CNT(now_fsta) == I2S_TXFIFO_DEPTH)
            {
                s_fifo_empty_observations++;
            }

            s_last_tx_count = I2S_TXCNT;

            rt_thread_yield();
        }
    }

    /*
     * Mark thread exit only after the loop has ended.
     * No printing here: the MSH thread owns console output.
     */

    s_feed_thread = RT_NULL;
}

/* ------------------------------------------------------------ */
/* Start                                                          */
/* ------------------------------------------------------------ */

static int i2s1_dout_start(void)
{
    rt_err_t err;
    rt_thread_t thread;

    if (s_running)
    {
        rt_kprintf("DOUT pattern test is already running.\n");
        return 0;
    }

    /*
     * Do not restart until the previous feeder has actually exited.
     */

    if (s_feed_thread != RT_NULL)
    {
        rt_kprintf("Previous feeder is still exiting.\n");
        rt_kprintf("Wait briefly, then check stats again.\n");
        return -RT_ERROR;
    }

    /* Reset software diagnostics. */

    s_words_written = 0;
    s_fifo_full_polls = 0;
    s_fifo_empty_observations = 0;

    s_last_free_words = I2S_TXFIFO_DEPTH;
    s_pattern_phase = 0;
    s_last_tx_count = I2S_TXCNT;

    /*
     * Preserve the register values used in the existing test.
     * This test does not change CCU clock registers or pin mux.
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

    thread = rt_thread_create(
        "i2s1pat",
        i2s1_dout_feed_entry,
        RT_NULL,
        FEED_STACK_SIZE,
        FEED_PRIORITY,
        FEED_TICK
    );

    if (thread == RT_NULL)
    {
        s_running = 0;

        rt_kprintf("ERROR: failed to create feeder thread.\n");
        return -RT_ENOMEM;
    }

    /*
     * Save the handle before startup, so the command cannot
     * accidentally create a second feeder.
     */

    s_feed_thread = thread;

    err = rt_thread_startup(thread);

    if (err != RT_EOK)
    {
        s_running = 0;
        s_feed_thread = RT_NULL;

        rt_kprintf("ERROR: failed to start feeder, err=%d.\n",
                   err);

        return -RT_ERROR;
    }

    rt_kprintf("I2S1 DOUT pattern feeder started.\n");
    rt_kprintf("Configuration: existing 384 kHz diagnostic setup.\n");
    rt_kprintf("CCU clock registers and pin mux were not changed.\n");
    rt_kprintf("Do not run another I2S TX test at the same time.\n");

    return 0;
}

/* ------------------------------------------------------------ */
/* Stop                                                           */
/* ------------------------------------------------------------ */

static int i2s1_dout_stop(void)
{
    if (s_feed_thread == RT_NULL)
    {
        s_running = 0;

        rt_kprintf("Feeder is not running.\n");
        rt_kprintf("I2S hardware registers were not changed.\n");

        return 0;
    }

    if (s_running)
    {
        s_running = 0;

        rt_kprintf("Stop requested.\n");
        rt_kprintf("Wait until thread_handle_present = 0 in stats.\n");
        rt_kprintf("I2S hardware registers were not changed.\n");
    }
    else
    {
        rt_kprintf("Feeder is already stopping.\n");
        rt_kprintf("Check stats before restarting.\n");
    }

    return 0;
}

/* ------------------------------------------------------------ */
/* MSH command                                                    */
/* ------------------------------------------------------------ */

static int i2s1_dout_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("\nUsage:\n");
        rt_kprintf("  i2s1_dout_pattern_test start\n");
        rt_kprintf("  i2s1_dout_pattern_test stop\n");
        rt_kprintf("  i2s1_dout_pattern_test stats\n");
        rt_kprintf("  i2s1_dout_pattern_test dump\n");
        rt_kprintf("  i2s1_dout_pattern_test observe\n");

        return 0;
    }

    if (!rt_strcmp(argv[1], "start"))
    {
        return i2s1_dout_start();
    }

    if (!rt_strcmp(argv[1], "stop"))
    {
        return i2s1_dout_stop();
    }

    if (!rt_strcmp(argv[1], "stats"))
    {
        i2s1_dout_stats();
        return 0;
    }

    if (!rt_strcmp(argv[1], "dump"))
    {
        i2s1_dout_dump();
        return 0;
    }

    if (!rt_strcmp(argv[1], "observe"))
    {
        return i2s1_dout_observe();
    }

    rt_kprintf("Unknown argument: %s\n", argv[1]);

    return -RT_ERROR;
}

MSH_CMD_EXPORT_ALIAS(
    i2s1_dout_cmd,
    i2s1_dout_pattern_test,
    I2S1 DOUT pattern test and TXCNT observer
);
