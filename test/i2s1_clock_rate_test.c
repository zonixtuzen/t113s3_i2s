/*
 * T113-S3 / RT-Thread: I2S1 clock-source/divider test for 384 kHz.
 * File: applications/i2s1_clock_rate_test.c
 *
 * MSH:
 *   i2s1_clock_rate_test dump
 *   i2s1_clock_rate_test 384
 *   i2s1_clock_rate_test off
 *
 * Clock-only test. Does NOT configure I2S format, pinmux, FIFO or DMA.
 *
 * Assumption from T113-S3 clock definitions:
 * PLL_AUDIO1 DIV5 = 614.4 MHz for PLL_AUDIO1_CTRL 0xF8417F00.
 * Source=PLL_AUDIO1_DIV5, N=/1, M=/25 -> 24.576 MHz module clock.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <string.h>

#define T113_CCU_BASE            0x02001000UL
#define REG_PLL_AUDIO1_CTRL      (T113_CCU_BASE + 0x080)
#define REG_I2S1_CLK             (T113_CCU_BASE + 0xA14)
#define REG_I2S_BGR              (T113_CCU_BASE + 0xA20)

#define I2S1_BUS_GATE            (1UL << 1)
#define I2S1_RESET_DEASSERT      (1UL << 17)
#define I2S1_MOD_CLK_GATE        (1UL << 31)

#define I2S1_CLK_SRC_MASK        (7UL << 24)
#define I2S1_CLK_SRC_AUDIO1_DIV5 (3UL << 24)
#define I2S1_CLK_N_MASK          (3UL << 8)
#define I2S1_CLK_M_MASK          (0x1FUL)
#define I2S1_CLK_M_DIV25         24UL /* encoded divisor = field + 1 */

static volatile uint32_t *i2s1_reg(uintptr_t addr)
{
    return (volatile uint32_t *)addr;
}

static uint32_t i2s1_read(uintptr_t addr)
{
    return *i2s1_reg(addr);
}

static void i2s1_write(uintptr_t addr, uint32_t value)
{
    *i2s1_reg(addr) = value;
    (void)*i2s1_reg(addr);
}

static void i2s1_clock_dump(void)
{
    rt_kprintf("=== I2S1 clock test ===\n");
    rt_kprintf("PLL_AUDIO1_CTRL = 0x%08x\n",
               (unsigned)i2s1_read(REG_PLL_AUDIO1_CTRL));
    rt_kprintf("I2S1_CLK_REG    = 0x%08x\n",
               (unsigned)i2s1_read(REG_I2S1_CLK));
    rt_kprintf("I2S_BGR_REG     = 0x%08x\n",
               (unsigned)i2s1_read(REG_I2S_BGR));
}

static void i2s1_clock_384(void)
{
    uint32_t v;

    /* Enable bus clock and release reset, preserving other BGR bits. */
    v = i2s1_read(REG_I2S_BGR);
    v |= I2S1_BUS_GATE;
    i2s1_write(REG_I2S_BGR, v);

    v = i2s1_read(REG_I2S_BGR);
    v |= I2S1_RESET_DEASSERT;
    i2s1_write(REG_I2S_BGR, v);

    /*
     * Set source=PLL_AUDIO1 DIV5, N=/1, M=/25, then enable module clock.
     * Only documented source/divider/gate fields are modified.
     */
    v = i2s1_read(REG_I2S1_CLK);
    v &= ~(I2S1_CLK_SRC_MASK | I2S1_CLK_N_MASK | I2S1_CLK_M_MASK);
    v |= I2S1_CLK_SRC_AUDIO1_DIV5 | I2S1_CLK_M_DIV25 | I2S1_MOD_CLK_GATE;
    i2s1_write(REG_I2S1_CLK, v);

    rt_thread_mdelay(1);
    rt_kprintf("Requested I2S1 module clock: 24.576 MHz.\n");
    rt_kprintf("Expected source=PLL_AUDIO1_DIV5, N=/1, M=/25.\n");
    i2s1_clock_dump();
}

static void i2s1_clock_off(void)
{
    uint32_t v;

    v = i2s1_read(REG_I2S1_CLK);
    v &= ~I2S1_MOD_CLK_GATE;
    i2s1_write(REG_I2S1_CLK, v);

    v = i2s1_read(REG_I2S_BGR);
    v &= ~I2S1_RESET_DEASSERT;
    i2s1_write(REG_I2S_BGR, v);

    v = i2s1_read(REG_I2S_BGR);
    v &= ~I2S1_BUS_GATE;
    i2s1_write(REG_I2S_BGR, v);

    rt_kprintf("I2S1 module clock gated off; reset asserted.\n");
    i2s1_clock_dump();
}

static int i2s1_clock_rate_test(int argc, char **argv)
{
    if (argc < 2 || !strcmp(argv[1], "dump")) {
        i2s1_clock_dump();
        return 0;
    }
    if (!strcmp(argv[1], "384")) {
        i2s1_clock_384();
        return 0;
    }
    if (!strcmp(argv[1], "off")) {
        i2s1_clock_off();
        return 0;
    }

    rt_kprintf("Usage: i2s1_clock_rate_test [dump|384|off]\n");
    return -RT_EINVAL;
}
MSH_CMD_EXPORT(i2s1_clock_rate_test, T113-S3 I2S1 clock source/divider test);