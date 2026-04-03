#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "parallel_bus.pio.h"
#include "hardware/watchdog.h"

#define BUS_BASE_PIN 0
#define CLK_PIN      16
#define DAC_EN_PIN   17
#define ADC_EN_PIN   18
#define MAX_BUF_SIZE 100000

// Global State
uint16_t sample_buffer[MAX_BUF_SIZE] __attribute__((aligned(4)));
static uint32_t buffer_ptr = (uint32_t)sample_buffer;
uint32_t current_user_buf_size = 0;

// Track channels globally so we can abort them on re-init
static int data_chan = -1;
static int ctrl_chan = -1;

typedef struct {
    PIO pio;
    uint sm;
    uint offset_tx;
    uint offset_rx;
} system_cfg_t;

// --- Safety: Stop any active DMA or PIO ---
void stop_all_activity(system_cfg_t *cfg) {
    if (data_chan != -1) dma_channel_abort(data_chan);
    if (ctrl_chan != -1) dma_channel_abort(ctrl_chan);

    pio_sm_set_enabled(cfg->pio, cfg->sm, false);
    pio_sm_clear_fifos(cfg->pio, cfg->sm);

    // Set bus to high-impedance (inputs) for safety
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, false);
    gpio_put(ADC_EN_PIN, 1); // Disable ADC
    gpio_put(DAC_EN_PIN, 0); // Disable DAC
}

float set_clock_freq_MHz(system_cfg_t *cfg, float target_Mhz) {
    uint32_t sys_clk = clock_get_hz(clk_sys);
    float div = (float)sys_clk / (target_Mhz * 1e6f * 2.0f);
    if (div < 1.0f) div = 1.0f;
    pio_sm_set_clkdiv(cfg->pio, cfg->sm, div);
    return (float)sys_clk / (div * 2.0f * 1e6f);
}

void init_dma_channels() {
    if (data_chan == -1) data_chan = dma_claim_unused_channel(true);
    if (ctrl_chan == -1) ctrl_chan = dma_claim_unused_channel(true);
}

void enable_tx_mode(system_cfg_t *cfg, uint32_t n) {
    gpio_put(ADC_EN_PIN, 1);
    gpio_put(DAC_EN_PIN, 1);
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, true);

    pio_sm_config c = parallel_tx_program_get_default_config(cfg->offset_tx);
    sm_config_set_out_pins(&c, BUS_BASE_PIN, 16);
    sm_config_set_sideset_pins(&c, CLK_PIN);
    sm_config_set_out_shift(&c, true, true, 16);
    pio_sm_init(cfg->pio, cfg->sm, cfg->offset_tx, &c);

    dma_channel_config d_cfg = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&d_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&d_cfg, true);
    channel_config_set_dreq(&d_cfg, pio_get_dreq(cfg->pio, cfg->sm, true));
    channel_config_set_chain_to(&d_cfg, ctrl_chan);
    dma_channel_configure(data_chan, &d_cfg, &cfg->pio->txf[cfg->sm], sample_buffer, n, false);

    dma_channel_config c_cfg = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c_cfg, DMA_SIZE_32);
    channel_config_set_chain_to(&c_cfg, data_chan);
    dma_channel_configure(ctrl_chan, &c_cfg, &dma_hw->ch[data_chan].read_addr, &buffer_ptr, 1, false);

    dma_start_channel_mask((1u << data_chan) | (1u << ctrl_chan));
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

void enable_rx_mode(system_cfg_t *cfg, uint32_t n) {
    // 1. Hardware Pin Setup
    gpio_put(DAC_EN_PIN, 0);
    gpio_put(ADC_EN_PIN, 0);
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, false);

    // 2. PIO Setup
    pio_sm_config c = parallel_rx_program_get_default_config(cfg->offset_rx);
    sm_config_set_in_pins(&c, BUS_BASE_PIN);
    sm_config_set_sideset_pins(&c, CLK_PIN);
    sm_config_set_in_shift(&c, false, true, 16);
    pio_sm_init(cfg->pio, cfg->sm, cfg->offset_rx, &c);

    // 3. Clear the buffer so we know what is new data
    memset(sample_buffer, 0, n * sizeof(uint16_t));

    // 4. Data Channel: One-Shot (No Chaining)
    dma_channel_config d_cfg = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&d_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&d_cfg, false);
    channel_config_set_write_increment(&d_cfg, true); // MUST BE TRUE
    channel_config_set_dreq(&d_cfg, pio_get_dreq(cfg->pio, cfg->sm, false));

    // Disable chaining for this test to ensure it doesn't loop back to 0
    channel_config_set_chain_to(&d_cfg, data_chan);

    dma_channel_configure(
        data_chan,
        &d_cfg,
        sample_buffer,           // Write Destination
        &cfg->pio->rxf[cfg->sm], // Read Source
        n,                       // Total samples to capture
        true                     // Start NOW
    );

    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

int main() {
    stdio_init_all();

    system_cfg_t cfg = {pio0, 0};

    // 1. Hardware Pin Setup (Do this ONCE)
    for(int i=BUS_BASE_PIN; i < BUS_BASE_PIN + 16; i++) {
        pio_gpio_init(cfg.pio, i);
    }
    pio_gpio_init(cfg.pio, CLK_PIN);

    gpio_init(DAC_EN_PIN); gpio_set_dir(DAC_EN_PIN, GPIO_OUT);
    gpio_init(ADC_EN_PIN); gpio_set_dir(ADC_EN_PIN, GPIO_OUT);

    // Initial safe state
    gpio_put(ADC_EN_PIN, 1);
    gpio_put(DAC_EN_PIN, 0);

    // 2. Load Programs
    cfg.offset_tx = pio_add_program(cfg.pio, &parallel_tx_program);
    cfg.offset_rx = pio_add_program(cfg.pio, &parallel_rx_program);

    init_dma_channels();

    while (true) {
        int c = getchar_timeout_us(10);
        if (c == 'I') {
            float f; int tx, rx, sz;
            // Use a specific format to consume the newline
            if (scanf("%f %d %d %d\n", &f, &tx, &rx, &sz) == 4) {
                stop_all_activity(&cfg);

                current_user_buf_size = (sz > MAX_BUF_SIZE) ? MAX_BUF_SIZE : sz;
                float actual = set_clock_freq_MHz(&cfg, f);

                if (tx) enable_tx_mode(&cfg, current_user_buf_size);
                else if (rx) enable_rx_mode(&cfg, current_user_buf_size);

                printf("OK %.3f %d\n", actual, current_user_buf_size);
                stdio_flush();
            }
        }
        else if (c == 'X') {
            // TERMINATE COMMAND
            printf("ACK_RESET\n");
            stdio_flush();
            sleep_ms(100); // Let the message get out

            // Trigger a hardware reboot
            // 1ms delay, 0 means "reboot now"
            watchdog_reboot(0, 0, 0);
        } else if (c == 'G') {
            int n; scanf("%d", &n);
            if (n > current_user_buf_size) n = current_user_buf_size;

            // WAIT for DMA to finish its one-shot run
            while (dma_channel_is_busy(data_chan)) {
                tight_loop_contents();
            }

            printf("DATA:");
            for(int i=0; i<n; i++) {
                printf("%04x", sample_buffer[i]);
            }
            printf("\n");
            stdio_flush();
        } else if (c == 'P') {
            int n; scanf("%d", &n);
            for(int i=0; i<n; i++) {
                uint32_t val; scanf("%04x", &val);
                sample_buffer[i] = (uint16_t)val;
            }
            printf("OK\n");
        } else if (c == 'L') {
            printf("%d\n", current_user_buf_size);
        }
    }
}


