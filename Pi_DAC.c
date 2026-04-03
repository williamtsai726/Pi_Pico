#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "parallel_bus.pio.h"

#define BUS_BASE_PIN 0
#define CLK_PIN      16
#define DAC_EN_PIN   17
#define ADC_EN_PIN   18
#define MAX_BUF_SIZE 100000

// -------------------- Buffers --------------------
uint16_t sample_buffer[MAX_BUF_SIZE] __attribute__((aligned(4)));

// This variable must hold the starting address of the buffer
static uint32_t buffer_base_addr = (uint32_t)sample_buffer;
volatile int current_user_buf_size = MAX_BUF_SIZE;

int data_chan, ctrl_chan;
double stored_f_hz = 1000000.0;

typedef struct {
    PIO pio;
    uint sm;
    uint offset_tx;
    uint offset_rx;
} system_cfg_t;

// ============================================================
// Clock Logic: Optimized for 2 instructions per cycle
// ============================================================
float set_clock_freq_hz(system_cfg_t *cfg, double target_hz) {
    uint32_t sys_clk = clock_get_hz(clk_sys);

    // Exactly 2 instructions = 1 Period (1 High + 1 Low)
    float instr_per_cycle = 2.0f;
    float div = (float)sys_clk / ((float)target_hz * instr_per_cycle);

    if (div < 1.0f) div = 1.0f;
    pio_sm_set_clkdiv(cfg->pio, cfg->sm, div);

    return (float)sys_clk / (div * instr_per_cycle);
}

void stop_system(system_cfg_t *cfg) {
    dma_channel_abort(data_chan);
    dma_channel_abort(ctrl_chan);
    pio_sm_set_enabled(cfg->pio, cfg->sm, false);
    pio_sm_clear_fifos(cfg->pio, cfg->sm);

    // Return pins to safe state
    gpio_put(ADC_EN_PIN, 1);
    gpio_put(DAC_EN_PIN, 0);
}

// ============================================================
// TX Continuous (Looping DMA)
// ============================================================
void setup_tx_system(system_cfg_t *cfg, int size) {
    stop_system(cfg);

    gpio_put(ADC_EN_PIN, 1);
    gpio_put(DAC_EN_PIN, 1);

    // Ensure PIO owns the pins
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, true);
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, CLK_PIN, 1, true);

    pio_sm_config c = parallel_tx_program_get_default_config(cfg->offset_tx);
    sm_config_set_out_pins(&c, BUS_BASE_PIN, 16);
    sm_config_set_sideset_pins(&c, CLK_PIN);

    // Shift out 16 bits, autopull enabled (critical for continuous DMA)
    sm_config_set_out_shift(&c, true, true, 16);

    pio_sm_init(cfg->pio, cfg->sm, cfg->offset_tx, &c);
    set_clock_freq_hz(cfg, stored_f_hz);

    // DATA CHAN: RAM -> PIO (Fix: Read from buffer, Write to TX FIFO)
    dma_channel_config d_cfg = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&d_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&d_cfg, true);
    channel_config_set_write_increment(&d_cfg, false);
    channel_config_set_dreq(&d_cfg, pio_get_dreq(cfg->pio, cfg->sm, true));
    channel_config_set_chain_to(&d_cfg, ctrl_chan);

    dma_channel_configure(
        data_chan, &d_cfg,
        &cfg->pio->txf[cfg->sm], // Destination: PIO TX FIFO
        sample_buffer,           // Source: RAM
        size,
        false                    // Don't start yet
    );

    // CTRL CHAN: Resets the Read Address of Data Channel
    dma_channel_config c_cfg = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&c_cfg, false);
    channel_config_set_write_increment(&c_cfg, false);
    channel_config_set_chain_to(&c_cfg, data_chan);

    dma_channel_configure(
        ctrl_chan, &c_cfg,
        &dma_hw->ch[data_chan].read_addr, // Write to DMA register
        &buffer_base_addr,                // Read from variable holding the address
        1,
        false
    );
}

void enable_tx_continuous(system_cfg_t *cfg) {
    // Start both channels. Control channel will immediately trigger Data channel.
    dma_start_channel_mask((1u << data_chan) | (1u << ctrl_chan));
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

// ============================================================
// RX Single Capture
// ============================================================
void setup_rx_once(system_cfg_t *cfg, int size) {
    stop_system(cfg);
    gpio_put(DAC_EN_PIN, 0);
    gpio_put(ADC_EN_PIN, 0);

    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, false);
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, CLK_PIN, 1, true);

    pio_sm_config c = parallel_rx_program_get_default_config(cfg->offset_rx);
    sm_config_set_in_pins(&c, BUS_BASE_PIN);
    sm_config_set_sideset_pins(&c, CLK_PIN);
    sm_config_set_in_shift(&c, false, true, 16); // Autopush enabled

    pio_sm_init(cfg->pio, cfg->sm, cfg->offset_rx, &c);
    set_clock_freq_hz(cfg, stored_f_hz);
}

void capture_samples(system_cfg_t *cfg, int num_samples) {
    dma_channel_config d_cfg = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&d_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&d_cfg, false);
    channel_config_set_write_increment(&d_cfg, true);
    channel_config_set_dreq(&d_cfg, pio_get_dreq(cfg->pio, cfg->sm, false));

    dma_channel_configure(
        data_chan, &d_cfg,
        sample_buffer,            // Destination: RAM
        &cfg->pio->rxf[cfg->sm],  // Source: PIO RX FIFO
        num_samples,
        true                      // Start immediately
    );

    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
    dma_channel_wait_for_finish_blocking(data_chan);
    pio_sm_set_enabled(cfg->pio, cfg->sm, false);
}

int main() {
    stdio_init_all();
    system_cfg_t cfg = {pio0, 0};

    // Global Pin Init
    for(int i = 0; i < 16; i++) pio_gpio_init(cfg.pio, i);
    pio_gpio_init(cfg.pio, CLK_PIN);

    gpio_init(DAC_EN_PIN); gpio_set_dir(DAC_EN_PIN, GPIO_OUT);
    gpio_init(ADC_EN_PIN); gpio_set_dir(ADC_EN_PIN, GPIO_OUT);

    cfg.offset_tx = pio_add_program(cfg.pio, &parallel_tx_program);
    cfg.offset_rx = pio_add_program(cfg.pio, &parallel_rx_program);

    data_chan = dma_claim_unused_channel(true);
    ctrl_chan = dma_claim_unused_channel(true);

    while (true) {
        int c = getchar_timeout_us(100);
        if (c == -1) continue;

        if (c == 'I') {
            int tx, rx, sz;
            if (scanf(" %lf %d %d %d", &stored_f_hz, &tx, &rx, &sz) == 4) {
                current_user_buf_size = (sz > MAX_BUF_SIZE) ? MAX_BUF_SIZE : sz;
                if (tx) setup_tx_system(&cfg, current_user_buf_size);
                else if (rx) setup_rx_once(&cfg, current_user_buf_size);
                printf("OK %lf %d\n", stored_f_hz, current_user_buf_size);
            }
        }
        else if (c == 'P') {
            int n;
            if (scanf(" %d", &n) == 1) {
                for(int i = 0; i < n; i++) {
                    uint32_t val;
                    scanf(" %4x", &val);
                    sample_buffer[i] = (uint16_t)val;
                }
                enable_tx_continuous(&cfg);
                printf("OK\n");
            }
        }
        else if (c == 'G') {
            int n;
            if (scanf(" %d", &n) == 1) {
                capture_samples(&cfg, n);
                printf("DATA:");
                for (int i = 0; i < n; i++) printf("%04x", sample_buffer[i]);
                printf("\n");
            }
        }
        else if (c == 'X') {
            stop_system(&cfg);
            printf("RESET\n");
        }
    }
}