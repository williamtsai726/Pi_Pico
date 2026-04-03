#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "parallel_bus.pio.h"

// Hardware Mapping
#define BUS_BASE_PIN 0    // GPIO 0-15
#define CLK_PIN      16   // Clock
#define DAC_EN_PIN   17   // DAC Enable
#define ADC_EN_PIN   18   // ADC Enable
#define BUF_SIZE     100000
#define ENABLE_TX    0

// Buffers and DMA Control
uint16_t sample_buffer[BUF_SIZE] __attribute__((aligned(4)));
static uint32_t buffer_ptr = (uint32_t)sample_buffer;
int data_chan, ctrl_chan;

typedef struct {
    PIO pio;
    uint sm;
    uint offset_tx;
    uint offset_rx;
} system_cfg_t;

// --- PIO Sampling Rate ---
void set_clock_freq_MHz(float target_Mhz) {
    uint32_t sys_clk = clock_get_hz(clk_sys);

    // 2 instructions per clock cycle
    float instr_per_cycle = 2.0f;

    float sm_freq = target_Mhz * 1e6f * instr_per_cycle;
    float div = (float)sys_clk / sm_freq;

    if (div < 1.0f) div = 1.0f;  // hardware limit

    pio_sm_set_clkdiv(pio0, 0, div);

    float actual_mhz = (float)sys_clk /
        (div * instr_per_cycle * 1e6f);

    printf("Clock: %.3f MHz (actual %.3f MHz, div %.3f)\n",
           target_Mhz, actual_mhz, div);
}

// --- DMA Loop Tx Setup ---
void setup_dma_tx(system_cfg_t *cfg) {
    data_chan = dma_claim_unused_channel(true);
    ctrl_chan = dma_claim_unused_channel(true);

    // Data Channel: RAM -> PIO
    dma_channel_config d_cfg = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&d_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&d_cfg, true);
    channel_config_set_write_increment(&d_cfg, false);
    channel_config_set_dreq(&d_cfg, pio_get_dreq(cfg->pio, cfg->sm, true));
    channel_config_set_chain_to(&d_cfg, ctrl_chan);
    channel_config_set_irq_quiet(&d_cfg, true);
    channel_config_set_high_priority(&d_cfg, true);
    channel_config_set_enable(&d_cfg, true);

    dma_channel_configure(data_chan, &d_cfg, &cfg->pio->txf[cfg->sm],
                          sample_buffer, BUF_SIZE, false);

    // Control Channel: Reset Data Channel Read Address
    dma_channel_config c_cfg = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&c_cfg, false);
    channel_config_set_write_increment(&c_cfg, false);
    channel_config_set_chain_to(&c_cfg, data_chan);
    channel_config_set_irq_quiet(&c_cfg, true);
    channel_config_set_high_priority(&c_cfg, true);
    channel_config_set_enable(&c_cfg, true);

    dma_channel_configure(ctrl_chan, &c_cfg, &dma_hw->ch[data_chan].read_addr,
                          &buffer_ptr, 1, false);
}

// --- DMA Loop Rx Setup ---
void setup_dma_rx(system_cfg_t *cfg) {
    data_chan = dma_claim_unused_channel(true);
    ctrl_chan = dma_claim_unused_channel(true);

    // Data Channel: PIO -> RAM
    dma_channel_config d_cfg = dma_channel_get_default_config(data_chan);
    channel_config_set_transfer_data_size(&d_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&d_cfg, false);
    channel_config_set_write_increment(&d_cfg, true);
    channel_config_set_dreq(&d_cfg, pio_get_dreq(cfg->pio, cfg->sm, false)); // RX
    channel_config_set_chain_to(&d_cfg, ctrl_chan);
    channel_config_set_irq_quiet(&d_cfg, true);
    channel_config_set_high_priority(&d_cfg, true);

    dma_channel_configure(
        data_chan, &d_cfg,
        sample_buffer,                 // write to RAM
        &cfg->pio->rxf[cfg->sm],        // read from PIO RX FIFO
        BUF_SIZE,
        false
    );

    // Control Channel: Reset Data Channel Read Address
    dma_channel_config c_cfg = dma_channel_get_default_config(ctrl_chan);
    channel_config_set_transfer_data_size(&c_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&c_cfg, false);
    channel_config_set_write_increment(&c_cfg, false);
    channel_config_set_chain_to(&c_cfg, data_chan);
    channel_config_set_irq_quiet(&c_cfg, true);
    channel_config_set_high_priority(&c_cfg, true);

    dma_channel_configure(
        ctrl_chan, &c_cfg,
        &dma_hw->ch[data_chan].write_addr,  // <-- key change
        &buffer_ptr,                        // buffer start
        1,
        false
    );
}

// --- Mode ---
void set_mode_tx(system_cfg_t *cfg) {
    dma_channel_abort(data_chan);
    dma_channel_abort(ctrl_chan);
    pio_sm_set_enabled(cfg->pio, cfg->sm, false);

    gpio_put(ADC_EN_PIN, 1); // inactive high
    gpio_put(DAC_EN_PIN, 1); // active high
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, true);

    pio_sm_config c = parallel_tx_program_get_default_config(cfg->offset_tx);
    sm_config_set_out_pins(&c, BUS_BASE_PIN, 16);
    sm_config_set_sideset_pins(&c, CLK_PIN);
    sm_config_set_out_shift(&c, true, true, 16);
    pio_sm_init(cfg->pio, cfg->sm, cfg->offset_tx, &c);

    pio_sm_clear_fifos(cfg->pio, cfg->sm);
    dma_start_channel_mask((1u << data_chan) | (1u << ctrl_chan));
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

void set_mode_rx(system_cfg_t *cfg) {
    dma_channel_abort(data_chan);
    dma_channel_abort(ctrl_chan);
    pio_sm_set_enabled(cfg->pio, cfg->sm, false);

    gpio_put(DAC_EN_PIN, 0); // inactive low
    gpio_put(ADC_EN_PIN, 0); // active low
    pio_sm_set_consecutive_pindirs(cfg->pio, cfg->sm, BUS_BASE_PIN, 16, false);

    pio_sm_config c = parallel_rx_program_get_default_config(cfg->offset_rx);
    sm_config_set_in_pins(&c, BUS_BASE_PIN);
    sm_config_set_sideset_pins(&c, CLK_PIN);
    sm_config_set_in_shift(&c, false, true, 16);
    pio_sm_init(cfg->pio, cfg->sm, cfg->offset_rx, &c);

    pio_sm_clear_fifos(cfg->pio, cfg->sm);
    dma_start_channel_mask((1u << data_chan) | (1u << ctrl_chan));
    pio_sm_set_enabled(cfg->pio, cfg->sm, true);
}

int main() {
    stdio_init_all();

    // Fill buffer with a ramp for initial validation
    for(int i=0; i<BUF_SIZE; i += 2) {
        sample_buffer[i] = 0x5555;
        sample_buffer[i + 1] = 0xAAAA;
    }
    system_cfg_t cfg = {pio0, 0};

    #if ENABLE_TX
        cfg.offset_tx = pio_add_program(cfg.pio, &parallel_tx_program);
    #else
        cfg.offset_rx = pio_add_program(cfg.pio, &parallel_rx_program);
    #endif

    // Initialization for the 16 bits bus
    for(int i=BUS_BASE_PIN; i < BUS_BASE_PIN + 16; i++) {
        pio_gpio_init(cfg.pio, i);
    }

    // Initialize the Clock pin
    pio_gpio_init(cfg.pio, CLK_PIN);

    gpio_init(DAC_EN_PIN); gpio_set_dir(DAC_EN_PIN, GPIO_OUT);
    gpio_init(ADC_EN_PIN); gpio_set_dir(ADC_EN_PIN, GPIO_OUT);
    pio_sm_set_consecutive_pindirs(cfg.pio, cfg.sm, CLK_PIN, 1, true);

    #if ENABLE_TX
        setup_dma_tx(&cfg);
        set_mode_tx(&cfg);
    #else
        setup_dma_rx(&cfg);
        set_mode_rx(&cfg);
    #endif

    set_clock_freq_MHz(10.0f);

    while (true) {
        // tight_loop_contents();
        printf("0x0000\n");
        // printf("\n--- Memory Snapshot (First 64 Samples) ---\n");
        // for (int i = 0; i < BUF_SIZE; i++) {
        //     // Print as Hex for bit-level debugging and Decimal for ADC scaling
        //     printf("Sample [%03d] @ %p: 0x%04X (%u)\n",
        //            i, &sample_buffer[i], sample_buffer[i], sample_buffer[i]);

        //     if ((i + 1) % 8 == 0) printf("\n");
        // }
        // printf("--- End of Dump ---\n");

        // #if !ENABLE_TX
        //     for (int i = 0; i < 100; i++) {
        //         printf("%u\n", sample_buffer[i]);
        //     }
        // #endif

        // int cmd = getchar_timeout_us(0);
    }
}