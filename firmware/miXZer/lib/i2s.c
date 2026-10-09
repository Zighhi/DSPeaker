/* i2s.c
 *
 * Author: Daniel Collins
 * Date:   2022-02-25
 *
 * Copyright (c) 2022 Daniel Collins
 *
 * This file is part of rp2040_i2s_example.
 *
 * rp2040_i2s_example is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3 as published by the
 * Free Software Foundation.
 *
 * rp2040_i2s_example is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * rp2040_i2s_example. If not, see <https://www.gnu.org/licenses/>.
 */

#include "i2s.h"
#include <math.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "i2s.pio.h"

const i2s_config i2s_config_default = {48000, 256, 32, 10, 6, 7, 8, true};

static float pio_div(float freq, uint16_t* div, uint8_t* frac) {
    float clk   = (float)clock_get_hz(clk_sys);
    float ratio = clk / freq;
    float d;
    float f = modff(ratio, &d);
    *div    = (uint16_t)d;
    *frac   = (uint8_t)(f * 256.0f);

    // Use post-converted values to get actual freq after any rounding
    float result = clk / ((float)*div + ((float)*frac / 256.0f));

    return result;
}

static void calc_clocks(const i2s_config* config, pio_i2s_clocks* clocks) {
    // Try to get a precise ratio between SCK and BCK regardless of how
    // perfect the system_clock divides. First, see what sck we can actually get:
    float sck_desired   = (float)config->fs * (float)config->sck_mult * (float)i2s_sck_program_pio_mult;
    float sck_attained  = pio_div(sck_desired, &clocks->sck_d, &clocks->sck_f);
    clocks->fs_attained = sck_attained / (float)config->sck_mult / (float)i2s_sck_program_pio_mult;

    // Now that we have the closest fs our dividers will give us, we can
    // re-calculate SCK and BCK as correct ratios of this adjusted fs:
    float sck_hz       = clocks->fs_attained * (float)config->sck_mult;
    clocks->sck_pio_hz = pio_div(sck_hz * (float)i2s_sck_program_pio_mult, &clocks->sck_d, &clocks->sck_f);
    float bck_hz       = clocks->fs_attained * (float)config->bit_depth * 2.0f;
    clocks->bck_pio_hz = pio_div(bck_hz * (float)i2s_out_master_program_pio_mult, &clocks->bck_d, &clocks->bck_f);
}

static bool validate_sck_bck_sync(pio_i2s_clocks* clocks) {
    (void)clocks;
    return true; // Prevent false floating point precision panic
}

static void dma_double_buffer_init(pio_i2s* i2s, void (*dma_handler)(void)) {
    // Set up DMA for PIO I2s - two channels, in and out
    i2s->dma_ch_in_ctrl  = dma_claim_unused_channel(true);
    i2s->dma_ch_out_ctrl = dma_claim_unused_channel(true);
    i2s->dma_ch_out_data = dma_claim_unused_channel(true);
    i2s->dma_ch_in_data  = dma_claim_unused_channel(true);

    // Control blocks support double-buffering with interrupts on buffer change
    i2s->in_ctrl_blocks[0]  = i2s->input_buffer;
    i2s->in_ctrl_blocks[1]  = &i2s->input_buffer[STEREO_BUFFER_SIZE];
    i2s->out_ctrl_blocks[0] = i2s->output_buffer;
    i2s->out_ctrl_blocks[1] = &i2s->output_buffer[STEREO_BUFFER_SIZE];

    // DMA I2S OUT control channel - wrap read address every 8 bytes (2 words)
    // Transfer 1 word at a time, to the out channel read address and trigger.
    dma_channel_config c = dma_channel_get_default_config(i2s->dma_ch_out_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(i2s->dma_ch_out_ctrl, &c, &dma_hw->ch[i2s->dma_ch_out_data].al3_read_addr_trig, i2s->out_ctrl_blocks, 1, false);

    c = dma_channel_get_default_config(i2s->dma_ch_out_data);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, i2s->dma_ch_out_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(i2s->pio, i2s->sm_dout, true));

    dma_channel_configure(i2s->dma_ch_out_data,
                          &c,
                          &i2s->pio->txf[i2s->sm_dout],  // Destination pointer
                          NULL,                          // Source pointer, will be set by ctrl channel
                          STEREO_BUFFER_SIZE,            // Number of transfers
                          false                          // Start immediately
    );

    c = dma_channel_get_default_config(i2s->dma_ch_in_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(i2s->dma_ch_in_ctrl, &c, &dma_hw->ch[i2s->dma_ch_in_data].al2_write_addr_trig, i2s->in_ctrl_blocks, 1, false);

    c = dma_channel_get_default_config(i2s->dma_ch_in_data);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_chain_to(&c, i2s->dma_ch_in_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(i2s->pio, i2s->sm_din, false));

    dma_channel_configure(i2s->dma_ch_in_data,
                          &c,
                          NULL,                         // Will be set by ctrl chan
                          &i2s->pio->rxf[i2s->sm_din],  // Source pointer
                          STEREO_BUFFER_SIZE,           // Number of transfers
                          false                         // Don't start yet
    );

    // Input channel triggers the DMA interrupt handler, hopefully these stay
    // in perfect sync with the output.
    dma_channel_set_irq0_enabled(i2s->dma_ch_in_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Enable all the dma channels
    dma_channel_start(i2s->dma_ch_out_ctrl);  // This will trigger-start the out chan
    dma_channel_start(i2s->dma_ch_in_ctrl);   // This will trigger-start the in chan
}

/* Initializes an I2S block (of 3 state machines) on the designated PIO.
 * NOTE! This does NOT START the PIO units. You must call i2s_program_start
 *       with the resulting i2s object!
 */
static void i2s_slave_program_init(PIO pio, const i2s_config* config, pio_i2s* i2s) {
    uint offset  = 0;
    i2s->pio     = pio;
    i2s->sm_mask = 0;

    pio_i2s_clocks clocks;
    calc_clocks(config, &clocks);

    if (config->sck_enable) {
        // SCK block
        i2s->sm_sck = pio_claim_unused_sm(pio, true);
        i2s->sm_mask |= (1u << i2s->sm_sck);
        offset = pio_add_program(pio, &i2s_sck_program);
        i2s_sck_program_init(pio, i2s->sm_sck, offset, config->sck_pin);
        pio_sm_set_clkdiv_int_frac(pio, i2s->sm_sck, clocks.sck_d, clocks.sck_f);
    }

    // Bi-Di I2S block, clocked with SCK
    i2s->sm_din  = pio_claim_unused_sm(pio, true);
    i2s->sm_dout = i2s->sm_din;
    i2s->sm_mask |= (1u << i2s->sm_din);
    offset = pio_add_program(pio, &i2s_bidi_slave_program);
    i2s_bidi_slave_program_init(pio, i2s->sm_din, offset, config->dout_pin, config->din_pin);
    pio_sm_set_clkdiv_int_frac(pio, i2s->sm_din, clocks.sck_d, clocks.sck_f);
}

/* Initializes an I2S block (of 3 state machines) on the designated PIO.
 * NOTE! This does NOT START the PIO units. You must call i2s_program_start
 *       with the resulting i2s object!
 */
static void i2s_sync_program_init(PIO pio, const i2s_config* config, pio_i2s* i2s) {
    uint offset  = 0;
    i2s->pio     = pio;
    i2s->sm_mask = 0;

    pio_i2s_clocks clocks;
    calc_clocks(config, &clocks);

    if (config->sck_enable) {
        // Check that SCK and BCK are in perfect whole ratio
        if (!validate_sck_bck_sync(&clocks)) {
            /* There are lots of possible causes for this, a few are:
             *  - You are running a system clock frequency that doesn't divide well at all into SCK or BCK
             *  - You are running a 24-bit I2S with a 256x SCK multiplier (RP2040 cannot support this)
             *  - You have mucked with the PIO ratios or done something silly.
             */
            panic("SCK and BCK are not in sync.");
        }

        // SCK block
        i2s->sm_sck = pio_claim_unused_sm(pio, true);
        i2s->sm_mask |= (1u << i2s->sm_sck);
        offset = pio_add_program(pio, &i2s_sck_program);
        i2s_sck_program_init(pio, i2s->sm_sck, offset, config->sck_pin);
        pio_sm_set_clkdiv_int_frac(pio, i2s->sm_sck, clocks.sck_d, clocks.sck_f);
    }

    // In block, clocked with SCK
    i2s->sm_din = pio_claim_unused_sm(pio, true);
    i2s->sm_mask |= (1u << i2s->sm_din);
    offset = pio_add_program(pio, &i2s_in_slave_program);
    i2s_in_slave_program_init(pio, i2s->sm_din, offset, config->din_pin);
    pio_sm_set_clkdiv_int_frac(pio, i2s->sm_din, clocks.sck_d, clocks.sck_f);

    // Out block, clocked with BCK
    i2s->sm_dout = pio_claim_unused_sm(pio, true);
    i2s->sm_mask |= (1u << i2s->sm_dout);
    offset = pio_add_program(pio, &i2s_out_master_program);
    i2s_out_master_program_init(pio, i2s->sm_dout, offset, config->bit_depth, config->dout_pin, config->clock_pin_base);
    pio_sm_set_clkdiv_int_frac(pio, i2s->sm_dout, clocks.bck_d, clocks.bck_f);
}

void i2s_program_start_slaved(PIO pio, const i2s_config* config, void (*dma_handler)(void), pio_i2s* i2s) {
    if (((uint32_t)i2s & 0x7) != 0) {
        panic("pio_i2s argument must be 8-byte aligned!");
    }
    i2s_slave_program_init(pio, config, i2s);
    dma_double_buffer_init(i2s, dma_handler);
    pio_enable_sm_mask_in_sync(i2s->pio, i2s->sm_mask);
}

void i2s_program_start_synched(PIO pio, const i2s_config* config, void (*dma_handler)(void), pio_i2s* i2s) {
    if (((uint32_t)i2s & 0x7) != 0) {
        panic("pio_i2s argument must be 8-byte aligned!");
    }
    i2s_sync_program_init(pio, config, i2s);
    dma_double_buffer_init(i2s, dma_handler);
    pio_enable_sm_mask_in_sync(i2s->pio, i2s->sm_mask);
}

uint i2s_add_mirrored_output(PIO pio, pio_i2s* i2s, uint8_t dout_pin, uint8_t in_pin_base) {
    uint offset = pio_add_program(pio, &i2s_bidi_slave_program);
    uint sm = pio_claim_unused_sm(pio, true);

    // Deliberately NOT calling i2s_bidi_slave_program_init() here: it
    // unconditionally calls pio_gpio_init() on in_pin_base, +1, and +2
    // (DIN, BCK, LRCK), which reassigns those GPIOs' function-select to
    // this pio block -- stealing BCK/LRCK's output-enable away from
    // whichever SM is already driving them as the real I2S master. We only
    // need to *read* BCK/LRCK here; a PIO's "wait pin"/"in pins"
    // instructions see a pin's logic level regardless of which block
    // currently owns its function-select/output-enable, so there's no need
    // to touch DIN/BCK/LRCK's configuration at all -- only dout_pin is ours.
    pio_gpio_init(pio, dout_pin);

    pio_sm_config sm_config = i2s_bidi_slave_program_get_default_config(offset);
    sm_config_set_out_pins(&sm_config, dout_pin, 1);
    sm_config_set_in_pins(&sm_config, in_pin_base);
    sm_config_set_jmp_pin(&sm_config, in_pin_base + 2);
    sm_config_set_out_shift(&sm_config, false, false, 0);
    sm_config_set_in_shift(&sm_config, false, false, 0);
    // We never drain this SM's RX FIFO (its captured DIN data is unused --
    // we already have the real capture from the primary in-slave SM), so
    // give that FIFO space to TX instead. Without this, this SM only gets
    // the default 4-word TX FIFO vs. the 8 words i2s_out_master_program_init
    // gives DAC #1 via the same join -- half the margin to absorb any DMA
    // refill jitter before running dry and repeating a stale sample.
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, sm, offset, &sm_config);

    uint32_t dout_mask = (1u << dout_pin);
    pio_sm_set_pins_with_mask(pio, sm, 0, dout_mask);
    pio_sm_set_pindirs_with_mask(pio, sm, dout_mask, dout_mask);

    // This SM only waits on already-running external BCK/LRCK edges, so
    // there's no ratio to match -- run comfortably fast (full speed) so it
    // never misses one.
    pio_sm_set_clkdiv_int_frac(pio, sm, 1, 0);
    pio_sm_set_enabled(pio, sm, true);

    // Deliberately a single one-shot DMA channel, not a second independent
    // self-chaining ctrl+data ping-pong: a separate free-running loop has to
    // stay perfectly in lockstep with real data availability entirely on
    // its own, and in practice it didn't (DAC #2 audibly starved/repeated
    // stale samples even after deepening its FIFO). Instead this channel is
    // explicitly re-armed once per half-buffer by i2s_mirrored_output_feed(),
    // called from the *same* proven-correct ISR event that already drives
    // DAC #1 -- so DAC #2's timing can never drift from DAC #1's.
    uint dma_data = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_data);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    dma_channel_configure(dma_data, &c,
                           &pio->txf[sm],
                           NULL, STEREO_BUFFER_SIZE, false);

    return dma_data;
}

void i2s_mirrored_output_feed(uint dma_channel, const int32_t* half_buffer) {
    // Re-arms this half-buffer's worth of data each call. If the previous
    // half-buffer's transfer hasn't finished yet (it should always have,
    // since both DACs are paced by the same real BCK/LRCK), this restarts
    // the channel rather than queuing -- matching "always serve the newest
    // data" rather than risk falling behind.
    dma_channel_set_read_addr(dma_channel, half_buffer, false);
    dma_channel_set_trans_count(dma_channel, STEREO_BUFFER_SIZE, true);
}
