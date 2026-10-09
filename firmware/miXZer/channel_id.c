#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "i2s.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

// Same physical pins as miXZer.c -- the DACs/amps are wired the same
// regardless of which firmware is running.
#define PIN_MCLK       14
#define PIN_DIN_BASE   3
#define PIN_BCK        4
#define PIN_LRCK       5
#define PIN_DOUT       6
#define PIN_DOUT2      7
#define PIN_MUTE       28

#define FS_HZ          48000
#define SCK_MULT       256
#define BIT_DEPTH      32

static __attribute__((aligned(8))) pio_i2s i2sA;
static volatile uint32_t half_idx = 0;
static uint dac2_dma_channel;
static volatile bool dac2_ready = false;

// ============================================================
// Channel identification tone generator -- true 4-way isolation
// ============================================================
// i2s_mirrored_output_feed() just DMAs whatever buffer pointer it's
// given out to DAC2 -- it only "mirrors" DAC1 because miXZer.c happens
// to pass it DAC1's own out_half buffer. Here DAC2 gets its own,
// independently-filled buffer (dac2_buf), so all four channels can be
// exercised one at a time with nothing else making sound at all:
//   0: DAC1-Left  -> amp1
//   1: DAC1-Right -> amp2
//   2: DAC2-Left  -> amp3
//   3: DAC2-Right -> amp4
// Every other channel is silent while one plays, so whichever driver
// makes sound is unambiguously the one just announced over serial.
//
// A distinct note per channel, all in a moderate, safe mid-range (C5,
// G5, C6, G6 -- ~522/774/1043/1600 Hz) so each channel is identifiable
// by ear alone as well as by the serial log. Deliberately not spanning
// into deep bass or extreme treble for any one channel, since assuming
// which channel is a tweeter vs a woofer is exactly what this tool
// exists to NOT assume. Fixed-point square wave (RP2040 has no
// hardware FPU), no trig needed -- half-period in samples per channel.
#define TONE_AMPLITUDE     2000000    // ~24% of full-scale (+-8.4M for 24-bit)
#define PLAY_SAMPLES       (FS_HZ * 3 / 2)   // 1.5s tone
#define SILENCE_SAMPLES    (FS_HZ / 2)       // 0.5s gap between channels

static const int channel_half_period[4] = { 46, 31, 23, 15 }; // ~C5,G5,C6,G6

static const char *channel_label[4] = {
    "amp1  (DAC1-Left)   -- C5, ~522Hz",
    "amp2  (DAC1-Right)  -- G5, ~774Hz",
    "amp3  (DAC2-Left)   -- C6, ~1043Hz",
    "amp4  (DAC2-Right)  -- G6, ~1600Hz",
};

static volatile int      cycle_state      = 0;   // 0..3, which channel is active
static volatile bool     in_gap           = false;
static volatile uint32_t samples_in_state = 0;
static volatile uint32_t phase_count      = 0;
static volatile int32_t  square           = TONE_AMPLITUDE;
static volatile bool     announce_pending = true;

static int32_t dac2_buf[STEREO_BUFFER_SIZE];

static void __isr dma_irq_handler(void)
{
    dma_hw->ints0 = 1u << i2sA.dma_ch_in_data;

    int32_t *out_half = i2sA.out_ctrl_blocks[half_idx];

    int target_dac  = cycle_state >> 1;   // 0 = DAC1, 1 = DAC2
    int target_slot = cycle_state & 1;    // 0 = Left sample, 1 = Right sample

    for (size_t i = 0; i < STEREO_BUFFER_SIZE; ++i)
    {
        int slot = (int)(i & 1u);
        bool tone_here = !in_gap && (slot == target_slot);
        int32_t tone_val = tone_here ? square : 0;

        out_half[i] = (target_dac == 0) ? tone_val : 0;
        dac2_buf[i] = (target_dac == 1) ? tone_val : 0;

        if (tone_here) {
            if (++phase_count >= (uint32_t)channel_half_period[cycle_state]) {
                phase_count = 0;
                square = -square;
            }
        }

        // Advance the play/gap/next-channel state machine once per frame
        // (every Right-slot sample), not once per raw sample.
        if (slot == 1) {
            uint32_t limit = in_gap ? SILENCE_SAMPLES : PLAY_SAMPLES;
            if (++samples_in_state >= limit) {
                samples_in_state = 0;
                phase_count = 0;
                square = TONE_AMPLITUDE;
                if (in_gap) {
                    in_gap = false;
                    cycle_state = (cycle_state + 1) & 3;
                    announce_pending = true;
                } else {
                    in_gap = true;
                }
            }
        }
    }

    if (dac2_ready) {
        i2s_mirrored_output_feed(dac2_dma_channel, dac2_buf);
    }

    half_idx ^= 1u;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1000);

    gpio_init(PIN_MUTE);
    gpio_set_dir(PIN_MUTE, GPIO_OUT);
    gpio_put(PIN_MUTE, true);   // muted while clocks settle

    set_sys_clock_khz(132000, true);

    i2s_config cfg = {
        .fs             = FS_HZ,
        .sck_mult       = SCK_MULT,
        .bit_depth      = BIT_DEPTH,
        .sck_pin        = PIN_MCLK,
        .dout_pin       = PIN_DOUT,
        .din_pin        = PIN_DIN_BASE,
        .clock_pin_base = PIN_BCK,
        .sck_enable     = true,
    };

    i2s_program_start_synched(pio0, &cfg, dma_irq_handler, &i2sA);

    gpio_set_drive_strength(PIN_MCLK, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_slew_rate(PIN_MCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(PIN_BCK,  GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(PIN_BCK,  GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(PIN_LRCK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(PIN_LRCK, GPIO_SLEW_RATE_SLOW);

    dac2_dma_channel = i2s_add_mirrored_output(pio1, &i2sA, PIN_DOUT2, PIN_DIN_BASE);
    dac2_ready = true;

    printf("\nChannel ID tool running -- true 4-way isolation.\n");
    printf("Each channel plays alone for 1.5s (~1500Hz), then 0.5s silence,\n");
    printf("then the next one. Everything else stays silent throughout.\n\n");
    printf("Settling before unmuting...\n");
    sleep_ms(1500);
    gpio_put(PIN_MUTE, false);
    printf("Unmuted -- cycling now.\n\n");

    while (true) {
        sleep_ms(20);
        if (announce_pending) {
            announce_pending = false;
            printf(">>> Now playing: %s\n", channel_label[cycle_state]);
        }
    }
}
