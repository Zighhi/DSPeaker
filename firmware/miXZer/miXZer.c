#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "i2s.h"
#include "hardware/dma.h"
#include "hardware/sync.h"

#define PIN_MCLK       14     // GP14: RP2040 -> SCK of WM8782 (12.288 MHz Master Clock)
                               // Isolated from the DIN/BCK/LRCK run to avoid crosstalk.
#define PIN_DIN_BASE   3      // GP3: WM8782 DOUT -> RP2040 (ADC Input Data)
#define PIN_BCK        4      // GP4: Shared Bit Clock (BCK) -- must be DIN+1, fixed by PIO design
#define PIN_LRCK       5      // GP5: Shared Frame Sync (LRCK) -- must be DIN+2, fixed by PIO design
#define PIN_DOUT       6      // GP6: RP2040 -> PCM5102A #1 DIN -> Left speaker (amp1 tweeter, amp2 woofer)
#define PIN_DOUT2      7      // GP7: RP2040 -> PCM5102A #2 DIN -> Right speaker (amp3 tweeter, amp4 woofer)
                               // Independently filled per-sample, NOT a mirror of DAC1
                               // -- see the per-speaker crossover block below.

// Bi-color power LED (salvaged laptop DC jack, two LED dies wired
// back-to-back across 2 leads -- color depends on current direction, not
// voltage level). Driven directly by two GPIOs acting as a push-pull pair;
// no H-bridge needed since a single low-current LED only needs two
// independent output drivers, which each GPIO already is on its own.
#define PIN_LED_A      15
#define PIN_LED_B      26

// Amp mute control: one BC337 transistor per Class-D board, base driven
// (through a resistor) from this single GPIO, collector/emitter across each
// board's mute-pin pair. GPIO HIGH saturates all 4 transistors, shorting
// all 4 mute-pin pairs at once (muted); LOW opens them (unmuted). Held high
// through boot to suppress the power-on pop while clocks/DSP settle.
#define PIN_MUTE       28

#define FS_HZ          48000  // 48 kHz
#define SCK_MULT       256    // 256 * fs = 12.288 MHz
#define BIT_DEPTH      32     // 24-bit audio in 32-bit I2S slots (64 * fs BCK = 3.072 MHz)

// Let clocks and the WM8782 fully settle/sync before releasing mute.
#define BOOT_SETTLE_DELAY_MS 2000

static __attribute__((aligned(8))) pio_i2s i2sA;
static volatile uint32_t half_idx = 0;
static uint dac2_dma_channel;
static volatile bool dac2_ready = false; // guards against the ISR firing before DAC2 init completes

// --- Per-speaker active crossover ---
// Confirmed via the channel_id.c identification tool:
//   amp1 (DAC1-Left)  = Left speaker tweeter
//   amp2 (DAC1-Right) = Left speaker woofer
//   amp3 (DAC2-Left)  = Right speaker tweeter
//   amp4 (DAC2-Right) = Right speaker woofer
// DAC2 is filled independently here, NOT mirrored from DAC1 --
// i2s_mirrored_output_feed() just DMAs whatever buffer it's handed, it
// doesn't require that buffer to be DAC1's own (proven in channel_id.c).
// So each speaker gets its OWN source channel split into a tweeter feed
// and a woofer feed, instead of the old scheme where the I2S Left/Right
// *slot* itself meant tweeter/woofer -- that sent the source's Left
// channel to both tweeters and Right channel to both woofers,
// scrambling stereo imaging. Now:
//   DAC1-Left  (amp1) = HPF(source Left)
//   DAC1-Right (amp2) = LPF(source Left)
//   DAC2-Left  (amp3) = HPF(source Right)
//   DAC2-Right (amp4) = LPF(source Right)
//
// Crossover is a real 4th-order Linkwitz-Riley (LR4) at fc=3kHz: two
// cascaded 2nd-order Butterworth (Q=0.7071) biquads per side, matched
// order on both HPF and LPF. This replaced an earlier design that
// cascaded three 1-pole HPF stages against a 2-pole LPF -- that gave
// enough extra slope to tame the tweeter's own Fs=1350Hz resonance, but
// the order mismatch broke the clean polarity-flip flat-sum property
// and left a real ~9-10dB dip around 1.8kHz in measurement. LR4 fixes
// both at once: it's steeper than that 3-pole hack (-28.4dB at 1350Hz
// vs -24.1dB before, so tweeter protection is if anything better) AND
// matched order, so it sums flat on its own. Verified in the filter
// design math before flashing: same-polarity sum is flat to numerical
// noise (<1e-13dB) across 500Hz-6kHz, while inverted-polarity summing
// (what the old LR2-style design needed) produces a near-total null
// right at crossover instead -- so unlike before, the woofer path here
// runs at NORMAL polarity, no inversion.
//
// RP2040 has no hardware FPU (float is ~80 cycles/op), so this runs in
// fixed point: Direct Form I biquad,
//   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
// Coefficients are Q3.28, not Q0.31 -- some of them exceed +-1.0 (e.g.
// the HPF's b1 = -1.514), so Q0.31's [-1, 1) range doesn't have the
// headroom. Multiplies run in 64-bit to avoid overflow, then rescale
// back down by the Q28 shift.
#define BIQUAD_SHIFT 28
typedef struct { int32_t x1, x2, y1, y2; } biquad_state_t;

// LR4 @ fc=3kHz, fs=48kHz, Q=0.7071 (Butterworth) per stage.
// (b0, b1, b2, a1, a2) in Q28.
static const int32_t HPF_BIQUAD_Q28[5] = { 203226142, -406452284, 203226142, -390370540, 154098572 };
static const int32_t LPF_BIQUAD_Q28[5] = {   8040872,   16081744,   8040872, -390370540, 154098572 };

// Level matching: a tweeter pad (estimated ~7dB, then a
// measurement-corrected ~13.4dB) gave inconsistent gap results across
// sessions (+6.41dB, then -2.53dB, then -16.5dB with the same firmware
// and no code changes in between), so it was pulled entirely. With no
// pad at all, a clean gated measurement (Sep 15) showed the woofer
// running 4.36dB hotter than the tweeter -- the woofer path gets that
// attenuation instead. Pure attenuation (gain < 1, Q0.31) rather than
// boosting the tweeter, to avoid needing extra fixed-point headroom
// and any risk of clipping.
#define WOOFER_PAD_Q31    1299959630   // 0.605341 (~-4.36dB, measured Sep 15)

// Baffle step compensation: below a frequency set by the front baffle's
// width, a driver radiates into full space instead of half space and
// loses up to ~6dB on-axis. Baffle is 110mm wide (from the CAD model,
// both drivers centered on that dimension), giving f_bs = 115/0.11m =
// ~1045Hz. Applied as a +4dB low shelf (conservative vs the full 6dB,
// since these sit close to room boundaries at the listening position
// and will pick up some of that gain from the room itself) to the
// source signal before it splits into the tweeter/woofer crossover, so
// whichever driver covers that range gets the correction.
#define BSC_LPF_ALPHA_Q31 258505988    // 0.120376, fc=1045.5Hz
#define BSC_BOOST_Q31     1256048567   // 0.584893 (G-1) for +4dB shelf

// History, for whoever reads this next: the original crossover was two
// cascaded 1st-order complementary HPF/LPF stages each (LR2-equivalent),
// which needed the woofer polarity inverted to sum flat -- confirmed by
// a real destructive-interference notch at 1.5-2.2kHz that a 30-sample
// delay (tried both directions) only made worse, but polarity inversion
// fixed outright. That got the tweeter's own Fs=1350Hz resonance poking
// through as a +8-10dB peak, so the tweeter HPF was bumped to 3 cascaded
// stages (18dB/oct) while the woofer LPF stayed 2-pole (12dB/oct) --
// fixed the resonance, but the resulting order mismatch broke the clean
// polarity-flip flat-sum property and left a real ~9-10dB dip around
// 1.8kHz. The LR4 design above replaced all of that at once: matched
// order on both sides, steeper than the 3-pole hack, normal (not
// inverted) woofer polarity. No added delay in any of this -- tested
// directly, it never helped; these two drivers are close enough
// together on the baffle that a timing correction isn't needed.
typedef struct { int32_t prev_x, prev_y; } filter_state_t;  // used by the BSC shelf below, not the crossover

static biquad_state_t hpfL1, hpfL2;  // source Left  -> amp1 (Left tweeter), LR4 = 2 cascaded Butterworth stages
static biquad_state_t hpfR1, hpfR2;  // source Right -> amp3 (Right tweeter)
static biquad_state_t lpfL1, lpfL2;  // source Left  -> amp2 (Left woofer)
static biquad_state_t lpfR1, lpfR2;  // source Right -> amp4 (Right woofer)

static filter_state_t bscL, bscR;  // baffle-step shelf, applied pre-crossover-split

static int32_t dac2_buf[STEREO_BUFFER_SIZE];

// --- Live driver isolation mode, switched over USB serial ---
// For measuring each driver's real, in-crossover response in isolation
// (e.g. tweeter level vs woofer level) without physically disconnecting
// speaker wire between takes. Send a single character over the USB
// serial connection at any time:
//   b/B = both (normal operation)
//   t/T = tweeters only (woofers muted, both speakers)
//   w/W = woofers only (tweeters muted, both speakers)
//   s/S = toggle baffle-step shelf on/off, for a clean back-to-back A/B
//         at a fixed mic position instead of comparing across sessions
typedef enum { MODE_BOTH = 0, MODE_TWEETERS_ONLY, MODE_WOOFERS_ONLY } output_mode_t;
static volatile output_mode_t output_mode = MODE_BOTH;
static volatile bool bsc_enabled = true;

// Direct Form I biquad, coefficients c[] = {b0,b1,b2,a1,a2} in Q28.
static inline int32_t biquad_process(int32_t x, biquad_state_t *s, const int32_t *c)
{
    int64_t acc = (int64_t)c[0] * x + (int64_t)c[1] * s->x1 + (int64_t)c[2] * s->x2
                - (int64_t)c[3] * s->y1 - (int64_t)c[4] * s->y2;
    int32_t y = (int32_t)(acc >> BIQUAD_SHIFT);
    s->x2 = s->x1; s->x1 = x;
    s->y2 = s->y1; s->y1 = y;
    return y;
}

static inline int32_t apply_woofer_pad(int32_t x)
{
    return (int32_t)(((int64_t)WOOFER_PAD_Q31 * (int64_t)x) >> 31);
}

// Low shelf: y = x + (G-1)*LPF(x). Unlike every other stage here this one
// adds gain, so unlike the others it can push a sample past full scale on
// already-hot bass content -- clamp instead of letting it wrap around.
static inline int32_t apply_bsc(int32_t x, filter_state_t *s)
{
    int64_t diff = (int64_t)x - (int64_t)s->prev_y;
    int32_t lp = s->prev_y + (int32_t)(((int64_t)BSC_LPF_ALPHA_Q31 * diff) >> 31);
    s->prev_x = x;
    s->prev_y = lp;
    int64_t boosted = ((int64_t)BSC_BOOST_Q31 * (int64_t)lp) >> 31;
    int64_t y = (int64_t)x + boosted;
    if (y > INT32_MAX) y = INT32_MAX;
    else if (y < INT32_MIN) y = INT32_MIN;
    return (int32_t)y;
}

// ISR timing-jitter detector. Each half-buffer is 48 frames @ 48kHz, so this
// IRQ should fire every ~1000us like clockwork. A real-time miss here (DMA
// not keeping the PIO TX FIFO fed, or the ISR itself running late) is
// exactly what an audible crackle/click sounds like -- catch it directly
// instead of guessing at the cause.
#define EXPECTED_IRQ_INTERVAL_US 1000
#define GLITCH_THRESHOLD_US      200   // +/- 20% off expected counts as a glitch
static volatile uint32_t last_irq_time_us = 0;
static volatile uint32_t glitch_count      = 0;
static volatile uint32_t max_interval_us   = 0;
static volatile uint32_t worst_glitch_us   = 0;

// IRQ handler: per-speaker crossover + driver isolation
static void __isr dma_irq_handler(void)
{
    uint32_t now = time_us_32();
    if (last_irq_time_us != 0) {
        uint32_t interval = now - last_irq_time_us;
        if (interval > max_interval_us) max_interval_us = interval;
        uint32_t deviation = (interval > EXPECTED_IRQ_INTERVAL_US)
                                 ? (interval - EXPECTED_IRQ_INTERVAL_US)
                                 : (EXPECTED_IRQ_INTERVAL_US - interval);
        if (deviation > GLITCH_THRESHOLD_US) {
            ++glitch_count;
            if (interval > worst_glitch_us) worst_glitch_us = interval;
        }
    }
    last_irq_time_us = now;

    dma_hw->ints0 = 1u << i2sA.dma_ch_in_data;

    int32_t *in_half  = i2sA.in_ctrl_blocks[half_idx];
    int32_t *out_half = i2sA.out_ctrl_blocks[half_idx];

    for (size_t i = 0; i < STEREO_BUFFER_SIZE; i += 2)
    {
        int32_t src_left  = bsc_enabled ? apply_bsc(in_half[i], &bscL)
                                        : in_half[i];
        int32_t src_right = bsc_enabled ? apply_bsc(in_half[i + 1], &bscR)
                                         : in_half[i + 1];

        // Left speaker (amp1 tweeter / amp2 woofer), both from source Left --
        // LR4: two cascaded Butterworth biquads per side, normal polarity
        int32_t tw_l = biquad_process(src_left, &hpfL1, HPF_BIQUAD_Q28);
        tw_l = biquad_process(tw_l, &hpfL2, HPF_BIQUAD_Q28);
        int32_t wf_l = biquad_process(src_left, &lpfL1, LPF_BIQUAD_Q28);
        wf_l = apply_woofer_pad(biquad_process(wf_l, &lpfL2, LPF_BIQUAD_Q28));

        // Right speaker (amp3 tweeter / amp4 woofer), both from source Right
        int32_t tw_r = biquad_process(src_right, &hpfR1, HPF_BIQUAD_Q28);
        tw_r = biquad_process(tw_r, &hpfR2, HPF_BIQUAD_Q28);
        int32_t wf_r = biquad_process(src_right, &lpfR1, LPF_BIQUAD_Q28);
        wf_r = apply_woofer_pad(biquad_process(wf_r, &lpfR2, LPF_BIQUAD_Q28));

        // Live driver isolation for measurement (see output_mode above) --
        // filters above still run either way, so state stays continuous
        // and there's no click/discontinuity switching modes mid-stream.
        if (output_mode == MODE_TWEETERS_ONLY) {
            wf_l = 0;
            wf_r = 0;
        } else if (output_mode == MODE_WOOFERS_ONLY) {
            tw_l = 0;
            tw_r = 0;
        }

        out_half[i]     = tw_l;   // DAC1-Left  -> amp1
        out_half[i + 1] = wf_l;   // DAC1-Right -> amp2
        dac2_buf[i]     = tw_r;   // DAC2-Left  -> amp3
        dac2_buf[i + 1] = wf_r;   // DAC2-Right -> amp4
    }

    // DAC #2 is independently filled above (see per-speaker crossover
    // block) -- this feeds that buffer out, not a mirror of DAC1's.
    if (dac2_ready) {
        i2s_mirrored_output_feed(dac2_dma_channel, dac2_buf);
    }

    half_idx ^= 1u;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(1000);

    gpio_init(PIN_LED_A);
    gpio_init(PIN_LED_B);
    gpio_set_dir(PIN_LED_A, GPIO_OUT);
    gpio_set_dir(PIN_LED_B, GPIO_OUT);
    gpio_put(PIN_LED_A, false);  // start on "amber": muted/starting up (A low, B high --
    gpio_put(PIN_LED_B, true);   // swapped from the first guess to match the real LED wiring)

    gpio_init(PIN_MUTE);
    gpio_set_dir(PIN_MUTE, GPIO_OUT);
    gpio_put(PIN_MUTE, true);    // muted at boot -- released once the audio path is confirmed stable

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

    // EMI mitigation: MCLK is isolated on GP14 so it can stay fast/full
    // drive for signal integrity. BCK/LRCK sit permanently adjacent to DIN
    // (fixed by the PIO's pin layout), so slow their edges down to reduce
    // crosstalk into the ADC data line.
    //
    // (Tested fast/full-drive BCK/LRCK as a DAC2-distortion diagnostic --
    // made no difference, so it's not a signal-integrity issue. Reverted.)
    gpio_set_drive_strength(PIN_MCLK, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_slew_rate(PIN_MCLK, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(PIN_BCK,  GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(PIN_BCK,  GPIO_SLEW_RATE_SLOW);
    gpio_set_drive_strength(PIN_LRCK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(PIN_LRCK, GPIO_SLEW_RATE_SLOW);

    // Second output for PCM5102 #2, slaved to the BCK/LRCK the sync above
    // already generates. Independently filled per-sample by the
    // per-speaker crossover in the ISR (see dac2_buf) -- not a mirror.
    // Runs on PIO1, not PIO0: PIO0 is already full (SCK + in-slave +
    // out-master = 3 programs sharing its 32-instruction memory), and a 4th
    // program there overflows it and corrupts the other three. PIO1 is
    // completely free and can still read GP3/4/5 (DIN/BCK/LRCK) fine --
    // GPIO inputs are readable by either PIO block regardless of which one
    // drives them.
    dac2_dma_channel = i2s_add_mirrored_output(pio1, &i2sA, PIN_DOUT2, PIN_DIN_BASE);
    dac2_ready = true;

    printf("\nAudio passthrough running (both DACs). Settling %d ms before unmute...\n",
           BOOT_SETTLE_DELAY_MS);
    sleep_ms(BOOT_SETTLE_DELAY_MS);

    // Audio path confirmed stable at this point (clocks settled, both DACs
    // running) -- release mute and flip the LED to "white" (running).
    gpio_put(PIN_MUTE, false);
    gpio_put(PIN_LED_A, true);
    gpio_put(PIN_LED_B, false);

    // The periodic "ISR timing" status report and the bulk audio-capture
    // dump that used to live here have both done their jobs (glitches
    // reliably read 0, and the capture caught the ground-loop click back
    // when that was the mystery). Removed -- that recurring USB CDC
    // traffic was itself a source of audible artifacts once the amps were
    // unmuted, and neither is needed now. The underlying glitch_count/
    // worst_glitch_us/max_interval_us tracking in the ISR is left in place
    // and still accumulates silently -- add a printf back here if it's
    // ever needed for debugging again.

    printf("\nDriver isolation mode: send 'b'=both 't'=tweeters-only 'w'=woofers-only over serial\n");
    printf("Current mode: BOTH\n\n");

    while (true)
    {
        sleep_ms(200);

        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == 'b' || c == 'B') {
                output_mode = MODE_BOTH;
                printf("Mode: BOTH (tweeters + woofers, normal operation)\n");
            } else if (c == 't' || c == 'T') {
                output_mode = MODE_TWEETERS_ONLY;
                printf("Mode: TWEETERS ONLY (woofers muted, both speakers)\n");
            } else if (c == 'w' || c == 'W') {
                output_mode = MODE_WOOFERS_ONLY;
                printf("Mode: WOOFERS ONLY (tweeters muted, both speakers)\n");
            } else if (c == 's' || c == 'S') {
                bsc_enabled = !bsc_enabled;
                printf("Baffle-step shelf: %s\n", bsc_enabled ? "ON (+4dB below ~1045Hz)" : "OFF (bypassed)");
            }
        }
    }
}
