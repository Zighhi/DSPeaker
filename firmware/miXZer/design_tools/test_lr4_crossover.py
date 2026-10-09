"""
Linkwitz-Riley 4th Order (LR4) 3 kHz Crossover Coefficient Calculator & Frequency Response Verifier
Calculates biquad coefficients for 48 kHz sampling rate and plots magnitude & phase response.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

fs = 48000.0  # Sampling rate
fc = 3000.0   # Crossover frequency 3 kHz (matches miXZer.c)

# LR4 = Cascaded 2nd-order Butterworth filters (Q = 0.7071)
# 1. Low-Pass Butterworth 2nd order (Q = 0.70710678)
b_lp2, a_lp2 = signal.butter(2, fc, fs=fs, btype='low', analog=False)

# 2. High-Pass Butterworth 2nd order (Q = 0.70710678)
b_hp2, a_hp2 = signal.butter(2, fc, fs=fs, btype='high', analog=False)

# LR4 Low-Pass = LP2 * LP2
w, h_lp2 = signal.freqz(b_lp2, a_lp2, worN=8192, fs=fs)
h_lp4 = h_lp2 ** 2

# LR4 High-Pass = HP2 * HP2
w, h_hp2 = signal.freqz(b_hp2, a_hp2, worN=8192, fs=fs)
h_hp4 = h_hp2 ** 2

# Summed Response
h_sum = h_lp4 + h_hp4
mag_sum_db = 20 * np.log10(np.abs(h_sum))

print("============================================================")
print("LINKWITZ-RILEY 4TH ORDER (LR4) 3 kHz BIQUAD COEFFICIENTS:")
print("============================================================")
print("2nd-Order Butterworth Low-Pass (LP2) Biquad Coefficients:")
print(f"  b0 = {b_lp2[0]:.8f}")
print(f"  b1 = {b_lp2[1]:.8f}")
print(f"  b2 = {b_lp2[2]:.8f}")
print(f"  a1 = {a_lp2[1]:.8f}")
print(f"  a2 = {a_lp2[2]:.8f}")

print("\n2nd-Order Butterworth High-Pass (HP2) Biquad Coefficients:")
print(f"  b0 = {b_hp2[0]:.8f}")
print(f"  b1 = {b_hp2[1]:.8f}")
print(f"  b2 = {b_hp2[2]:.8f}")
print(f"  a1 = {a_hp2[1]:.8f}")
print(f"  a2 = {a_hp2[2]:.8f}")

# Same values as int32 Q28 arrays (b0, b1, b2, a1, a2), ready to paste into miXZer.c
q28 = lambda b, a: ", ".join(str(int(round(v * (1 << 28)))) for v in (*b, a[1], a[2]))
print(f"\nHPF_BIQUAD_Q28[5] = {{ {q28(b_hp2, a_hp2)} }};")
print(f"LPF_BIQUAD_Q28[5] = {{ {q28(b_lp2, a_lp2)} }};")
print("============================================================")

# Plot Magnitude Response
fig, ax = plt.subplots(figsize=(10, 6), dpi=180)
fig.patch.set_facecolor('#0d1117')
ax.set_facecolor('#0d1117')

ax.semilogx(w, 20 * np.log10(np.abs(h_lp4)), label='Woofer LR4 Low-Pass (-24 dB/oct)', color='#00b0ff', linewidth=2)
ax.semilogx(w, 20 * np.log10(np.abs(h_hp4)), label='Tweeter LR4 High-Pass (-24 dB/oct)', color='#d500f9', linewidth=2)
ax.semilogx(w, mag_sum_db, label='Acoustic Summed Response (0 dB Flat)', color='#00e676', linestyle='--', linewidth=2.5)

ax.axvline(fc, color='#ffab00', linestyle=':', label='Crossover Frequency Fc = 3 kHz (-6.0 dB point)')
ax.axhline(-6.02, color='#ffab00', linestyle='--', alpha=0.5)

ax.set_xlim(20, 20000)
ax.set_ylim(-40, 5)
ax.grid(True, which='both', color='#263238', linestyle=':')
ax.set_title("LINKWITZ-RILEY 4TH ORDER (LR4) 3 kHz CROSSOVER FREQUENCY RESPONSE", color='white', fontsize=11, fontweight='bold', pad=15)
ax.set_xlabel("Frequency (Hz)", color='#cccccc', fontsize=9)
ax.set_ylabel("Magnitude (dB)", color='#cccccc', fontsize=9)
ax.tick_params(colors='#cccccc')
ax.legend(facecolor='#161b22', edgecolor='#333333', labelcolor='white')

plt.tight_layout()
import os
artifact_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lr4_crossover_response.png")
plt.savefig(artifact_path)
print(f"Saved LR4 Crossover Response Plot: {artifact_path}")
