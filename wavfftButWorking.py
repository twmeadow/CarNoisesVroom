#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 21 15:36:26 2025

@author: tylermeadows
"""

import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt
import sounddevice as sd

# --- Load WAV ---
sample_rate, dataF = wavfile.read("aero_car_noise.wav")

def plotFFT(data):
    # If stereo, convert to mono
    if data.ndim > 1:
        data = data.mean(axis=1)
    
    # --- FFT ---
    N = len(data)
    fft_vals = np.fft.fft(data)
    fft_freqs = np.fft.fftfreq(N, 1/sample_rate)
    
    # Keep only positive freq half
    pos_mask = fft_freqs >= 0
    fft_vals = fft_vals[pos_mask]
    fft_freqs = fft_freqs[pos_mask]
    
    # --- Plot ---
    plt.plot(fft_freqs, np.abs(fft_vals) / np.max(np.abs(fft_vals)))
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.xlim(0, 800)
    plt.tight_layout()
    plt.show()
    
    # return 10 largest magnitudes (just values)
    k = 10
    largest = np.partition(np.abs(fft_vals), -k)[-k:]
    
    return largest


def make_sine_sum(freqs, mags, fs, t, add = 0):
    freqs = np.array(freqs)
    mags  = np.array(mags)

    signal = np.zeros_like(t)

    # zip requires equal lengths
    for f, m in zip(freqs, mags):
        signal += m * np.sin(2 * np.pi * (f + add) * t)

    return signal


# --- Example usage ---
fs = 44100  # sampling rate
duration = 2  # seconds

# Time array for 1 second
t = np.linspace(0, duration, int(fs * duration), endpoint=False)

# INPUT FREQUENCIES & MAGNITUDES
maxfreq = 0

fl = [175, 250, 380, 410, 320, 20, 80, 100, 120, 200, 150]
ml = [.6, 1, .6, .6, .4, .15, .15, .15, .15, .3, .5]  # <-- fixed to have 8 magnitudes

# generate your signal
signal = make_sine_sum(fl, ml, fs, t)



# FFT your synthesized signal
signal = make_sine_sum(fl, ml, fs, t)
sd.play(signal, fs)
plotFFT(signal)