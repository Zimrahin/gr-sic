#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
import scipy


# Modulates a bit sequence by FIR filtering
def pulse_shape_bits_fir(bits: np.ndarray, fir_taps: np.ndarray, sps: int) -> np.ndarray:
    """Modulates a bit sequence by
    1. Upsampling according to samples per symbol (sps).
    2. Filtering with an FIR filter defined by the FIR taps (fir_taps).
    """
    # Upsample with zeros
    upsampled_bits = np.zeros(len(bits) * sps)
    upsampled_bits[::sps] = bits.astype(np.int16) * 2 - 1  # (-1 to 1)

    # Apply FIR filtering and crop at the end (sps - 1) samples
    return scipy.signal.convolve(upsampled_bits, fir_taps, mode="full")[: -sps + 1]


# Modulates in frequency a real array of symbols. Outputs IQ complex signal
def modulate_frequency(symbols: np.ndarray, fsk_deviation: float, fs: float) -> np.ndarray:
    """Modulates in frequency a real array of symbols. Outputs IQ complex signal."""
    # fsk_deviation: a value of 1 in symbols maps to a frequency of fsk_deviation
    # Compute the phase increment per sample based on fsk_deviation
    phase_increments = symbols * (2 * np.pi * fsk_deviation / fs)

    # Prepending a zero ensures that the signal starts at phase 0
    phase_increments = np.insert(phase_increments, 0, 0)
    phase: np.ndarray = np.cumsum(phase_increments)  # Integrate the phase increments

    return np.exp(1j * phase)  # Return the complex IQ signal


# Generate Gaussian FIR filter taps
def gaussian_fir_taps(sps: int, ntaps: int, bt: float, gain: float = 1.0) -> np.ndarray:
    """Generate Gaussian FIR filter taps"""
    # Scaling factor for time based on BT (bandwidth-bit period product)
    t_scale: float = np.sqrt(np.log(2.0)) / (2 * np.pi * bt)

    t = np.linspace(-(ntaps - 1) / 2, (ntaps - 1) / 2, ntaps)  # Symmetric time indices around zero
    taps = np.exp(-((t / (sps * t_scale)) ** 2) / 2)  # Gaussian function
    return gain * taps / np.sum(taps)  # Normalise and apply gain


# Generate half-sine pulse FIR filter taps
def half_sine_fir_taps(sps: int) -> np.ndarray:
    return np.sin(np.linspace(0, np.pi, sps + 1))


# Modulate input I_chips and Q_chips in quadrature, with half a symbol offset.
def oqpsk_modulate(I_chips: np.ndarray, Q_chips: np.ndarray, fir_taps: np.ndarray, sps: int) -> np.ndarray:
    from .filters import fractional_delay_fir_filter

    """Modulate input I_chips and Q_chips in quadrature, with half a symbol offset."""
    assert I_chips.shape == Q_chips.shape, "I_chips and Q_chips must have the same size"

    # Apply pulse shaping (FIR filtering)
    # Concatenating a 0 at the beginning and end for boundary conditions
    # (first chip and last chip must be in the unit circle)
    hss_I_chips = pulse_shape_bits_fir(np.concatenate((I_chips, [0])), fir_taps=fir_taps, sps=sps)
    hss_Q_chips = pulse_shape_bits_fir(np.concatenate(([0], Q_chips)), fir_taps=fir_taps, sps=sps)

    # Apply half-symbol offset to Quadrature component
    hss_I_chips = fractional_delay_fir_filter(hss_I_chips, sps / 2, same_size=False)
    hss_Q_chips = np.pad(hss_Q_chips, (0, len(hss_I_chips) - len(hss_Q_chips)), mode="constant")

    # Pack into complex array and crop remainders resulting from concatenating
    iq_signal = hss_I_chips + 1j * hss_Q_chips
    iq_signal = iq_signal[sps // 2 :]
    iq_signal = iq_signal[: len(I_chips) * sps + int(np.ceil(sps / 2)) + 1]  # Magic expression found by inspection

    return iq_signal
