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


# Pads iq_samples_interference with zeros at the beginning (delay_zero_padding)
# and at the end to match the length of iq_samples.
def pad_interference(
    iq_samples: np.ndarray, iq_samples_interference: np.ndarray, delay_zero_padding: int
) -> np.ndarray:
    """
    Pads iq_samples_interference with zeros at the beginning (delay_zero_padding)
    and at the end to match the length of iq_samples.
    """
    assert 0 <= delay_zero_padding < len(iq_samples), "delay_zero_padding out of range"

    # If the interference plus delay is too long, crop the interference
    max_interference_length = len(iq_samples) - delay_zero_padding
    if len(iq_samples_interference) > max_interference_length:
        iq_samples_interference = iq_samples_interference[:max_interference_length]

    # Zero padding at the end
    end_padding = len(iq_samples) - (len(iq_samples_interference) + delay_zero_padding)

    padded_interference = np.concatenate(
        [
            np.zeros(delay_zero_padding, dtype=iq_samples_interference.dtype),
            iq_samples_interference,
            np.zeros(end_padding, dtype=iq_samples_interference.dtype),
        ]
    )

    return padded_interference


# Multiply an input complex exponential.
def multiply_by_complex_exponential(
    input_signal: np.ndarray,
    sample_rate: float,
    freq: float,
    phase: float = 0,
    amplitude: float = 1,
    offset: complex = 0,
) -> np.ndarray:
    """Multiply an input complex exponential."""
    t = np.arange(input_signal.size) / sample_rate
    complex_cosine = offset + amplitude * np.exp(1j * (2 * np.pi * freq * t + phase))

    return input_signal * complex_cosine


# Correlation wrapper to estimate where an interference is on an affected packet.
def correlation_wrapper(affected: np.ndarray, interference: np.ndarray) -> np.ndarray:
    """Correlation wrapper to estimate where an interference is on an affected packet."""
    template_energy = np.sum(np.abs(interference) ** 2)  # For amplitude estimation
    offset = len(interference) - 1  # Because using mode="full"
    correlation = scipy.signal.correlate(affected, interference, mode="full")[offset : offset + len(affected)]
    correlation /= template_energy  # Normalise for amplitude estimation
    return correlation


# Subtract a known interference from an affected packet.
def subtract_interference_wrapper(
    affected: np.ndarray,
    interference: np.ndarray,
    sample_rate: float,
    freq_offsets: list[float] | range,
    amplitude: float = None,
    phase: float = None,
    samples_shift: int = None,
    *,
    fine_step: float | None = None,  # Step size (Hz) for the fine search
    fine_window: float | None = None,  # Half-width (Hz) of the window around best coarse frequency
    verbose: bool = False,
) -> np.ndarray:
    """Subtract a known interference from an affected packet."""
    est_frequency, est_amplitude, est_phase, est_samples_shift = find_interference_parameters(
        affected, interference, freq_offsets, sample_rate, fine_step=fine_step, fine_window=fine_window
    )
    if verbose:
        print(f"{est_frequency = } [Hz]")
        print(f"{est_amplitude = :.2f} [-]")
        print(f"{est_phase = :.2f} [rad]")
        print(f"{est_samples_shift = } [samples]")

    # Update parameters if not defined
    amplitude, phase, samples_shift = map(
        lambda old, new: new if old is None else old,
        (amplitude, phase, samples_shift),
        (est_amplitude, est_phase, est_samples_shift),
    )

    # Subtract the interference
    ready_to_subtract = multiply_by_complex_exponential(
        interference, sample_rate, freq=est_frequency, phase=phase, amplitude=amplitude
    )
    ready_to_subtract = pad_interference(affected, ready_to_subtract, samples_shift)

    return affected - ready_to_subtract


# Estimate best frequency offset, amplitude, phase and sample shift to subtract from affected packet.
def find_interference_parameters(
    affected: np.ndarray,
    interference: np.ndarray,
    freq_offsets: list[float] | range,
    sample_rate: float,
    *,
    fine_step: float | None = None,  # Step size (Hz) for the fine search
    fine_window: float | None = None,  # Half-width (Hz) of the window around best coarse frequency
) -> tuple[float, float, float, int]:
    """Estimate best frequency offset, amplitude, phase and sample shift to subtract from affected packet.

    If fine_step and fine_window are not None:
      1. Coarse search over freq_offsets
      2. Fine search around the best coarse frequency within ±fine_window at steps of fine_step
    """

    def _single_search(freq_list):
        best_amp = -np.inf
        best_freq = 0.0
        best_ph = 0.0
        best_idx = 0

        for f in freq_list:
            rotated = multiply_by_complex_exponential(interference, sample_rate=sample_rate, freq=f)
            corr = correlation_wrapper(affected, rotated)
            abs_corr = np.abs(corr)
            idx = np.argmax(abs_corr)
            amp = abs_corr[idx]

            if amp > best_amp:
                best_amp = amp
                best_freq = f
                best_ph = np.angle(corr[idx])
                best_idx = idx

        return best_freq, best_amp, best_ph, best_idx

    if fine_step is None or fine_window is None:
        return _single_search(freq_offsets)

    # Else, do fine search after coarse search
    coarse_freq, _, _, _ = _single_search(freq_offsets)
    low = coarse_freq - fine_window
    high = coarse_freq + fine_window
    fine_freqs = np.arange(low, high, fine_step)
    return _single_search(fine_freqs)
