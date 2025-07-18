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
from abc import ABC, abstractmethod
from typing import Literal, get_args

from .demodulation import symbol_sync, demodulate_frequency, binary_slicer, TEDType
from .modulation import gaussian_fir_taps, half_sine_fir_taps
from .filters import single_pole_iir_filter
from .packet_utils import (
    correlate_access_code,
    compute_crc,
    ble_whitening,
    pack_bits_to_uint8,
    generate_access_code_ble,
    pack_chips_to_bytes,
    preamble_detection_802154,
)

# FSK demodulation types
DemodulationType = Literal[
    "INSTANTANEOUS_FREQUENCY",
    "BAND_PASS",
]

# Receiver protocols
ReceiverType = Literal[
    "BLE",
    "IEEE802154",
]


class Receiver(ABC):
    crc_size: int = None  # Bytes
    max_packet_len: int = None  # Bytes

    @abstractmethod  # Receives an array of complex data and returns hard decision array
    def demodulate(
        self,
        iq_samples: np.ndarray,
        demodulation_type: DemodulationType,
        ted_type: TEDType,
    ) -> np.ndarray:
        pass

    @abstractmethod  # Receive hard decisions and return dictionary with detected packets
    def process_phy_packet(self, *args, **kwargs) -> list[dict]:
        pass

    @abstractmethod  # Wrap previous methods in one call
    def demodulate_to_packet(self, *args, **kwargs) -> list[dict]:
        pass

    def set_symbol_sync_parameters(  # Set specific symbol sync parameters
        self,
        TED_gain: float = 1.0,
        loop_BW: float = 4.5e-3,
        damping: float = 1.0,
        max_deviation: float = 0,
    ) -> None:
        self._symbol_sync_param_TED_gain = TED_gain
        self._symbol_sync_param_loop_BW = loop_BW
        self._symbol_sync_param_damping = damping
        self._symbol_sync_param_max_deviation = max_deviation


class ReceiverBLE(Receiver):
    _crc_size: int = 3  # 3 bytes CRC for BLE
    _max_payload_size: int = 255  # Bytes

    def __init__(self, sample_rate: int, transmission_rate: float = 1e6):
        self._sample_rate = int(sample_rate)
        self.transmission_rate: float = transmission_rate  # BLE 1 Mb/s or 2Mb/s

        # Generate Gaussian taps and convolve with rectangular window
        gauss_taps = gaussian_fir_taps(sps=self._sps, ntaps=self._sps, bt=0.5)
        gauss_taps = scipy.signal.convolve(gauss_taps, np.ones(self._sps))
        gauss_taps /= np.sum(gauss_taps)  # Unitary gain
        self._gauss_taps = gauss_taps

        self.set_symbol_sync_parameters()

    # Receives an array of complex data and returns hard decision array
    def demodulate(
        self,
        iq_samples: np.ndarray,
        demodulation_type: DemodulationType = "INSTANTANEOUS_FREQUENCY",
        ted_type: TEDType = "MOD_MUELLER_AND_MULLER",
    ) -> np.ndarray:
        """Receives an array of complex data and returns hard decision array."""

        if demodulation_type == "INSTANTANEOUS_FREQUENCY":
            # Low pass matched filter (Gaussian kernel)
            # Generate Gaussian taps and convolve with rectangular window
            iq_samples = scipy.signal.correlate(iq_samples, self._gauss_taps, mode="full")

            # Frequency demodulation
            freq_samples = demodulate_frequency(
                iq_samples, gain=(self._sample_rate) / (2 * np.pi * self._fsk_deviation)
            )
            freq_samples -= single_pole_iir_filter(freq_samples, alpha=160e-6)
            before_symbol_sync = freq_samples

        elif demodulation_type == "BAND_PASS":
            complex_exp = np.exp(
                1j * 2 * np.pi * self._fsk_deviation * np.arange(len(self._gauss_taps)) / self._sample_rate
            )
            gauss_bandpass_lower = self._gauss_taps / complex_exp
            gauss_bandpass_higher = self._gauss_taps * complex_exp

            # Band-pass filter (complex signal, complex filter, complex output)
            iq_samples_lower = scipy.signal.correlate(iq_samples, gauss_bandpass_lower, mode="full")
            iq_samples_higher = scipy.signal.correlate(iq_samples, gauss_bandpass_higher, mode="full")

            # Magnitude squared and subtract
            iq_samples_lower_square = iq_samples_lower * np.conj(iq_samples_lower)
            iq_samples_higher_square = iq_samples_higher * np.conj(iq_samples_higher)
            before_symbol_sync = np.real(iq_samples_higher_square - iq_samples_lower_square)
            before_symbol_sync /= np.max(before_symbol_sync)

        else:
            raise ValueError(
                f"Invalid demodulation type '{demodulation_type}'. Choose from {list(get_args(DemodulationType))}"
            )

        # Symbol synchronisation
        bit_samples = symbol_sync(
            before_symbol_sync,
            sps=self._sps,
            ted_type=ted_type,
            TED_gain=self._symbol_sync_param_TED_gain,
            loop_BW=self._symbol_sync_param_loop_BW,
            damping=self._symbol_sync_param_damping,
            max_deviation=self._symbol_sync_param_max_deviation,
        )
        bit_samples = binary_slicer(bit_samples)

        return bit_samples

    # Receive hard decisions (bit samples) and return dictionary with detected packets
    def process_phy_packet(
        self, bit_samples: np.ndarray, base_address: int = 0x12345678, preamble_threshold: int = 4
    ) -> list[dict]:
        """Receive hard decisions (bit samples) and return dictionary with detected packets."""
        # Decode detected packets found in bit_samples array
        preamble_positions: np.ndarray = correlate_access_code(
            bit_samples, generate_access_code_ble(base_address), threshold=preamble_threshold
        )
        detected_packets: list[dict] = []

        # Read packets starting from the end of the preamble
        for preamble in preamble_positions:
            # Length reading for BLE
            payload_start: int = preamble + 2 * 8  # S0 + length byte
            header = pack_bits_to_uint8(bit_samples[preamble:payload_start])  # Whitened
            header, lsfr = ble_whitening(header)  # De-whitened, length_byte includes S0
            payload_length: int = header[-1]  # Payload length in bytes, without CRC

            # Payload reading and de-whitening
            total_bytes: int = payload_length + self._crc_size
            payload_and_crc = pack_bits_to_uint8(bit_samples[payload_start : payload_start + total_bytes * 8])
            payload_and_crc, _ = ble_whitening(payload_and_crc, lsfr)

            # CRC check
            header_and_payload = np.concatenate((header, payload_and_crc[: -self._crc_size]))
            computed_crc = compute_crc(
                header_and_payload, crc_init=0x00FFFF, crc_poly=0x00065B, crc_size=self._crc_size
            )
            crc_check = True if (computed_crc == payload_and_crc[-self._crc_size :]).all() else False

            payload = header_and_payload[2:]  # Remove CRC bytes

            # Append dictionary to return list
            detected_packets.append(
                {"payload": payload, "length": len(payload), "crc_check": crc_check, "position_in_array": payload_start}
            )

        return detected_packets

    # Receive IQ data and return dictionary with detected packets.
    def demodulate_to_packet(
        self,
        iq_samples: np.ndarray,
        demodulation_type: DemodulationType = "INSTANTANEOUS_FREQUENCY",
        ted_type: TEDType = "MOD_MUELLER_AND_MULLER",
        base_address: int = 0x12345678,
        preamble_threshold: int = 4,
    ) -> list[dict]:
        """Receive IQ data and return dictionary with detected packets."""
        bit_samples = self.demodulate(
            iq_samples, demodulation_type=demodulation_type, ted_type=ted_type
        )  # From IQ samples to hard decisions
        received_packets: list[dict] = self.process_phy_packet(
            bit_samples, base_address=base_address, preamble_threshold=preamble_threshold
        )  # From hard decisions to packets

        return received_packets

    @property
    def transmission_rate(self) -> float:
        return self._transmission_rate

    @transmission_rate.setter
    def transmission_rate(self, rate: float) -> None:
        self._transmission_rate = rate
        self._fsk_deviation = self.transmission_rate * 0.25
        self._sps = int(self._sample_rate / self.transmission_rate)


class Receiver802154(Receiver):
    _crc_size: int = 2  # 2 bytes CRC for IEEE 802.15.4
    _max_packet_len: int = 127  # Bytes

    # Chip mapping for differential MSK encoding
    chip_mapping: np.ndarray = np.array(
        [
            0xE077AE6C,  # 0
            0xCE077AE6,  # 1
            0x6CE077AE,  # 2
            0xE6CE077A,  # 3
            0xAE6CE077,  # 4
            0x7AE6CE07,  # 5
            0x77AE6CE0,  # 6
            0x877AE6CE,  # 7
            0x1F885193,  # 8
            0x31F88519,  # 9
            0x931F8851,  # A
            0x1931F885,  # B
            0x51931F88,  # C
            0x851931F8,  # D
            0x8851931F,  # E
            0x78851931,  # F
        ],
        dtype=np.uint32,
    )

    def __init__(self, sample_rate: int, transmission_rate: float = 2e6):
        self._sample_rate = int(sample_rate)
        self.transmission_rate: float = transmission_rate  # IEEE 802.15.4 2 Mchip/s

        # Matched filtering (Half Sine FIR taps)
        hss_taps = half_sine_fir_taps(2 * self._sps)  # One symbol is two chips long
        hss_taps /= np.sum(hss_taps)  # Unitary gain
        self.hss_taps = hss_taps

        # Matched filtering (Rect taps) after frequency demodulation
        rect_taps = np.ones(self._sps)
        rect_taps /= np.sum(rect_taps)  # Unitary gain
        self.rect_taps = rect_taps

        self.set_symbol_sync_parameters()

    # Receives an array of complex data and returns hard decision array
    def demodulate(
        self,
        iq_samples: np.ndarray,
        demodulation_type: DemodulationType = "INSTANTANEOUS_FREQUENCY",
        ted_type: TEDType = "MOD_MUELLER_AND_MULLER",
    ) -> np.ndarray:
        """Receives an array of complex data and returns hard decision array."""

        if demodulation_type == "INSTANTANEOUS_FREQUENCY":
            # Low pass matched filter (half sine shape taps)
            iq_samples = scipy.signal.correlate(iq_samples, self.hss_taps, mode="full")

            # Frequency demodulation
            freq_samples = demodulate_frequency(iq_samples, gain=(self._sample_rate) / (2 * np.pi * self.fsk_deviation))
            freq_samples -= single_pole_iir_filter(freq_samples, alpha=160e-6)
            before_symbol_sync = scipy.signal.correlate(freq_samples, self.rect_taps, mode="full")

        elif demodulation_type == "BAND_PASS":
            complex_exp = np.exp(
                1j * 2 * np.pi * self.fsk_deviation * np.arange(len(self.rect_taps)) / self._sample_rate
            )
            rect_bandpass_lower = self.rect_taps / complex_exp
            rect_bandpass_higher = self.rect_taps * complex_exp

            # Band-pass filter (complex signal, complex filter, complex output)
            iq_samples_lower = scipy.signal.correlate(iq_samples, rect_bandpass_lower, mode="full")
            iq_samples_higher = scipy.signal.correlate(iq_samples, rect_bandpass_higher, mode="full")

            # Magnitude squared and subtract
            iq_samples_lower_square = iq_samples_lower * np.conj(iq_samples_lower)
            iq_samples_higher_square = iq_samples_higher * np.conj(iq_samples_higher)
            before_symbol_sync = np.real(iq_samples_higher_square - iq_samples_lower_square)
            before_symbol_sync /= np.max(before_symbol_sync)

        else:
            raise ValueError(
                f"Invalid demodulation type '{demodulation_type}'. Choose from {list(get_args(DemodulationType))}"
            )

        # Symbol synchronisation
        bit_samples = symbol_sync(
            before_symbol_sync,
            sps=self._sps,
            ted_type=ted_type,
            TED_gain=self._symbol_sync_param_TED_gain,
            loop_BW=self._symbol_sync_param_loop_BW,
            damping=self._symbol_sync_param_damping,
            max_deviation=self._symbol_sync_param_max_deviation,
        )
        bit_samples = binary_slicer(bit_samples)

        return bit_samples

    # Receive hard decisions (bit samples) and return dictionary with detected packets
    def process_phy_packet(
        self,
        chip_samples: np.ndarray,
        preamble_threshold: int = 8,
        CRC_included: bool = True,
    ) -> list[dict]:
        """Receive hard decisions (bit samples) and return dictionary with detected packets."""

        preamble_positions: np.ndarray = preamble_detection_802154(chip_samples, preamble_threshold, self.chip_mapping)
        detected_packets: list[dict] = []

        # Read packets starting from the end of the preamble
        for preamble in preamble_positions:
            # Length reading for IEEE 802.15.4
            payload_start: int = preamble + 2 * 32  # 2 nibbles, 1 byte
            payload_length = pack_chips_to_bytes(
                chip_samples[preamble:payload_start], num_bytes=1, chip_mapping=self.chip_mapping, threshold=10
            )  # Payload length in bytes
            payload_length = payload_length[0]
            if payload_length > self._max_packet_len:  # Maximum payload length is 127 bytes
                continue  # The packet is lost (not valid)

            try:
                # Payload reading
                payload = pack_chips_to_bytes(
                    chip_samples[payload_start : payload_start + payload_length * 64],
                    num_bytes=payload_length,
                    chip_mapping=self.chip_mapping,
                    threshold=32,  # Once the preamble and length are detected, just return closest guess
                )
            except AssertionError as e:  # There was a problem processing the packet
                continue

            crc_check = None
            # CRC check
            if CRC_included:
                computed_crc = compute_crc(
                    payload[: -self._crc_size], crc_init=0x0000, crc_poly=0x011021, crc_size=self._crc_size
                )
                crc_check = True if (computed_crc == payload[-self._crc_size :]).all() else False
                payload = payload[: -self._crc_size]  # Remove CRC bytes

            # Append dictionary to return list
            detected_packets.append(
                {"payload": payload, "length": len(payload), "crc_check": crc_check, "position_in_array": payload_start}
            )

        return detected_packets

    # Receive IQ data and return dictionary with detected packets.
    def demodulate_to_packet(
        self,
        iq_samples: np.ndarray,
        demodulation_type: DemodulationType = "INSTANTANEOUS_FREQUENCY",
        ted_type: TEDType = "MOD_MUELLER_AND_MULLER",
        preamble_threshold: int = 8,
        CRC_included: bool = True,
    ) -> list[dict]:
        """Receive IQ data and return dictionary with detected packets."""
        bit_samples = self.demodulate(
            iq_samples, demodulation_type=demodulation_type, ted_type=ted_type
        )  # From IQ samples to hard decisions
        received_packets: list[dict] = self.process_phy_packet(
            bit_samples, CRC_included=CRC_included, preamble_threshold=preamble_threshold
        )  # From hard decisions to packets

        return received_packets

    @property
    def transmission_rate(self) -> float:
        return self._transmission_rate

    @transmission_rate.setter
    def transmission_rate(self, rate: float) -> None:
        self._transmission_rate = rate
        self._fsk_deviation = self.transmission_rate * 0.25
        self._sps = int(self._sample_rate / self.transmission_rate)  # Samples per chip
