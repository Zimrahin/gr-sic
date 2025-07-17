#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
import threading
from gnuradio import gr
from .utils.transmitters import TransmitterBLE
from .utils.packet_utils import triangular_wave


class ble_packet_source(gr.sync_block):
    """
    This block generates BLE packets from tagged triggers using precomputed modulated IQ waveforms.
    The purpose of using a sync block is to allow for parallel and synchronous transmission of multiple
    packets acrosss various blocks.
    """

    def __init__(
        self,
        sample_rate: float,
        payload_length: int,
        transmission_rate: float,
        base_address: int,
    ):
        gr.sync_block.__init__(
            self,
            name="ble_packet_source",
            in_sig=[np.complex64],
            out_sig=[np.complex64],
        )
        # Parameters
        self.max_payload_length = 255
        self.sample_rate = sample_rate
        self.payload_template = triangular_wave(step=2, length=self.max_payload_length)
        self.base_address = base_address
        self.mutex = threading.Lock()

        # Changeable at runtime
        self.transmitter = TransmitterBLE(sample_rate, transmission_rate)
        self.current_payload_length = payload_length

        # State variables
        self.transmitting = False
        self.active_waveform = np.array([], dtype=np.complex64)
        self.active_remaining = 0
        self.active_offset = 0  # Where in the waveform we are currently transmitting

        self.waveform = self.generate_waveform(payload_length, transmission_rate)

    def generate_waveform(self, payload_length: int, transmission_rate: float) -> np.ndarray:
        constrained_length = max(0, min(payload_length, self.max_payload_length))
        payload_segment = self.payload_template[:constrained_length]
        self.transmitter.transmission_rate = transmission_rate
        return self.transmitter.modulate_from_payload(payload_segment, self.base_address)

    def set_payload_length(self, length: int):
        new_length = int(length)
        new_waveform = self.generate_waveform(new_length, self.transmitter.transmission_rate)
        with self.mutex:
            self.current_payload_length = new_length
            self.waveform = new_waveform

    def set_transmission_rate(self, transmission_rate: float):
        new_waveform = self.generate_waveform(self.current_payload_length, transmission_rate)
        with self.mutex:
            self.transmitter.transmission_rate = transmission_rate
            self.waveform = new_waveform

    def start_burst(self):
        with self.mutex:
            if len(self.waveform) == 0:
                self.transmitting = False
                return
            self.transmitting = True
            # Capture current waveform for the entire burst
            self.active_waveform = self.waveform
            self.active_remaining = len(self.waveform)
            self.active_offset = 0

    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        noutput_items = len(out)
        idx = 0

        while idx < noutput_items:
            if self.transmitting:
                # Use burst-specific waveform copy
                chunk = min(noutput_items - idx, self.active_remaining)
                out[idx : idx + chunk] = self.active_waveform[self.active_offset : self.active_offset + chunk]
                idx += chunk
                self.active_offset += chunk
                self.active_remaining -= chunk

                if self.active_remaining <= 0:
                    self.transmitting = False
            else:
                # Check for triggers
                tags = self.get_tags_in_window(0, idx, noutput_items)
                if not tags:
                    out[idx:] = in0[idx:]
                    return noutput_items

                tag = tags[0]
                tag_offset = tag.offset - self.nitems_read(0)

                # Copy samples before trigger
                if tag_offset > idx:
                    out[idx:tag_offset] = in0[idx:tag_offset]
                    idx = tag_offset

                self.start_burst()  # Start new burst at trigger position

        return noutput_items
