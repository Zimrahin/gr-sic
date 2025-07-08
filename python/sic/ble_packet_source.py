#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr
import numpy as np
import threading
from .utils.ble_transmitter import TransmitterBLE, triangular_wave


class ble_packet_source(gr.sync_block):
    """
    Generates BLE packets from tagged triggers using precomputed modulated IQ waveforms.
    """

    def __init__(self, sample_rate: float, payload_length: int, transmission_rate: float):
        gr.sync_block.__init__(
            self, name="ble_packet_source", in_sig=[np.complex64], out_sig=[np.complex64]
        )
        # Parameters
        self.max_payload_length = 255
        self.sample_rate = sample_rate
        self.transmission_rate = transmission_rate
        self.payload_template = triangular_wave(step=2, length=self.max_payload_length)
        self.transmitter = TransmitterBLE(sample_rate, transmission_rate)
        self.param_mutex = threading.Lock()

        self.waveform = np.array([], dtype=np.complex64)
        self.set_payload_length(payload_length)  # Initialises IQ waveform

        # State variables
        self.transmitting = False
        self.samples_remaining = 0
        self.waveform_offset = 0  # Where in the waveform we are currently transmitting

    def start_burst(self):
        """Start new burst using current parameters"""
        with self.param_mutex:
            if len(self.waveform) == 0:
                self.transmitting = False
                return
            self.transmitting = True
            self.samples_remaining = len(self.waveform)
            self.waveform_offset = 0

    def set_payload_length(self, length: int):
        """Update payload length and regenerate waveform"""
        constrained_length = max(0, min(int(length), self.max_payload_length))
        new_waveform = self.transmitter.modulate_from_payload(
            self.payload_template[:constrained_length]
        )
        with self.param_mutex:
            self.payload_length = constrained_length
            self.waveform = new_waveform

    def set_transmission_rate(self, transmission_rate: float):
        with self.param_mutex:
            self.transmission_rate = float(transmission_rate)

    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        noutput_items = len(out)
        idx = 0

        while idx < noutput_items:
            if self.transmitting:
                # Copy precomputed waveform in chunks
                chunk = min(noutput_items - idx, self.samples_remaining)
                out[idx : idx + chunk] = self.waveform[
                    self.waveform_offset : self.waveform_offset + chunk
                ]
                idx += chunk
                self.samples_remaining -= chunk

                if self.samples_remaining <= 0:
                    self.transmitting = False
            else:
                # Check for trigger tags in remaining buffer
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
