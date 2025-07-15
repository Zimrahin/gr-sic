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
from .utils.transmitters import Transmitter802154
from .utils.packet_utils import triangular_wave


class ieee802154_packet_source(gr.sync_block):
    """
    This block generates IEEE 802.15.4 packets from tagged triggers using precomputed modulated IQ waveforms.
    The purpose of using a sync block is to allow for parallel and synchronous transmission of multiple
    packets acrosss various blocks.
    """

    def __init__(self, sample_rate: float, payload_length: int, append_crc: bool):
        gr.sync_block.__init__(
            self,
            name="ieee802154_packet_source",
            in_sig=[np.complex64],
            out_sig=[np.complex64],
        )

        # Parameters
        self.max_payload_length = 127
        self.sample_rate = sample_rate
        self.payload_template = triangular_wave(step=4, length=self.max_payload_length)
        self.mutex = threading.Lock()
        self.append_crc = append_crc
        self.transmitter = Transmitter802154(sample_rate)

        # Changeable at runtime
        self.current_payload_length = payload_length

        # State variables
        self.transmitting = False
        self.samples_remaining = 0
        self.waveform_offset = 0  # Where in the waveform we are currently transmitting

        self.waveform = self.generate_waveform(payload_length)

    def generate_waveform(self, payload_length: int) -> np.ndarray:
        """Generate IQ waveform using current parameters"""
        constrained_length = max(0, min(payload_length, self.max_payload_length))
        payload_segment = self.payload_template[:constrained_length]
        return self.transmitter.modulate_from_payload(payload_segment, self.append_crc)

    def set_payload_length(self, length: int):
        """Update payload length and regenerate waveform"""
        new_length = int(length)
        new_waveform = self.generate_waveform(new_length)
        with self.mutex:
            self.current_payload_length = new_length
            self.waveform = new_waveform

    def start_burst(self):
        """Start new burst using current parameters"""
        with self.mutex:
            if len(self.waveform) == 0:
                self.transmitting = False
                return
            self.transmitting = True
            self.samples_remaining = len(self.waveform)
            self.waveform_offset = 0

    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        noutput_items = len(out)
        idx = 0

        while idx < noutput_items:
            if self.transmitting:
                # Copy precomputed waveform in chunks
                chunk = min(noutput_items - idx, self.samples_remaining)
                out[idx : idx + chunk] = self.waveform[self.waveform_offset : self.waveform_offset + chunk]
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
