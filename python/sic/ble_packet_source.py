#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
from gnuradio import gr
import threading


class ble_packet_source(gr.sync_block):
    """
    Adds constant amplitude to tagged complex stream with runtime changeable parameters.
    """

    def __init__(self, sample_rate: float, payload_length: int, amplitude: float):
        gr.sync_block.__init__(self, name="ble_packet_source", in_sig=[np.complex64], out_sig=[np.complex64])
        # Parameters
        self.sample_rate = sample_rate
        self.payload_length = payload_length
        self.amplitude = amplitude
        self.transmission_rate = 1e6  # FIX ME

        # State variables
        self.transmitting = False
        self.samples_remaining = 0
        self.current_amplitude = amplitude
        self.current_payload_length = payload_length

        self.param_mutex = threading.Lock()

    def start_burst(self):
        """Start new burst using current parameters"""
        with self.param_mutex:
            self.transmitting = True
            self.samples_remaining = int(self.payload_length * 8 * self.sample_rate / self.transmission_rate)
            self.current_amplitude = self.amplitude
            self.current_payload_length = self.payload_length

    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        noutput_items = len(out)
        idx = 0

        # Process entire buffer
        while idx < noutput_items:
            if self.transmitting:
                chunk = min(noutput_items - idx, self.samples_remaining)
                # Replace output with constant amplitude complex samples
                out[idx : idx + chunk] = complex(self.current_amplitude, 0)
                idx += chunk
                self.samples_remaining -= chunk

                if self.samples_remaining <= 0:
                    self.transmitting = False
            else:
                # Check for trigger tags in remaining buffer
                tags = self.get_tags_in_window(0, idx, noutput_items)
                if tags:
                    tag = tags[0]
                    tag_offset = tag.offset - self.nitems_read(0)

                    # Process samples before tag
                    if tag_offset > idx:
                        out[idx:tag_offset] = in0[idx:tag_offset]
                        idx = tag_offset

                    self.start_burst()
                else:
                    # No triggers - copy remaining input
                    out[idx:] = in0[idx:]
                    idx = noutput_items

        return noutput_items

    # Runtime parameter setters
    def set_payload_length(self, length):
        with self.param_mutex:
            self.payload_length = int(length)

    def set_amplitude(self, amplitude):
        with self.param_mutex:
            self.amplitude = float(amplitude)
