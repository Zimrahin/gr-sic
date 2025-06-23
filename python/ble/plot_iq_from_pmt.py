#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.

# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
import matplotlib.pyplot as plt
from gnuradio import gr


class plot_iq_from_pmt(gr.sync_block):
    """
    Plot IQ data from PMT messages.
    """

    def __init__(self, sample_rate=1.0):
        gr.sync_block.__init__(self, name="plot_iq_from_pmt", in_sig=None, out_sig=None)

        # Parameters
        self.sample_rate = sample_rate
        print(f"{sample_rate = } Hz")

        # PMT message input port
        self.message_port_register_in(gr.pmt.intern("in"))
        self.set_msg_handler(gr.pmt.intern("in"), self.handle_message)

    def handle_message(self, msg):
        """
        Handle incoming PMT messages containing IQ data.
        """
        try:
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)

            # Get packet ID from metadata
            packet_id = gr.pmt.to_long(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))
            print(f"Received packet ID: {packet_id}")

            # Convert PMT vector to numpy array
            if gr.pmt.is_c32vector(data):
                iq = np.array(gr.pmt.c32vector_elements(data))
                print(f"{len(iq)} complex samples received")
            else:
                print("Unsupported data type")
                return

        except Exception as e:
            print(f"Error processing message: {e}")
            return

    def work(self, input_items, output_items):
        # No stream processing, just message handling
        return len(input_items[0])
