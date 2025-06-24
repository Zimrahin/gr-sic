#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
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

        # PMT message input port
        self.message_port_register_in(gr.pmt.intern("in"))
        self.set_msg_handler(gr.pmt.intern("in"), self.handle_message)

        # Initialise plot
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots()
        self.sample_rate = sample_rate
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Amplitude")
        self.ax.grid(True)
        self.ax.set_title("Waiting for data...")
        self.ax.legend()
        self.fig.tight_layout()
        self.fig.show()

    def handle_message(self, msg):
        """
        Handle incoming PMT messages containing IQ data.
        """
        try:
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)

            # Get packet ID from metadata
            packet_id = gr.pmt.to_long(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))

            # Convert PMT vector to numpy array
            if gr.pmt.is_c32vector(data):
                iq: np.ndarray = np.array(gr.pmt.c32vector_elements(data))
            else:
                print("Plot IQ from PMT: Unsupported data type")
                return

        except Exception as e:
            print(f"Plot IQ from PMT: Error processing message: {e}")
            return

        # Prepare data for plotting
        real = np.real(iq)
        imag = np.imag(iq)
        time = np.arange(len(real)) / self.sample_rate

        # Update plot
        self.ax.clear()

        # Plot I and Q components
        self.ax.plot(time, real, "b-", label="I (In-phase)", alpha=0.7)
        self.ax.plot(time, imag, "r-", label="Q (Quadrature)", alpha=0.7)

        # Set labels and title
        self.ax.legend(loc="upper right")
        self.ax.set_xlabel("Time (seconds)" if self.sample_rate != 1.0 else "Samples")
        self.ax.set_ylabel("Amplitude")
        self.ax.grid(True)
        self.ax.set_title(f"Packet {packet_id} (Length: {len(real)} samples)")

        # Dynamic axis scaling
        max_amp = max(np.max(np.abs(real)), np.max(np.abs(imag))) * 1.1
        self.ax.set_ylim(-max_amp, max_amp)
        self.ax.set_xlim(time[0], time[-1])

        # Add zero reference line
        self.ax.axhline(0, color="k", linestyle="--", alpha=0.3)

        # Refresh plot without blocking
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def work(self, input_items, output_items):
        # No stream processing, just message handling
        return len(input_items[0])
