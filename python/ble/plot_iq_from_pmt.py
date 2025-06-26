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
import matplotlib
from gnuradio import gr
import threading
import queue
import time

matplotlib.rcParams["toolbar"] = "none"  # Disable the toolbar globally


class plot_iq_from_pmt(gr.sync_block):
    """
    Plot IQ data from PMT messages.
    This block receives PMT messages containing IQ data, processes them in a
    separate thread, and updates a matplotlib plot with the results.
    The block is designed to handle heavy computation without blocking the
    main thread, using a thread-safe queue to manage incoming messages.
    Parameters:
    - sample_rate: The sample rate of the incoming data (default: 1).
    - max_queue_size: Maximum size of the queue for incoming messages (default: 5).
    Usage:
    Connect this block to a source that sends PMT messages containing IQ data.
    The messages should have a structure where the first element is a dictionary
    with metadata (including "Packet ID") and the second element is a c32vector
    containing the IQ samples.
    """

    def __init__(self, sample_rate: int = 1, max_queue_size: int = 5):
        gr.sync_block.__init__(self, name="plot_iq_from_pmt", in_sig=None, out_sig=None)

        # Parameters
        self.sample_rate = sample_rate if sample_rate == 1 else sample_rate / 1e6  # µs
        self.max_queue_size = max_queue_size

        # PMT IQ input port
        self.message_port_register_in(gr.pmt.intern("IQ"))
        self.set_msg_handler(gr.pmt.intern("IQ"), self.handle_iq_message)

        # Queue for storing IQ packets
        self.queue = queue.Queue(maxsize=self.max_queue_size)
        self.processing_thread = threading.Thread(target=self.process_queue, daemon=True)
        self.processing_thread.start()

        # Initialise plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Amplitude")
        self.ax.grid(True)
        self.ax.set_title("Waiting for data...")
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.show()

    def handle_iq_message(self, msg):
        """Queue incoming IQ arrays for processing."""
        try:
            if self.queue.full():
                print("Warning: Queue full - dropping message")
            else:
                self.queue.put_nowait(msg)
        except Exception as e:
            print(f"handle_iq_message(): {e}")

    def process_queue(self):
        """Process messages from queue in background thread."""
        while True:
            try:
                msg = self.queue.get()
                self.process_message(msg)
            except Exception as e:
                print(f"process_queue(): {e}")

    def process_message(self, msg):
        """Process message (heavy computation + plotting)."""
        try:
            # Parse PMT message
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)
            packet_id = gr.pmt.to_long(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))

            if not gr.pmt.is_c32vector(data):
                return
            iq = np.array(gr.pmt.c32vector_elements(data))

            # Successive Interference Cancellation will go here
            time.sleep(2)

            # Plot IQ data
            real = np.real(iq)
            imag = np.imag(iq)
            time_axis = np.arange(len(real)) / self.sample_rate
            max_amp = max(np.max(np.abs(real)), np.max(np.abs(imag))) * 1.1

            self.ax.clear()
            self.ax.plot(time_axis, real, "b-", label="I (In-phase)", alpha=0.7)
            self.ax.plot(time_axis, imag, "r-", label="Q (Quadrature)", alpha=0.7)
            self.ax.legend(loc="upper right")
            self.ax.set_xlabel("Time (µs)" if self.sample_rate != 1 else "Samples")
            self.ax.set_ylabel("Amplitude")
            self.ax.set_title(f"Processed Packet {packet_id}")
            self.ax.set_ylim(-max_amp, max_amp)
            self.ax.set_xlim(time_axis[0], time_axis[-1])
            self.ax.axhline(0, color="k", linestyle="--", alpha=0.3)

            # GUI update
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception as e:
            print(f"Message processing failed: {e}")

    def work(self, input_items, output_items):
        return len(input_items[0])  # No stream processing
