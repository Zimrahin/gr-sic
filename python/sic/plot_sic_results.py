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
from collections import OrderedDict

matplotlib.rcParams["toolbar"] = "none"  # Disable the toolbar globally


class plot_sic_results(gr.basic_block):
    """
    Plot IQ data from PMT messages.
    This block receives PMT messages containing IQ data, processes them in a
    separate thread, and updates a matplotlib plot with the results.
    Parameters:
    - sample_rate: The sample rate of the incoming data.
    Usage:
    Connect this block to a Successive Interference Cancellation (SIC) block
    that outputs PMT messages with SIC results.
    """

    def __init__(self, sample_rate: float, max_queue_size: int) -> None:
        gr.basic_block.__init__(self, name="plot_sic_results", in_sig=None, out_sig=None)
        self.sample_rate = sample_rate
        self.max_queue_size = max_queue_size

        # PMT input port
        self.message_port_register_in(gr.pmt.intern("in"))
        self.set_msg_handler(gr.pmt.intern("in"), self.handle_plot_message)

        # Plotting queue
        self.plot_queue = queue.Queue(maxsize=max_queue_size)
        self.plot_thread = threading.Thread(target=self.process_plot_queue, daemon=True)
        self.plot_thread.start()

        # Initialise plot
        plt.ion()
        self.fig, self.ax_grid = plt.subplots(2, 2)
        self.fig.suptitle("Waiting for data...", fontsize=14)
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.show()

    def handle_plot_message(self, msg: gr.pmt) -> None:
        try:
            if self.plot_queue.full():
                self.plot_queue.get_nowait()
            self.plot_queue.put_nowait(msg)
        except Exception as e:
            print(f"Plot message handling error: {e}")

    def process_plot_queue(self) -> None:
        while True:
            msg = self.plot_queue.get(block=True)
            try:
                meta = gr.pmt.car(msg)
                data_vector = gr.pmt.cdr(msg)

                packet_id = gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL)
                title = f"Packet {packet_id}"
                iq_before = np.array(gr.pmt.c32vector_elements(gr.pmt.vector_ref(data_vector, 0)))
                iq_after = np.array(gr.pmt.c32vector_elements(gr.pmt.vector_ref(data_vector, 1)))
                payload_before = bytes(gr.pmt.u8vector_elements(gr.pmt.vector_ref(data_vector, 2)))
                payload_after = bytes(gr.pmt.u8vector_elements(gr.pmt.vector_ref(data_vector, 3)))

                self.plot_results(iq_before, iq_after, payload_before, payload_after, title)

            except Exception as e:
                print(f"Error processing plot message: {e}")
                continue

    def plot_results(
        self,
        iq_before: np.ndarray,
        iq_after: np.ndarray,
        payload_before: bytes,
        payload_after: bytes,
        title: str,
    ) -> None:
        """Update the plot with the SIC results."""
        if not self.fig:
            return
        try:
            # Convert payloads to integer arrays
            payload_before_ints: np.ndarray = np.frombuffer(payload_before, dtype=np.uint8)
            payload_after_ints: np.ndarray = np.frombuffer(payload_after, dtype=np.uint8)
            plot_sample_rate = self.sample_rate if self.sample_rate == 1 else int(self.sample_rate / 1e6)

            # --- Top Left: IQ Before ---
            ax = self.ax_grid[0, 0]
            ax.clear()
            time_axis = np.arange(len(iq_before)) / plot_sample_rate
            ax.plot(time_axis, np.real(iq_before), "b-", label="I (In-phase)", alpha=0.7)
            ax.plot(time_axis, np.imag(iq_before), "r-", label="Q (Quadrature)", alpha=0.7)
            ax.set_xlabel("Time (µs)" if plot_sample_rate != 1 else "Samples")
            ax.set_ylabel("Amplitude")
            ax.set_title("IQ (Before SIC)")
            ax.legend(loc="upper right")
            ax.axhline(0, color="k", linestyle="--", alpha=0.3)

            # --- Bottom Left: IQ After ---
            ax = self.ax_grid[1, 0]
            ax.clear()
            time_axis = np.arange(len(iq_after)) / plot_sample_rate
            ax.plot(time_axis, np.real(iq_after), "b-", label="I (In-phase)", alpha=0.7)
            ax.plot(time_axis, np.imag(iq_after), "r-", label="Q (Quadrature)", alpha=0.7)
            ax.set_xlabel("Time (µs)" if plot_sample_rate != 1 else "Samples")
            ax.set_ylabel("Amplitude")
            ax.set_title("IQ (After SIC)")
            ax.legend(loc="upper right")
            ax.axhline(0, color="k", linestyle="--", alpha=0.3)

            # --- Top Right: Payload Before ---
            ax = self.ax_grid[0, 1]
            ax.clear()
            ax.plot(payload_before_ints, "ro-", markersize=4, alpha=0.9)
            ax.set_xlabel("Byte Index")
            ax.set_ylabel("Value (0-255)")
            ax.set_title(f"Payload (Before SIC)")
            ax.set_ylim(-5, 260)
            ax.set_yticks(np.arange(0, 256, 32))
            ax.grid(True)

            # --- Bottom Right: Payload After ---
            ax = self.ax_grid[1, 1]
            ax.clear()
            ax.plot(payload_after_ints, "bo-", markersize=4, alpha=0.9)
            ax.set_xlabel("Byte Index")
            ax.set_ylabel("Value (0-255)")
            ax.set_title(f"Payload (After SIC)")
            ax.set_ylim(-5, 260)
            ax.set_yticks(np.arange(0, 256, 32))
            ax.grid(True)

            self.fig.suptitle(title, fontsize=14)

            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception as e:
            print(f"plot_results() failed: {e}")
