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


class plot_iq_from_pmt(gr.sync_block):
    """
    Plot IQ data from PMT messages.
    This block receives PMT messages containing IQ data, processes them in a
    separate thread, and updates a matplotlib plot with the results.
    The block is designed to handle heavy computation without blocking the
    main thread, using a thread-safe queue to manage incoming messages.
    Parameters:
    - sample_rate: The sample rate of the incoming data.
    - max_queue_size: Maximum size of the queue for incoming messages.
    Usage:
    Connect this block to a source that sends PMT messages containing IQ data.
    The messages should have a structure where the first element is a dictionary
    with metadata (including "Packet ID") and the second element is a c32vector
    containing the IQ samples.
    """

    def __init__(self, sample_rate: float, max_queue_size: int) -> None:
        gr.sync_block.__init__(self, name="plot_iq_from_pmt", in_sig=None, out_sig=None)

        # Parameters
        self.sample_rate = (
            sample_rate if sample_rate == 1 else int(sample_rate / 1e6)
        )  # µs
        self.max_queue_size = max_queue_size

        # PMT input ports
        self.message_port_register_in(gr.pmt.intern("iq"))
        self.set_msg_handler(gr.pmt.intern("iq"), self.handle_iq_message)
        self.message_port_register_in(gr.pmt.intern("pdu"))
        self.set_msg_handler(gr.pmt.intern("pdu"), self.handle_payload_message)

        # Queue for IQ packets
        self.iq_queue = queue.Queue(maxsize=self.max_queue_size)
        self.processing_thread = threading.Thread(
            target=self.process_iq_queue, daemon=True
        )
        self.processing_thread.start()

        # Ordered dictionary for payload metadata
        self.max_cache_size = max_queue_size
        self.payload_cache: OrderedDict[int, bytes] = OrderedDict()
        self.cache_lock = threading.Lock()

        # Initialise plot
        plt.ion()
        self.fig, self.ax_grid = plt.subplots(2, 2)
        self.fig.suptitle("Waiting for data...", fontsize=14)
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.show()

    def handle_iq_message(self, msg: gr.pmt) -> None:
        """Queue incoming IQ messages."""
        try:
            if self.iq_queue.full():
                self.iq_queue.get_nowait()  # Discard oldest message
            self.iq_queue.put_nowait(msg)
        except Exception as e:
            print(f"handle_iq_message(): {e}")

    def handle_payload_message(self, msg: gr.pmt) -> None:
        """Store payloads in ordered dictionary."""
        try:
            meta = gr.pmt.car(msg)
            crc_ok: bool = gr.pmt.to_bool(
                gr.pmt.dict_ref(meta, gr.pmt.intern("CRC check"), gr.pmt.PMT_NIL)
            )
            if not crc_ok:
                return
            packet_id: int = gr.pmt.to_uint64(
                gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL)
            )
            payload: bytes = bytes(gr.pmt.u8vector_elements(gr.pmt.cdr(msg)))

            with self.cache_lock:
                if len(self.payload_cache) >= self.max_cache_size:
                    self.payload_cache.popitem(last=False)  # Remove oldest item
                self.payload_cache[packet_id] = payload
                self.payload_cache.move_to_end(packet_id)

        except Exception as e:
            print(f"handle_payload_meta(): {e}")

    def process_iq_queue(self) -> None:
        """Process messages from queue in background thread."""
        while True:
            msg: gr.pmt = self.iq_queue.get(
                block=True, timeout=None
            )  # Block until available
            self.heavy_processing(msg)

    def heavy_processing(self, msg: gr.pmt) -> None:
        """Process IQ message and correlate with payload."""
        try:
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)
            if not gr.pmt.is_c32vector(data):
                return
            packet_id: int = gr.pmt.to_uint64(
                gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL)
            )
            iq_mixed: np.ndarray = np.array(gr.pmt.c32vector_elements(data))

            payload: bytes = None
            with self.cache_lock:
                payload = self.payload_cache.pop(packet_id, None)  # Remove if found
            if payload is None:
                return

            # Simulate heavy processing
            time.sleep(2)

            title = f"Packet {packet_id}"
            self.plot_results(iq_mixed, iq_mixed, payload, payload, title)

        except Exception as e:
            if packet_id is not None:
                with self.cache_lock:
                    if packet_id in self.payload_cache:
                        del self.payload_cache[packet_id]
            print(f"Message processing failed: {e}")

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
            payload_before_ints: np.ndarray = np.frombuffer(
                payload_before, dtype=np.uint8
            )
            payload_after_ints: np.ndarray = np.frombuffer(payload_after, dtype=np.uint8)

            # --- Top Left: IQ Before ---
            ax = self.ax_grid[0, 0]
            ax.clear()
            time_axis = np.arange(len(iq_before)) / self.sample_rate
            ax.plot(time_axis, np.real(iq_before), "b-", label="I (In-phase)", alpha=0.7)
            ax.plot(
                time_axis, np.imag(iq_before), "r-", label="Q (Quadrature)", alpha=0.7
            )
            ax.set_xlabel("Time (µs)" if self.sample_rate != 1 else "Samples")
            ax.set_ylabel("Amplitude")
            ax.set_title("IQ (Before SIC)")
            ax.legend(loc="upper right")
            ax.axhline(0, color="k", linestyle="--", alpha=0.3)

            # --- Bottom Left: IQ After ---
            ax = self.ax_grid[1, 0]
            ax.clear()
            time_axis = np.arange(len(iq_after)) / self.sample_rate
            ax.plot(time_axis, np.real(iq_after), "b-", label="I (In-phase)", alpha=0.7)
            ax.plot(time_axis, np.imag(iq_after), "r-", label="Q (Quadrature)", alpha=0.7)
            ax.set_xlabel("Time (µs)" if self.sample_rate != 1 else "Samples")
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
