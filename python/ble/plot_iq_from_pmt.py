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
    - sample_rate: The sample rate of the incoming data (default: 1).
    - max_queue_size: Maximum size of the queue for incoming messages (default: 5).
    Usage:
    Connect this block to a source that sends PMT messages containing IQ data.
    The messages should have a structure where the first element is a dictionary
    with metadata (including "Packet ID") and the second element is a c32vector
    containing the IQ samples.
    """

    def __init__(self, sample_rate: int = 1, max_queue_size: int = 5) -> None:
        gr.sync_block.__init__(self, name="plot_iq_from_pmt", in_sig=None, out_sig=None)

        # Parameters
        self.sample_rate = sample_rate if sample_rate == 1 else int(sample_rate / 1e6)  # µs
        self.max_queue_size = max_queue_size

        # PMT input ports
        self.message_port_register_in(gr.pmt.intern("iq"))
        self.set_msg_handler(gr.pmt.intern("iq"), self.handle_iq_message)
        self.message_port_register_in(gr.pmt.intern("pdu"))
        self.set_msg_handler(gr.pmt.intern("pdu"), self.handle_payload_message)

        # Queue for IQ packets
        self.iq_queue = queue.Queue(maxsize=self.max_queue_size)
        self.processing_thread = threading.Thread(target=self.process_iq_queue, daemon=True)
        self.processing_thread.start()

        # Ordered dictionary for payload metadata
        self.max_cache_size = max_queue_size
        self.payload_cache: OrderedDict[int, bytes] = OrderedDict()
        self.cache_lock = threading.Lock()

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
            crc_ok: bool = gr.pmt.to_bool(gr.pmt.dict_ref(meta, gr.pmt.intern("CRC check"), gr.pmt.PMT_NIL))
            if not crc_ok:
                return
            packet_id: int = gr.pmt.to_uint64(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))
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
            msg: gr.pmt = self.iq_queue.get(block=True, timeout=None)  # Block until available
            self.heavy_processing(msg)

    def heavy_processing(self, msg: gr.pmt) -> None:
        """Process IQ message and correlate with payload."""
        try:
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)
            if not gr.pmt.is_c32vector(data):
                return
            packet_id: int = gr.pmt.to_uint64(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))
            iq: np.ndarray = np.array(gr.pmt.c32vector_elements(data))

            # Retrieve payload from cache
            payload: bytes = None
            with self.cache_lock:
                payload = self.payload_cache.pop(packet_id, None)  # Remove if found
            if payload is None:
                return

            # Simulate heavy processing
            time.sleep(2)

            # Prepare title with payload info
            payload_hex: str = payload.hex()
            if len(payload_hex) > 10:  # Truncate long payloads
                payload_hex = payload_hex[:20] + "..."
            title = f"Packet {packet_id} (Payload: {payload_hex})"

            # Generate plot data
            real: np.ndarray = np.real(iq)
            imag: np.ndarray = np.imag(iq)
            time_axis: np.ndarray = np.arange(len(real)) / self.sample_rate
            max_amp: float = max(np.max(np.abs(real)), np.max(np.abs(imag))) * 1.1

            # Update plot
            self.ax.clear()
            self.ax.plot(time_axis, real, "b-", label="I (In-phase)", alpha=0.7)
            self.ax.plot(time_axis, imag, "r-", label="Q (Quadrature)", alpha=0.7)
            self.ax.legend(loc="upper right")
            self.ax.set_xlabel("Time (µs)" if self.sample_rate != 1 else "Samples")
            self.ax.set_ylabel("Amplitude")
            self.ax.set_title(title)
            self.ax.set_ylim(-max_amp, max_amp)
            self.ax.set_xlim(time_axis[0], time_axis[-1])
            self.ax.axhline(0, color="k", linestyle="--", alpha=0.3)

            # GUI update
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

        except Exception as e:
            if packet_id is not None:
                with self.cache_lock:
                    if packet_id in self.payload_cache:
                        del self.payload_cache[packet_id]
            print(f"Message processing failed: {e}")

    def work(self, input_items, output_items) -> int:
        return len(input_items[0])  # No stream processing
