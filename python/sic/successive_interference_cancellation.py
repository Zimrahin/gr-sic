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
import queue
import time
from collections import OrderedDict


class successive_interference_cancellation(gr.basic_block):
    """
    Applies Successive Interference Cancellation (SIC) to incoming IQ data,
    assuming it contains a mixed IQ signal with overlapping BLE and IEEE 802.15.4
    transmissions.
    The SIC processing is done in a separate thread to avoid blocking the main
    flowgraph, using a thread-safe queue to manage incoming messages.
    Parameters:
    - sample_rate: The sample rate of the incoming data.
    - max_queue_size: Maximum size of the queue for incoming messages.
    - protocol_high: The protocol for the stronger signal that is demodulated first.
    - protocol_low: The protocol for the weaker signal that is demodulated last.
    - frequency_start: Exhaustive search start offset frequency.
    - frequency_stop: Exhaustive search stop offset frequency.
    - frequency_step_coarse: Coarse frequency step for the exhaustive search.
    - frequency_step_fine: Fine frequency step for the exhaustive search.
    """

    def __init__(
        self,
        sample_rate: float,
        max_queue_size: int,
        protocol_high: int,
        protocol_low: int,
        frequency_start: float,
        frequency_stop: float,
        frequency_step_coarse: float,
        frequency_step_fine: float,
        frequency_fine_window: float,
        ble_transmission_rate: float,
    ):
        gr.basic_block.__init__(self, name="successive_interference_cancellation", in_sig=None, out_sig=None)
        self.sample_rate = sample_rate
        self.max_queue_size = max_queue_size

        self.message_port_register_in(gr.pmt.intern("iq"))
        self.set_msg_handler(gr.pmt.intern("iq"), self.handle_iq_message)
        self.message_port_register_in(gr.pmt.intern("pdu"))
        self.set_msg_handler(gr.pmt.intern("pdu"), self.handle_payload_message)
        self.message_port_register_out(gr.pmt.intern("out"))

        self.iq_queue = queue.Queue(maxsize=max_queue_size)
        self.payload_cache = OrderedDict()
        self.max_cache_size = max_queue_size
        self.mutex = threading.Lock()

        self.processing_thread = threading.Thread(target=self.process_iq_queue, daemon=True)
        self.processing_thread.start()

    def handle_iq_message(self, msg: gr.pmt) -> None:
        try:
            if self.iq_queue.full():
                self.iq_queue.get_nowait()
            self.iq_queue.put_nowait(msg)
        except Exception as e:
            print(f"handle_iq_message error: {e}")

    def handle_payload_message(self, msg: gr.pmt) -> None:
        try:
            meta = gr.pmt.car(msg)
            crc_ok = gr.pmt.to_bool(gr.pmt.dict_ref(meta, gr.pmt.intern("CRC check"), gr.pmt.PMT_NIL))
            if not crc_ok:
                return

            packet_id = gr.pmt.to_uint64(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))
            payload = bytes(gr.pmt.u8vector_elements(gr.pmt.cdr(msg)))

            with self.mutex:
                if len(self.payload_cache) >= self.max_cache_size:
                    self.payload_cache.popitem(last=False)
                self.payload_cache[packet_id] = payload
                self.payload_cache.move_to_end(packet_id)
        except Exception as e:
            print(f"handle_payload_message error: {e}")

    def process_iq_queue(self) -> None:
        while True:
            msg = self.iq_queue.get(block=True)
            self.heavy_processing(msg)

    def heavy_processing(self, msg: gr.pmt) -> None:
        try:
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)
            if not gr.pmt.is_c32vector(data):
                return
            packet_id = gr.pmt.to_uint64(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))
            iq_before = np.array(gr.pmt.c32vector_elements(data))

            # Get payload from cache
            with self.mutex:
                payload_before = self.payload_cache.pop(packet_id, None)
            if payload_before is None:
                return

            # Simulate heavy processing and placeholder outputs
            time.sleep(2)
            iq_after = iq_before
            payload_after = payload_before

            # Prepare output message
            meta_out = gr.pmt.make_dict()
            meta_out = gr.pmt.dict_add(meta_out, gr.pmt.intern("Packet ID"), gr.pmt.from_long(packet_id))
            data_out = gr.pmt.make_vector(4, gr.pmt.PMT_NIL)
            gr.pmt.vector_set(data_out, 0, gr.pmt.init_c32vector(len(iq_before), iq_before))
            gr.pmt.vector_set(data_out, 1, gr.pmt.init_c32vector(len(iq_after), iq_after))
            gr.pmt.vector_set(data_out, 2, gr.pmt.init_u8vector(len(payload_before), list(payload_before)))
            gr.pmt.vector_set(data_out, 3, gr.pmt.init_u8vector(len(payload_after), list(payload_after)))

            self.message_port_pub(gr.pmt.intern("out"), gr.pmt.cons(meta_out, data_out))

        except Exception as e:
            print(f"Heavy processing failed: {e}")
            if packet_id is not None:
                with self.mutex:
                    if packet_id in self.payload_cache:
                        del self.payload_cache[packet_id]

    def set_frequency_start(self, frequency_start: int):
        """Set the start frequency for exhaustive search."""
        with self.mutex:
            self.frequency_start = frequency_start

    def set_frequency_stop(self, frequency_stop: int):
        """Set the stop frequency for exhaustive search."""
        with self.mutex:
            self.frequency_stop = frequency_stop

    def set_frequency_step_coarse(self, frequency_step_coarse: int):
        """Set the coarse frequency step for exhaustive search."""
        with self.mutex:
            self.frequency_step_coarse = frequency_step_coarse

    def set_frequency_step_fine(self, frequency_step_fine: int):
        """Set the fine frequency step for exhaustive search."""
        with self.mutex:
            self.frequency_step_fine = frequency_step_fine
