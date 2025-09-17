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
import scipy.signal
import threading
import queue
import time
from collections import OrderedDict
from .utils.receivers import ReceiverBLE, Receiver802154, Receiver
from .utils.transmitters import TransmitterBLE, Transmitter802154, Transmitter
from .utils.interference import subtract_interference_wrapper


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
        upsampling_factor: int,
        ble_transmission_rate: float,
        ieee802154_transmission_rate: float,  # Chip rate
    ):
        gr.basic_block.__init__(self, name="successive_interference_cancellation", in_sig=None, out_sig=None)
        self.upsampling_factor = upsampling_factor
        self.sample_rate = upsampling_factor * sample_rate
        self.max_queue_size = max_queue_size
        self.protocol_high = protocol_high
        self.protocol_low = protocol_low
        self.ble_transmission_rate = ble_transmission_rate
        self.ieee802154_transmission_rate = ieee802154_transmission_rate
        self.frequency_start = frequency_start
        self.frequency_stop = frequency_stop
        self.frequency_step_coarse = frequency_step_coarse
        self.frequency_step_fine = frequency_step_fine
        self.frequency_fine_window = frequency_fine_window
        self.protocol_map = {
            0: (TransmitterBLE, ReceiverBLE, True),
            1: (Transmitter802154, Receiver802154, False),
        }
        self.transmitter_high, self.receiver_high = self._init_protocol(protocol_high)
        self.transmitter_low, self.receiver_low = self._init_protocol(protocol_low)

        self.message_port_register_in(gr.pmt.intern("iq"))
        self.set_msg_handler(gr.pmt.intern("iq"), self.handle_iq_message)
        self.message_port_register_in(gr.pmt.intern("pdu"))
        self.set_msg_handler(gr.pmt.intern("pdu"), self.handle_payload_message)
        self.message_port_register_out(gr.pmt.intern("out"))

        # Queue and cache to store IQ arrays and payloads
        self.iq_queue = queue.Queue(maxsize=max_queue_size)
        self.payload_cache = OrderedDict()
        self.max_cache_size = max_queue_size
        self.mutex = threading.Lock()

        self.processing_thread = threading.Thread(target=self.process_iq_queue, daemon=True)
        self.processing_thread.start()

    def _init_protocol(self, protocol_id: int) -> tuple[Transmitter, Receiver]:
        if protocol_id not in self.protocol_map:
            raise ValueError(f"Invalid protocol ID: {protocol_id}")
        tx_class, rx_class, is_ble = self.protocol_map[protocol_id]

        if is_ble:
            transmitter = tx_class(self.sample_rate, self.ble_transmission_rate)
            receiver = rx_class(self.sample_rate, self.ble_transmission_rate)
        else:
            transmitter = tx_class(self.sample_rate, self.ieee802154_transmission_rate)
            receiver = rx_class(self.sample_rate, self.ieee802154_transmission_rate)

        return transmitter, receiver

    def handle_iq_message(self, msg: gr.pmt) -> None:
        try:
            if self.iq_queue.full():
                self.iq_queue.get_nowait()
            self.iq_queue.put_nowait(msg)
        except Exception as e:
            print(f"handle_iq_message failed: {e}")

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
            print(f"handle_payload_message failed: {e}")

    def process_iq_queue(self) -> None:
        while True:
            try:
                msg = self.iq_queue.get(block=True)
                self.process_iq_message(msg)
            except Exception as e:
                print(f"process_iq_queue thread failed: {e}")
                time.sleep(0.1)

    def process_iq_message(self, msg: gr.pmt) -> None:
        try:
            packet_id = None
            meta = gr.pmt.car(msg)
            data = gr.pmt.cdr(msg)
            if not gr.pmt.is_c32vector(data):
                return
            packet_id = gr.pmt.to_uint64(gr.pmt.dict_ref(meta, gr.pmt.intern("Packet ID"), gr.pmt.PMT_NIL))
            iq_before = np.array(gr.pmt.c32vector_elements(data))

            # Get payload from cache (obtained from pdu message input )
            with self.mutex:
                payload_high: bytes = self.payload_cache.pop(packet_id, None)
            if payload_high is None:
                return

            # Upsample to target sample rate
            if self.upsampling_factor > 1:
                target_length = len(iq_before) * self.upsampling_factor
                iq_before = scipy.signal.resample(iq_before, target_length)

            # Successive Interference Cancellation
            try:
                received_packet_low: dict = self.receiver_low.demodulate_to_packet(iq_before)[0]
            except Exception:
                received_packet_low = []  # Treat any exception as no packet received successfully
            payload_low_before = received_packet_low["payload"] if received_packet_low else []

            payload_low_after, iq_after = self.sic_implementation(
                self.sample_rate,
                iq_before,
                self.receiver_high,
                self.transmitter_high,
                self.receiver_low,
                self.frequency_start,
                self.frequency_stop,
                self.frequency_step_coarse,
                self.frequency_step_fine,
                self.frequency_fine_window,
                payload_high=np.frombuffer(payload_high, dtype=np.uint8),
            )

            # Downsample
            if self.upsampling_factor > 1:
                iq_after = scipy.signal.resample(iq_after, int(len(iq_after) / self.upsampling_factor))
                iq_before = scipy.signal.resample(iq_before, int(len(iq_before) / self.upsampling_factor))

            # Prepare output message
            meta_out = gr.pmt.make_dict()
            meta_out = gr.pmt.dict_add(meta_out, gr.pmt.intern("Packet ID"), gr.pmt.from_long(packet_id))
            data_out = gr.pmt.make_vector(4, gr.pmt.PMT_NIL)
            gr.pmt.vector_set(data_out, 0, gr.pmt.init_c32vector(len(iq_before), iq_before))
            gr.pmt.vector_set(data_out, 1, gr.pmt.init_c32vector(len(iq_after), iq_after))
            gr.pmt.vector_set(data_out, 2, gr.pmt.init_u8vector(len(payload_low_before), list(payload_low_before)))
            gr.pmt.vector_set(data_out, 3, gr.pmt.init_u8vector(len(payload_low_after), list(payload_low_after)))

            self.message_port_pub(gr.pmt.intern("out"), gr.pmt.cons(meta_out, data_out))

        except Exception as e:
            print(f"process_iq_message failed: {e}")
            if packet_id is not None:
                with self.mutex:
                    if packet_id in self.payload_cache:
                        del self.payload_cache[packet_id]

    def sic_implementation(
        self,
        sample_rate: float,
        mixed_iq_signal: np.ndarray,
        receiver_high: Receiver,  # Receiver for the stronger signal
        transmitter_high: Transmitter,  # Transmitter for the stronger signal
        receiver_low: Receiver,  # Receiver for the weaker signal
        frequency_start: float,
        frequency_stop: float,
        frequency_step_coarse: float,
        frequency_step_fine: float,
        frequency_fine_window: float,
        *,
        payload_high: np.ndarray = None,
        verbose: bool = False,
    ) -> tuple:
        """
        Perform successive interference cancellation:
        1. Demodulate stronger signal
        2. Synthesise stronger signal
        3. Subtract stronger signal from mixed signal
        4. Demodulate weaker signal from residual
        """
        # 1. Demodulate stronger signal if not given
        if payload_high is None:
            try:
                received_packet_high: dict = receiver_high.demodulate_to_packet(mixed_iq_signal)[0]
                payload_high = received_packet_high["payload"]
                if not received_packet_high["crc_check"]:
                    return [], mixed_iq_signal
            except Exception:
                return [], mixed_iq_signal

        # 2. Synthesise stronger signal
        synthesised_high = transmitter_high.modulate_from_payload(payload_high)

        # 3. Subtract stronger signal from mixed signal
        subtracted_iq_signal = subtract_interference_wrapper(
            affected=mixed_iq_signal,
            interference=synthesised_high,
            sample_rate=sample_rate,
            freq_offsets=np.arange(frequency_start, frequency_stop, frequency_step_coarse),
            fine_step=frequency_step_fine,
            fine_window=frequency_fine_window,
            verbose=verbose,
        )

        # 4. Demodulate weaker signal from residual
        try:
            received_packet_low: dict = receiver_low.demodulate_to_packet(subtracted_iq_signal)[0]
            payload_low = received_packet_low["payload"]
        except Exception:
            return [], subtracted_iq_signal

        return payload_low, subtracted_iq_signal

    def set_frequency_start(self, frequency_start: float):
        with self.mutex:
            self.frequency_start = frequency_start

    def set_frequency_stop(self, frequency_stop: float):
        with self.mutex:
            self.frequency_stop = frequency_stop

    def set_frequency_step_coarse(self, frequency_step_coarse: float):
        with self.mutex:
            self.frequency_step_coarse = frequency_step_coarse

    def set_frequency_step_fine(self, frequency_step_fine: float):
        with self.mutex:
            self.frequency_step_fine = frequency_step_fine

    def set_ble_transmission_rate(self, rate: float):
        with self.mutex:
            self.ble_transmission_rate = rate
            self.transmitter_high, self.receiver_high = self._init_protocol(self.protocol_high)
            self.transmitter_low, self.receiver_low = self._init_protocol(self.protocol_low)

    def set_ieee802154_transmission_rate(self, rate: float):
        with self.mutex:
            self.ieee802154_transmission_rate = rate
            self.transmitter_high, self.receiver_high = self._init_protocol(self.protocol_high)
            self.transmitter_low, self.receiver_low = self._init_protocol(self.protocol_low)
