#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

from gnuradio import gr, blocks, filter, analog, digital, sic
from gnuradio.fft import window
import numpy as np


class ble_hier_rx(gr.hier_block2):
    """
    BLE receiver hier block
    """

    def __init__(self, sample_rate: float, symbol_rate: float, preamble_threshold: int, base_address: int):
        gr.hier_block2.__init__(
            self,
            "ble_hier_rx",
            gr.io_signature(1, 1, gr.sizeof_gr_complex),
            gr.io_signature(1, 1, gr.sizeof_char),
        )

        self._sample_rate = sample_rate
        self._symbol_rate = symbol_rate
        self._preamble_threshold = preamble_threshold
        self._base_address = base_address

        # Low Pass Filter
        self.lpf = filter.fir_filter_ccf(
            decimation=1,
            taps=filter.firdes.low_pass(
                gain=1,
                sampling_freq=sample_rate,
                cutoff_freq=symbol_rate,
                transition_width=200e3,
                window=window.WIN_HAMMING,
            ),
        )

        # Quadrature Demodulator
        fsk_deviation_hz = symbol_rate / 4
        self.quadrature_demod = analog.quadrature_demod_cf(gain=sample_rate / (2 * np.pi * fsk_deviation_hz))

        # DC Blocker (IIR high-pass)
        self.iir = filter.single_pole_iir_filter_ff(alpha=160e-6)
        self.subtraction = blocks.sub_ff()

        # Symbol Synchronisation
        self.symbol_sync = digital.symbol_sync_ff(
            digital.TED_MOD_MUELLER_AND_MULLER,
            sample_rate / symbol_rate,  # Samples/symbol
            4.5e-3,  # Loop bandwidth
            1.0,  # Damping factor
            1.0,  # TED gain
            0,  # Maximum deviation
            1,  # Output samples/symbol
            digital.constellation_bpsk().base(),
            digital.IR_MMSE_8TAP,
            128,
            [],
        )

        # Packet Sink Block (Custom OOT)
        self.packet_sink = sic.ble_packet_sink(self._base_address, self._preamble_threshold, 0x1, 0)

        # Stream connections
        self.connect(self, self.lpf)
        self.connect(self.lpf, self.quadrature_demod)
        self.connect(self.quadrature_demod, (self.subtraction, 0))
        self.connect(self.quadrature_demod, self.iir, (self.subtraction, 1))
        self.connect(self.subtraction, self.symbol_sync)
        self.connect(self.symbol_sync, self.packet_sink)
        self.connect(self.packet_sink, self)

        # Message connections
        self.message_port_register_hier_out("pdu")
        self.msg_connect((self.packet_sink, "pdu"), (self, "pdu"))

    def set_symbol_rate(self, symbol_rate: float):
        self._symbol_rate = symbol_rate

        # Update LPF taps
        lpf_taps = filter.firdes.low_pass(
            gain=1,
            sampling_freq=self._sample_rate,
            cutoff_freq=symbol_rate,
            transition_width=200e3,
            window=window.WIN_HAMMING,
        )
        self.lpf.set_taps(lpf_taps)

        # Update Quadrature Demod gain
        fsk_deviation_hz = symbol_rate / 4
        new_gain = self._sample_rate / (2 * np.pi * fsk_deviation_hz)
        self.quadrature_demod.set_gain(new_gain)

        # Update Symbol Sync samples/symbol
        self.symbol_sync.set_sps(self._sample_rate / symbol_rate)
