#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Diego Badillo-San-Juan <diego.badillo-san-juan@inria.fr>
# Copyright 2025 Inria.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import numpy as np
import pmt
from gnuradio import gr
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from pyqtgraph import PlotWidget

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")


class plot_sic_results(gr.sync_block, QtCore.QObject):
    # Define our signal with the required parameters
    update_signal = QtCore.pyqtSignal(
        np.ndarray,  # iq_before
        np.ndarray,  # iq_after
        bytes,  # payload_before
        bytes,  # payload_after
        str,  # title
    )

    def __init__(self, sample_rate: float, max_queue_size: int, parent=None):
        gr.sync_block.__init__(self, name="plot_sic_results", in_sig=None, out_sig=None)
        QtCore.QObject.__init__(self, parent)

        self.sample_rate = sample_rate
        self.max_queue_size = max_queue_size

        self.plot_widget = sic_plotter(sample_rate)
        self.plot_widget.show()
        self.update_signal.connect(self.plot_widget.update_plots)

        self.message_port_register_in(pmt.intern("in"))
        self.set_msg_handler(pmt.intern("in"), self.handle_plot_message)

        self.latest_data = None

    def handle_plot_message(self, msg) -> None:
        try:
            meta = pmt.car(msg)
            data_vector = pmt.cdr(msg)

            packet_id = pmt.dict_ref(meta, pmt.intern("Packet ID"), pmt.PMT_NIL)
            title = f"Packet {pmt.to_python(packet_id)}" if not pmt.is_null(packet_id) else "SIC Results"

            iq_before = np.array(pmt.c32vector_elements(pmt.vector_ref(data_vector, 0)))
            iq_after = np.array(pmt.c32vector_elements(pmt.vector_ref(data_vector, 1)))
            payload_before = bytes(pmt.u8vector_elements(pmt.vector_ref(data_vector, 2)))
            payload_after = bytes(pmt.u8vector_elements(pmt.vector_ref(data_vector, 3)))

            self.latest_data = (iq_before, iq_after, payload_before, payload_after, title)

            self.update_signal.emit(iq_before, iq_after, payload_before, payload_after, title)

        except Exception as e:
            print(f"Error processing message: {e}")

    def qwidget(self):
        # If we have data but no GUI update has happened yet, update now
        if self.latest_data:
            self.plot_widget.update_plots(*self.latest_data)
        return self.plot_widget


class sic_plotter(QtWidgets.QWidget):
    def __init__(self, sample_rate, parent=None):
        super().__init__(parent)
        self.sample_rate = sample_rate
        self.setWindowTitle("SIC Results")
        layout = QtWidgets.QGridLayout()
        self.setLayout(layout)

        # Create plots
        self.iq_before_plot = PlotWidget(title="IQ (Before SIC)")
        self.iq_after_plot = PlotWidget(title="IQ (After SIC)")
        self.payload_before_plot = PlotWidget(title="Payload (Before SIC)")
        self.payload_after_plot = PlotWidget(title="Payload (After SIC)")

        # Configure plots
        self._configure_iq_plot(self.iq_before_plot, autorange=True)
        self._configure_iq_plot(self.iq_after_plot, autorange=False)
        self._configure_payload_plot(self.payload_before_plot)
        self._configure_payload_plot(self.payload_after_plot)

        # Link X and Y axes between before and after IQ plots
        self.iq_after_plot.setXLink(self.iq_before_plot)
        self.iq_after_plot.setYLink(self.iq_before_plot)

        # Add to layout
        layout.addWidget(self.iq_before_plot, 0, 0)
        layout.addWidget(self.payload_before_plot, 0, 1)
        layout.addWidget(self.iq_after_plot, 1, 0)
        layout.addWidget(self.payload_after_plot, 1, 1)

    def _configure_iq_plot(self, plot, autorange=True):
        plot.setLabel("left", "Amplitude")
        plot.addLegend()
        plot.showGrid(x=True, y=True)
        if autorange:
            plot.enableAutoRange(axis=pg.ViewBox.XYAxes)
        else:
            plot.disableAutoRange()

    def _configure_payload_plot(self, plot):
        plot.setLabel("left", "Value (0-255)")
        plot.setLabel("bottom", "Byte Index")
        plot.setYRange(-5, 260)
        plot.showGrid(x=True, y=True)
        plot.getPlotItem().getAxis("left").setTicks([[(i, str(i)) for i in range(0, 256, 32)]])

    def update_plots(self, iq_before, iq_after, payload_before, payload_after, title):
        self.setWindowTitle(title)
        self._update_iq_plot(self.iq_before_plot, iq_before, "Before")
        self._update_iq_plot(self.iq_after_plot, iq_after, "After")
        self._update_payload_plot(self.payload_before_plot, payload_before, "Before")
        self._update_payload_plot(self.payload_after_plot, payload_after, "After")

    def _update_iq_plot(self, plot, iq_data, label):
        """Update a single IQ plot"""
        # Time axis
        plot_sample_rate = self.sample_rate if self.sample_rate == 1 else self.sample_rate / 1e6
        time_axis = np.arange(len(iq_data)) / plot_sample_rate

        plot.clear()

        plot.plot(time_axis, np.real(iq_data), pen=pg.mkPen("b", width=1.5), name="I (In-phase)", alpha=0.1)
        plot.plot(time_axis, np.imag(iq_data), pen=pg.mkPen("r", width=1.5), name="Q (Quadrature)", alpha=0.7)

        plot.setLabel("bottom", "Time (µs)" if plot_sample_rate != 1 else "Samples")

    def _update_payload_plot(self, plot, payload_data, label):
        payload_ints = np.frombuffer(payload_data, dtype=np.uint8)
        x = np.arange(len(payload_ints))

        plot.clear()
        plot.plot(
            x,
            payload_ints,
            pen=None,
            symbol="o",
            symbolPen="r" if label == "Before" else "b",
            symbolSize=5,
            symbolBrush=pg.mkBrush(255, 0, 0, 255) if label == "Before" else pg.mkBrush(0, 0, 255, 255),
        )
        plot.autoRange()
        plot.setYRange(-5, 260)
        plot.getPlotItem().getAxis("left").setTicks([[(i, str(i)) for i in range(0, 256, 32)]])
