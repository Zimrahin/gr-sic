#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: ble_packet_example
# Author: Diego Badillo-San-Juan
# GNU Radio version: v3.11.0.0git-843-g6b25c171

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import analog
import math
from gnuradio import blocks
import pmt
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import sic
import sip
import threading



class ble_packet_example(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "ble_packet_example", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("ble_packet_example")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "ble_packet_example")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.tuning_LPF_cutoff_kHz = tuning_LPF_cutoff_kHz = 1000
        self.trigger_delay = trigger_delay = 200E-6
        self.samples_per_bit = samples_per_bit = 10
        self.samp_rate = samp_rate = int(10e6)
        self.plot_N = plot_N = 16000
        self.fsk_deviation_hz = fsk_deviation_hz = int(250e3)
        self.decimation = decimation = 1

        ##################################################
        # Blocks
        ##################################################

        self._tuning_LPF_cutoff_kHz_range = qtgui.Range(500, 2000, 1, 1000, 200)
        self._tuning_LPF_cutoff_kHz_win = qtgui.RangeWidget(self._tuning_LPF_cutoff_kHz_range, self.set_tuning_LPF_cutoff_kHz, "'tuning_LPF_cutoff_kHz'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._tuning_LPF_cutoff_kHz_win, 13, 0, 1, 1)
        for r in range(13, 14):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.sic_ble_packet_sink_0 = sic.ble_packet_sink(0x12345678, 0, 0x1, 0)
        self.qtgui_time_sink_x_1 = qtgui.time_sink_f(
            (int( plot_N/ samples_per_bit)), #size
            int(samp_rate / samples_per_bit), #samp_rate
            "Synced and downsampled", #name
            2, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_1.set_update_time(0.10)
        self.qtgui_time_sink_x_1.set_y_axis(-2, 2)

        self.qtgui_time_sink_x_1.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_1.enable_tags(True)
        self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, trigger_delay, 1, "Payload start")
        self.qtgui_time_sink_x_1.enable_autoscale(False)
        self.qtgui_time_sink_x_1.enable_grid(False)
        self.qtgui_time_sink_x_1.enable_axis_labels(True)
        self.qtgui_time_sink_x_1.enable_control_panel(False)
        self.qtgui_time_sink_x_1.enable_stem_plot(False)


        labels = ["Quad Demodded", "Sliced", 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, 0, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                self.qtgui_time_sink_x_1.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_time_sink_x_1.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_1.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_1.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_1.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_1.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_1.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_1_win = sip.wrapinstance(self.qtgui_time_sink_x_1.qwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_time_sink_x_1_win, 16, 0, 1, 1)
        for r in range(16, 17):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
            plot_N, #size
            samp_rate, #samp_rate
            "Testing Tag IQ Stream block", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, trigger_delay, 0, "Payload start")
        self.qtgui_time_sink_x_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0.enable_grid(False)
        self.qtgui_time_sink_x_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0.enable_stem_plot(False)


        labels = ['Signal 1', 'Signal 2', 'Signal 3', 'Signal 4', 'Signal 5',
            'Signal 6', 'Signal 7', 'Signal 8', 'Signal 9', 'Signal 10']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ['blue', 'red', 'green', 'black', 'cyan',
            'magenta', 'yellow', 'dark red', 'dark green', 'dark blue']
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]
        styles = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
            -1, -1, -1, -1, -1]


        for i in range(2):
            if len(labels[i]) == 0:
                if (i % 2 == 0):
                    self.qtgui_time_sink_x_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_win)
        self.low_pass_filter_0_0 = filter.fir_filter_ccf(
            decimation,
            firdes.low_pass(
                1,
                samp_rate,
                (tuning_LPF_cutoff_kHz*1000),
                (samp_rate/100),
                window.WIN_HAMMING,
                6.76))
        self.digital_symbol_sync_xx_0 = digital.symbol_sync_ff(
            digital.TED_MOD_MUELLER_AND_MULLER,
            (samples_per_bit / decimation),
            (0.045/10),
            1.0,
            1.0,
            0,
            1,
            digital.constellation_bpsk().base(),
            digital.IR_MMSE_8TAP,
            128,
            [])
        self.blocks_uchar_to_float_0_0_0_0 = blocks.uchar_to_float()
        self.blocks_throttle2_0 = blocks.throttle( gr.sizeof_gr_complex*1, (samp_rate/2), True, 0 if "auto" == "auto" else max( int(float(0.1) * (samp_rate/2)) if "auto" == "time" else int(0.1), 1) )
        self.blocks_file_source_0 = blocks.file_source(gr.sizeof_gr_complex*1, '/home/diego/Documents/GNU_radio_OOT_modules/gr-sic/examples/data/BLE_124B.dat', True, 0, 0)
        self.blocks_file_source_0.set_begin_tag(pmt.PMT_NIL)
        self.blocks_add_xx_0 = blocks.add_vcc(1)
        self.ble_tagged_iq_to_vector_0 = sic.tagged_iq_to_vector((int(trigger_delay * samp_rate)), (int(trigger_delay * samp_rate)), (128*8*8*samples_per_bit))
        self.ble_tag_iq_stream_0 = sic.tag_iq_stream(samples_per_bit)
        self.ble_plot_iq_from_pmt_0 = sic.plot_iq_from_pmt(int(samp_rate))
        self.analog_quadrature_demod_cf_0 = analog.quadrature_demod_cf(((samp_rate / decimation)/(2*math.pi*fsk_deviation_hz)))
        self.analog_fastnoise_source_x_0 = analog.fastnoise_source_c(analog.GR_GAUSSIAN, 0.025, 0, 8192)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.ble_tagged_iq_to_vector_0, 'out'), (self.ble_plot_iq_from_pmt_0, 'iq'))
        self.msg_connect((self.sic_ble_packet_sink_0, 'pdu'), (self.ble_plot_iq_from_pmt_0, 'pdu'))
        self.connect((self.analog_fastnoise_source_x_0, 0), (self.blocks_add_xx_0, 1))
        self.connect((self.analog_quadrature_demod_cf_0, 0), (self.digital_symbol_sync_xx_0, 0))
        self.connect((self.ble_tag_iq_stream_0, 0), (self.ble_tagged_iq_to_vector_0, 0))
        self.connect((self.ble_tag_iq_stream_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.ble_tag_iq_stream_0, 0))
        self.connect((self.blocks_add_xx_0, 0), (self.low_pass_filter_0_0, 0))
        self.connect((self.blocks_file_source_0, 0), (self.blocks_throttle2_0, 0))
        self.connect((self.blocks_throttle2_0, 0), (self.blocks_add_xx_0, 0))
        self.connect((self.blocks_uchar_to_float_0_0_0_0, 0), (self.qtgui_time_sink_x_1, 1))
        self.connect((self.digital_symbol_sync_xx_0, 0), (self.qtgui_time_sink_x_1, 0))
        self.connect((self.digital_symbol_sync_xx_0, 0), (self.sic_ble_packet_sink_0, 0))
        self.connect((self.low_pass_filter_0_0, 0), (self.analog_quadrature_demod_cf_0, 0))
        self.connect((self.sic_ble_packet_sink_0, 0), (self.ble_tag_iq_stream_0, 1))
        self.connect((self.sic_ble_packet_sink_0, 0), (self.blocks_uchar_to_float_0_0_0_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "ble_packet_example")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_tuning_LPF_cutoff_kHz(self):
        return self.tuning_LPF_cutoff_kHz

    def set_tuning_LPF_cutoff_kHz(self, tuning_LPF_cutoff_kHz):
        self.tuning_LPF_cutoff_kHz = tuning_LPF_cutoff_kHz
        self.low_pass_filter_0_0.set_taps(firdes.low_pass(1, self.samp_rate, (self.tuning_LPF_cutoff_kHz*1000), (self.samp_rate/100), window.WIN_HAMMING, 6.76))

    def get_trigger_delay(self):
        return self.trigger_delay

    def set_trigger_delay(self, trigger_delay):
        self.trigger_delay = trigger_delay
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, self.trigger_delay, 0, "Payload start")
        self.qtgui_time_sink_x_1.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, self.trigger_delay, 1, "Payload start")

    def get_samples_per_bit(self):
        return self.samples_per_bit

    def set_samples_per_bit(self, samples_per_bit):
        self.samples_per_bit = samples_per_bit
        self.digital_symbol_sync_xx_0.set_sps((self.samples_per_bit / self.decimation))
        self.qtgui_time_sink_x_1.set_samp_rate(int(self.samp_rate / self.samples_per_bit))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.analog_quadrature_demod_cf_0.set_gain(((self.samp_rate / self.decimation)/(2*math.pi*self.fsk_deviation_hz)))
        self.blocks_throttle2_0.set_sample_rate((self.samp_rate/2))
        self.low_pass_filter_0_0.set_taps(firdes.low_pass(1, self.samp_rate, (self.tuning_LPF_cutoff_kHz*1000), (self.samp_rate/100), window.WIN_HAMMING, 6.76))
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.qtgui_time_sink_x_1.set_samp_rate(int(self.samp_rate / self.samples_per_bit))

    def get_plot_N(self):
        return self.plot_N

    def set_plot_N(self, plot_N):
        self.plot_N = plot_N

    def get_fsk_deviation_hz(self):
        return self.fsk_deviation_hz

    def set_fsk_deviation_hz(self, fsk_deviation_hz):
        self.fsk_deviation_hz = fsk_deviation_hz
        self.analog_quadrature_demod_cf_0.set_gain(((self.samp_rate / self.decimation)/(2*math.pi*self.fsk_deviation_hz)))

    def get_decimation(self):
        return self.decimation

    def set_decimation(self, decimation):
        self.decimation = decimation
        self.analog_quadrature_demod_cf_0.set_gain(((self.samp_rate / self.decimation)/(2*math.pi*self.fsk_deviation_hz)))
        self.digital_symbol_sync_xx_0.set_sps((self.samples_per_bit / self.decimation))




def main(top_block_cls=ble_packet_example, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
