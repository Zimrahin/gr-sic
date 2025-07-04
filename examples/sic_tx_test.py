#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: sic_tx_test
# Author: diego
# GNU Radio version: v3.11.0.0git-843-g6b25c171

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import sic
import threading
import sip



class sic_tx_test(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "sic_tx_test", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("sic_tx_test")
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "sic_tx_test")

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
        self.sps = sps = 10
        self.samp_rate = samp_rate = int(10e6)
        self.payload_length = payload_length = 50
        self.pause_button = pause_button = 0
        self.amplitude = amplitude = 1

        ##################################################
        # Blocks
        ##################################################

        self._payload_length_range = qtgui.Range(1, 255, 1, 50, 200)
        self._payload_length_win = qtgui.RangeWidget(self._payload_length_range, self.set_payload_length, "'payload_length'", "eng_slider", int, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._payload_length_win)
        self._amplitude_range = qtgui.Range(0, 1, 0.01, 1, 200)
        self._amplitude_win = qtgui.RangeWidget(self._amplitude_range, self.set_amplitude, "'amplitude'", "eng_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._amplitude_win)
        self.tx_trigger_button = _tx_trigger_button_toggle_button = qtgui.MsgPushButton('tx_trigger_button', 'pressed',1,"default","default")
        self.tx_trigger_button = _tx_trigger_button_toggle_button

        self.top_layout.addWidget(_tx_trigger_button_toggle_button)
        self.sic_transmission_enabler_0 = sic.transmission_enabler(1024)
        self.sic_periodic_message_source_0 = sic.periodic_message_source(gr.pmt.mp("trigger"), 1000, (-1))
        self.sic_ble_packet_source_0 = sic.ble_packet_source(10e6, payload_length, amplitude)
        self.qtgui_waterfall_sink_x_0 = qtgui.waterfall_sink_c(
            8192, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            0, #fc
            samp_rate, #bw
            "", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_waterfall_sink_x_0.set_update_time(0.10)
        self.qtgui_waterfall_sink_x_0.enable_grid(False)
        self.qtgui_waterfall_sink_x_0.enable_axis_labels(True)



        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_waterfall_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_waterfall_sink_x_0.set_color_map(i, colors[i])
            self.qtgui_waterfall_sink_x_0.set_line_alpha(i, alphas[i])

        self.qtgui_waterfall_sink_x_0.set_intensity_range(-140, 0)

        self._qtgui_waterfall_sink_x_0_win = sip.wrapinstance(self.qtgui_waterfall_sink_x_0.qwidget(), Qt.QWidget)

        self.top_layout.addWidget(self._qtgui_waterfall_sink_x_0_win)
        self.qtgui_time_sink_x_0 = qtgui.time_sink_c(
            (int(samp_rate/100)), #size
            samp_rate, #samp_rate
            "", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0.enable_tags(True)
        self.qtgui_time_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_TAG, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "trigger")
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
        self._pause_button_choices = {'Pressed': bool(1), 'Released': bool(0)}

        _pause_button_toggle_button = qtgui.ToggleButton(self.set_pause_button, 'pause_button', self._pause_button_choices, False, 'value')
        _pause_button_toggle_button.setColors("default", "default", "default", "default")
        self.pause_button = _pause_button_toggle_button

        self.top_layout.addWidget(_pause_button_toggle_button)
        self.blocks_throttle2_0_1 = blocks.throttle( gr.sizeof_gr_complex*1, (samp_rate/200), True, 0 if "auto" == "auto" else max( int(float(0.1) * (samp_rate/200)) if "auto" == "time" else int(0.1), 1) )


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.pause_button, 'state'), (self.sic_periodic_message_source_0, 'pause'))
        self.msg_connect((self.sic_periodic_message_source_0, 'out'), (self.sic_transmission_enabler_0, 'trigger'))
        self.msg_connect((self.tx_trigger_button, 'pressed'), (self.sic_transmission_enabler_0, 'trigger'))
        self.connect((self.blocks_throttle2_0_1, 0), (self.sic_ble_packet_source_0, 0))
        self.connect((self.sic_ble_packet_source_0, 0), (self.qtgui_time_sink_x_0, 0))
        self.connect((self.sic_ble_packet_source_0, 0), (self.qtgui_waterfall_sink_x_0, 0))
        self.connect((self.sic_transmission_enabler_0, 0), (self.blocks_throttle2_0_1, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "sic_tx_test")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_throttle2_0_1.set_sample_rate((self.samp_rate/200))
        self.qtgui_time_sink_x_0.set_samp_rate(self.samp_rate)
        self.qtgui_waterfall_sink_x_0.set_frequency_range(0, self.samp_rate)

    def get_payload_length(self):
        return self.payload_length

    def set_payload_length(self, payload_length):
        self.payload_length = payload_length
        self.sic_ble_packet_source_0.set_payload_length(self.payload_length)

    def get_pause_button(self):
        return self.pause_button

    def set_pause_button(self, pause_button):
        self.pause_button = pause_button

    def get_amplitude(self):
        return self.amplitude

    def set_amplitude(self, amplitude):
        self.amplitude = amplitude
        self.sic_ble_packet_source_0.set_amplitude(self.amplitude)




def main(top_block_cls=sic_tx_test, options=None):

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
