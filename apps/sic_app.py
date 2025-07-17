#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: SIC Simulation
# Author: Diego Badillo-San-Juan
# GNU Radio version: v3.11.0.0git-843-g6b25c171

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import iio
from gnuradio import sic
import threading
import sip



class sic_app(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "SIC Simulation", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("SIC Simulation")
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "sic_app")

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
        self.samp_rate = samp_rate = int(2e6)
        self.trigger_delay = trigger_delay = 700e-6 *2
        self.symbol_rate_high = symbol_rate_high = int(1e6)
        self.payload_length_l = payload_length_l = 140
        self.payload_length_h = payload_length_h = 10
        self.pause_button = pause_button = 0
        self.label_ble = label_ble = ''
        self.label_802154_tx_rate = label_802154_tx_rate = '250 kbit/s'
        self.label_802154 = label_802154 = ''
        self.delay_l = delay_l = 0
        self.delay_h = delay_h = (int(160 * samp_rate * 1e-6))
        self.centre_frequency = centre_frequency = int(2495e6)
        self.ble_transmission_rate = ble_transmission_rate = 1000000
        self.amplitude_l = amplitude_l = 0.5
        self.amplitude_h = amplitude_h = 1

        ##################################################
        # Blocks
        ##################################################

        self._payload_length_l_range = qtgui.Range(1, 255, 1, 140, 200)
        self._payload_length_l_win = qtgui.RangeWidget(self._payload_length_l_range, self.set_payload_length_l, "Payload Length (bytes)", "eng_slider", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._payload_length_l_win, 3, 1, 1, 1)
        for r in range(3, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._delay_l_range = qtgui.Range(0, (int(800 * samp_rate * 1e-6)), 1, 0, 200)
        self._delay_l_win = qtgui.RangeWidget(self._delay_l_range, self.set_delay_l, "Delay (µs)", "eng_slider", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._delay_l_win, 4, 1, 1, 1)
        for r in range(4, 5):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        # Create the options list
        self._ble_transmission_rate_options = [1000000, 2000000]
        # Create the labels list
        self._ble_transmission_rate_labels = ['1 Mbit/s', '2 Mbit/s']
        # Create the combo box
        self._ble_transmission_rate_tool_bar = Qt.QToolBar(self)
        self._ble_transmission_rate_tool_bar.addWidget(Qt.QLabel("BLE Transmission Rate" + ": "))
        self._ble_transmission_rate_combo_box = Qt.QComboBox()
        self._ble_transmission_rate_tool_bar.addWidget(self._ble_transmission_rate_combo_box)
        for _label in self._ble_transmission_rate_labels: self._ble_transmission_rate_combo_box.addItem(_label)
        self._ble_transmission_rate_callback = lambda i: Qt.QMetaObject.invokeMethod(self._ble_transmission_rate_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._ble_transmission_rate_options.index(i)))
        self._ble_transmission_rate_callback(self.ble_transmission_rate)
        self._ble_transmission_rate_combo_box.currentIndexChanged.connect(
            lambda i: self.set_ble_transmission_rate(self._ble_transmission_rate_options[i]))
        # Create the radio buttons
        self.top_grid_layout.addWidget(self._ble_transmission_rate_tool_bar, 2, 1, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.tx_trigger_button = _tx_trigger_button_toggle_button = qtgui.MsgPushButton('Trigger Transmission', 'pressed',1,"green","default")
        self.tx_trigger_button = _tx_trigger_button_toggle_button

        self.top_grid_layout.addWidget(_tx_trigger_button_toggle_button, 0, 0, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.sic_transmission_enabler_0 = sic.transmission_enabler(1024)
        self.sic_periodic_message_source_0 = sic.periodic_message_source(gr.pmt.mp("trigger"), 2000, (-1))
        self.sic_ble_packet_source_0_0 = sic.ble_packet_source(samp_rate, payload_length_l, ble_transmission_rate, 0x12345678)
        self.qtgui_time_sink_x_0_0_0 = qtgui.time_sink_c(
            160000, #size
            samp_rate, #samp_rate
            "Testing Tag IQ Stream block", #name
            1, #number of inputs
            None # parent
        )
        self.qtgui_time_sink_x_0_0_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0_0_0.set_y_axis(-2, 2)

        self.qtgui_time_sink_x_0_0_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0_0_0.enable_tags(True)
        self.qtgui_time_sink_x_0_0_0.set_trigger_mode(qtgui.TRIG_MODE_NORM, qtgui.TRIG_SLOPE_POS, 0.5, trigger_delay, 0, "Payload start")
        self.qtgui_time_sink_x_0_0_0.enable_autoscale(True)
        self.qtgui_time_sink_x_0_0_0.enable_grid(False)
        self.qtgui_time_sink_x_0_0_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0_0_0.enable_control_panel(False)
        self.qtgui_time_sink_x_0_0_0.enable_stem_plot(False)


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
                    self.qtgui_time_sink_x_0_0_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0_0_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0_0_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0_0_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0_0_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0_0_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0_0_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0_0_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_0_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0_0_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_time_sink_x_0_0_0_win)
        self._payload_length_h_range = qtgui.Range(1, 125, 1, 10, 200)
        self._payload_length_h_win = qtgui.RangeWidget(self._payload_length_h_range, self.set_payload_length_h, "Payload Length (bytes)", "eng_slider", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._payload_length_h_win, 3, 0, 1, 1)
        for r in range(3, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._pause_button_choices = {'Pressed': bool(1), 'Released': bool(0)}

        _pause_button_toggle_button = qtgui.ToggleButton(self.set_pause_button, 'Pause Periodic Transmission', self._pause_button_choices, False, 'value')
        _pause_button_toggle_button.setColors("red", "default", "red", "default")
        self.pause_button = _pause_button_toggle_button

        self.top_grid_layout.addWidget(_pause_button_toggle_button, 0, 1, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._label_ble_tool_bar = Qt.QToolBar(self)

        if None:
            self._label_ble_formatter = None
        else:
            self._label_ble_formatter = lambda x: str(x)

        self._label_ble_tool_bar.addWidget(Qt.QLabel("<font size='5'><b>BLE</b> (weaker signal) </font>"))
        self._label_ble_label = Qt.QLabel(str(self._label_ble_formatter(self.label_ble)))
        self._label_ble_tool_bar.addWidget(self._label_ble_label)
        self.top_grid_layout.addWidget(self._label_ble_tool_bar, 1, 1, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._label_802154_tx_rate_tool_bar = Qt.QToolBar(self)

        if None:
            self._label_802154_tx_rate_formatter = None
        else:
            self._label_802154_tx_rate_formatter = lambda x: str(x)

        self._label_802154_tx_rate_tool_bar.addWidget(Qt.QLabel("Transmission Rate: "))
        self._label_802154_tx_rate_label = Qt.QLabel(str(self._label_802154_tx_rate_formatter(self.label_802154_tx_rate)))
        self._label_802154_tx_rate_tool_bar.addWidget(self._label_802154_tx_rate_label)
        self.top_grid_layout.addWidget(self._label_802154_tx_rate_tool_bar, 2, 0, 1, 1)
        for r in range(2, 3):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._label_802154_tool_bar = Qt.QToolBar(self)

        if None:
            self._label_802154_formatter = None
        else:
            self._label_802154_formatter = lambda x: str(x)

        self._label_802154_tool_bar.addWidget(Qt.QLabel("<font size='5'><b>IEEE 802.15.4</b> (stronger signal) </font>"))
        self._label_802154_label = Qt.QLabel(str(self._label_802154_formatter(self.label_802154)))
        self._label_802154_tool_bar.addWidget(self._label_802154_label)
        self.top_grid_layout.addWidget(self._label_802154_tool_bar, 1, 0, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.iio_pluto_source_0 = iio.fmcomms2_source_fc32('192.168.2.1' if '192.168.2.1' else iio.get_pluto_uri(), [True, True], 32768)
        self.iio_pluto_source_0.set_len_tag_key('')
        self.iio_pluto_source_0.set_frequency(centre_frequency)
        self.iio_pluto_source_0.set_samplerate(samp_rate)
        self.iio_pluto_source_0.set_gain_mode(0, 'manual')
        self.iio_pluto_source_0.set_gain(0, 50)
        self.iio_pluto_source_0.set_quadrature(True)
        self.iio_pluto_source_0.set_rfdc(False)
        self.iio_pluto_source_0.set_bbdc(False)
        self.iio_pluto_source_0.set_filter_params('Auto', '', 0, 0)
        self.iio_pluto_sink_0 = iio.fmcomms2_sink_fc32('192.168.3.1' if '192.168.3.1' else iio.get_pluto_uri(), [True, True], 32768, False)
        self.iio_pluto_sink_0.set_len_tag_key('')
        self.iio_pluto_sink_0.set_bandwidth(int(samp_rate))
        self.iio_pluto_sink_0.set_frequency(centre_frequency)
        self.iio_pluto_sink_0.set_samplerate(samp_rate)
        self.iio_pluto_sink_0.set_attenuation(0, 0)
        self.iio_pluto_sink_0.set_filter_params('Auto', '', 0, 0)
        self._delay_h_range = qtgui.Range(0, (int(800 * samp_rate * 1e-6)), 1, (int(160 * samp_rate * 1e-6)), 200)
        self._delay_h_win = qtgui.RangeWidget(self._delay_h_range, self.set_delay_h, "Delay (µs)", "eng_slider", int, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._delay_h_win, 4, 0, 1, 1)
        for r in range(4, 5):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.blocks_delay_0 = blocks.delay(gr.sizeof_gr_complex*1, delay_l)
        self._amplitude_l_range = qtgui.Range(0, 4, 0.2, 0.5, 200)
        self._amplitude_l_win = qtgui.RangeWidget(self._amplitude_l_range, self.set_amplitude_l, "Amplitude Multiplier", "eng_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._amplitude_l_win, 5, 1, 1, 1)
        for r in range(5, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._amplitude_h_range = qtgui.Range(0, 4, 0.2, 1, 200)
        self._amplitude_h_win = qtgui.RangeWidget(self._amplitude_h_range, self.set_amplitude_h, "Amplitude Multiplier", "eng_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._amplitude_h_win, 5, 0, 1, 1)
        for r in range(5, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)


        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.pause_button, 'state'), (self.sic_periodic_message_source_0, 'pause'))
        self.msg_connect((self.sic_periodic_message_source_0, 'out'), (self.sic_transmission_enabler_0, 'trigger'))
        self.msg_connect((self.tx_trigger_button, 'pressed'), (self.sic_transmission_enabler_0, 'trigger'))
        self.connect((self.blocks_delay_0, 0), (self.iio_pluto_sink_0, 0))
        self.connect((self.iio_pluto_source_0, 0), (self.qtgui_time_sink_x_0_0_0, 0))
        self.connect((self.sic_ble_packet_source_0_0, 0), (self.blocks_delay_0, 0))
        self.connect((self.sic_transmission_enabler_0, 0), (self.sic_ble_packet_source_0_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "sic_app")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_delay_h((int(160 * self.samp_rate * 1e-6)))
        self.iio_pluto_sink_0.set_bandwidth(int(self.samp_rate))
        self.iio_pluto_sink_0.set_samplerate(self.samp_rate)
        self.iio_pluto_source_0.set_samplerate(self.samp_rate)
        self.qtgui_time_sink_x_0_0_0.set_samp_rate(self.samp_rate)

    def get_trigger_delay(self):
        return self.trigger_delay

    def set_trigger_delay(self, trigger_delay):
        self.trigger_delay = trigger_delay
        self.qtgui_time_sink_x_0_0_0.set_trigger_mode(qtgui.TRIG_MODE_NORM, qtgui.TRIG_SLOPE_POS, 0.5, self.trigger_delay, 0, "Payload start")

    def get_symbol_rate_high(self):
        return self.symbol_rate_high

    def set_symbol_rate_high(self, symbol_rate_high):
        self.symbol_rate_high = symbol_rate_high

    def get_payload_length_l(self):
        return self.payload_length_l

    def set_payload_length_l(self, payload_length_l):
        self.payload_length_l = payload_length_l
        self.sic_ble_packet_source_0_0.set_payload_length(self.payload_length_l)

    def get_payload_length_h(self):
        return self.payload_length_h

    def set_payload_length_h(self, payload_length_h):
        self.payload_length_h = payload_length_h

    def get_pause_button(self):
        return self.pause_button

    def set_pause_button(self, pause_button):
        self.pause_button = pause_button

    def get_label_ble(self):
        return self.label_ble

    def set_label_ble(self, label_ble):
        self.label_ble = label_ble
        Qt.QMetaObject.invokeMethod(self._label_ble_label, "setText", Qt.Q_ARG("QString", str(self._label_ble_formatter(self.label_ble))))

    def get_label_802154_tx_rate(self):
        return self.label_802154_tx_rate

    def set_label_802154_tx_rate(self, label_802154_tx_rate):
        self.label_802154_tx_rate = label_802154_tx_rate
        Qt.QMetaObject.invokeMethod(self._label_802154_tx_rate_label, "setText", Qt.Q_ARG("QString", str(self._label_802154_tx_rate_formatter(self.label_802154_tx_rate))))

    def get_label_802154(self):
        return self.label_802154

    def set_label_802154(self, label_802154):
        self.label_802154 = label_802154
        Qt.QMetaObject.invokeMethod(self._label_802154_label, "setText", Qt.Q_ARG("QString", str(self._label_802154_formatter(self.label_802154))))

    def get_delay_l(self):
        return self.delay_l

    def set_delay_l(self, delay_l):
        self.delay_l = delay_l
        self.blocks_delay_0.set_dly(int(self.delay_l))

    def get_delay_h(self):
        return self.delay_h

    def set_delay_h(self, delay_h):
        self.delay_h = delay_h

    def get_centre_frequency(self):
        return self.centre_frequency

    def set_centre_frequency(self, centre_frequency):
        self.centre_frequency = centre_frequency
        self.iio_pluto_sink_0.set_frequency(self.centre_frequency)
        self.iio_pluto_source_0.set_frequency(self.centre_frequency)

    def get_ble_transmission_rate(self):
        return self.ble_transmission_rate

    def set_ble_transmission_rate(self, ble_transmission_rate):
        self.ble_transmission_rate = ble_transmission_rate
        self._ble_transmission_rate_callback(self.ble_transmission_rate)
        self.sic_ble_packet_source_0_0.set_transmission_rate(self.ble_transmission_rate)

    def get_amplitude_l(self):
        return self.amplitude_l

    def set_amplitude_l(self, amplitude_l):
        self.amplitude_l = amplitude_l

    def get_amplitude_h(self):
        return self.amplitude_h

    def set_amplitude_h(self, amplitude_h):
        self.amplitude_h = amplitude_h




def main(top_block_cls=sic_app, options=None):

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
