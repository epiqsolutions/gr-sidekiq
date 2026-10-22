#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Four-port Sidekiq RX
# GNU Radio version: v3.10.11.0-1-gee27d6f3

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import sidekiq
import sip
import threading



class quad_source(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Four-port Sidekiq RX", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Four-port Sidekiq RX")
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

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "quad_source")

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
        self.sample_rate = sample_rate = 60e6
        self.samp_rate = samp_rate = 32000
        self.run_rx_calibration = run_rx_calibration = 0
        self.frequency4 = frequency4 = 1030e6
        self.frequency3 = frequency3 = 1020e6
        self.frequency2 = frequency2 = 1010e6
        self.frequency = frequency = 1000e6
        self.bandwidth = bandwidth = sample_rate * .8

        ##################################################
        # Blocks
        ##################################################

        self._sample_rate_range = qtgui.Range(1e6, 250e6, 1e6, 60e6, 200)
        self._sample_rate_win = qtgui.RangeWidget(self._sample_rate_range, self.set_sample_rate, "'sample_rate'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._sample_rate_win)
        self._frequency4_range = qtgui.Range(250e6, 6000e6, 1e6, 1030e6, 200)
        self._frequency4_win = qtgui.RangeWidget(self._frequency4_range, self.set_frequency4, "Frequency 4", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._frequency4_win)
        self._frequency3_range = qtgui.Range(250e6, 6000e6, 1e6, 1020e6, 200)
        self._frequency3_win = qtgui.RangeWidget(self._frequency3_range, self.set_frequency3, "Frequency 3", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._frequency3_win)
        self._frequency2_range = qtgui.Range(250e6, 6000e6, 1e6, 1010e6, 200)
        self._frequency2_win = qtgui.RangeWidget(self._frequency2_range, self.set_frequency2, "Frequency 2", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._frequency2_win)
        self._frequency_range = qtgui.Range(250e6, 6000e6, 1e6, 1000e6, 200)
        self._frequency_win = qtgui.RangeWidget(self._frequency_range, self.set_frequency, "Frequency 1", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._frequency_win)
        self._bandwidth_range = qtgui.Range(1e6, 250e6, 1e6, sample_rate * .8, 200)
        self._bandwidth_win = qtgui.RangeWidget(self._bandwidth_range, self.set_bandwidth, "'bandwidth'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._bandwidth_win)
        self.sidekiq_sidekiq_rx_0 = sidekiq.sidekiq_rx(0, (-1), 0, frequency, 1, frequency2, 2, frequency3, 3, frequency4, sample_rate, bandwidth, 1, 1, 1, 0, 0, 1, 2)
        _run_rx_calibration_push_button = Qt.QPushButton('Run RX Calibration')
        _run_rx_calibration_push_button = Qt.QPushButton('Run RX Calibration')
        self._run_rx_calibration_choices = {'Pressed': 1, 'Released': 0}
        _run_rx_calibration_push_button.pressed.connect(lambda: self.set_run_rx_calibration(self._run_rx_calibration_choices['Pressed']))
        _run_rx_calibration_push_button.released.connect(lambda: self.set_run_rx_calibration(self._run_rx_calibration_choices['Released']))
        self.top_layout.addWidget(_run_rx_calibration_push_button)
        self.qtgui_freq_sink_x_3 = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            frequency4, #fc
            sample_rate, #bw
            "Channel 3", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_3.set_update_time(0.10)
        self.qtgui_freq_sink_x_3.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_3.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_3.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_3.enable_autoscale(False)
        self.qtgui_freq_sink_x_3.enable_grid(False)
        self.qtgui_freq_sink_x_3.set_fft_average(1.0)
        self.qtgui_freq_sink_x_3.enable_axis_labels(True)
        self.qtgui_freq_sink_x_3.enable_control_panel(False)
        self.qtgui_freq_sink_x_3.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_3.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_3.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_3.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_3.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_3.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_3_win = sip.wrapinstance(self.qtgui_freq_sink_x_3.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_3_win)
        self.qtgui_freq_sink_x_2 = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            frequency3, #fc
            sample_rate, #bw
            "Channel 2", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_2.set_update_time(0.10)
        self.qtgui_freq_sink_x_2.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_2.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_2.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_2.enable_autoscale(False)
        self.qtgui_freq_sink_x_2.enable_grid(False)
        self.qtgui_freq_sink_x_2.set_fft_average(1.0)
        self.qtgui_freq_sink_x_2.enable_axis_labels(True)
        self.qtgui_freq_sink_x_2.enable_control_panel(False)
        self.qtgui_freq_sink_x_2.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_2.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_2.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_2.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_2.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_2.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_2_win = sip.wrapinstance(self.qtgui_freq_sink_x_2.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_2_win)
        self.qtgui_freq_sink_x_0_0 = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            frequency2, #fc
            sample_rate, #bw
            "Channel 1", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_0_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0_0.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_0_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0_0.enable_grid(False)
        self.qtgui_freq_sink_x_0_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_0_0.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_0_win)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
            1024, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            frequency, #fc
            sample_rate, #bw
            "Channel 0", #name
            1,
            None # parent
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis((-140), 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(False)
        self.qtgui_freq_sink_x_0.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._qtgui_freq_sink_x_0_win)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.sidekiq_sidekiq_rx_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.sidekiq_sidekiq_rx_0, 1), (self.qtgui_freq_sink_x_0_0, 0))
        self.connect((self.sidekiq_sidekiq_rx_0, 2), (self.qtgui_freq_sink_x_2, 0))
        self.connect((self.sidekiq_sidekiq_rx_0, 3), (self.qtgui_freq_sink_x_3, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "quad_source")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_bandwidth(self.sample_rate * .8)
        self.qtgui_freq_sink_x_0.set_frequency_range(self.frequency, self.sample_rate)
        self.qtgui_freq_sink_x_0_0.set_frequency_range(self.frequency2, self.sample_rate)
        self.sidekiq_sidekiq_rx_0.set_rx_sample_rate(self.sample_rate)
        self.qtgui_freq_sink_x_2.set_frequency_range(self.frequency3, self.sample_rate)
        self.qtgui_freq_sink_x_3.set_frequency_range(self.frequency4, self.sample_rate)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_run_rx_calibration(self):
        return self.run_rx_calibration

    def set_run_rx_calibration(self, run_rx_calibration):
        self.run_rx_calibration = run_rx_calibration
        self.sidekiq_sidekiq_rx_0.run_rx_cal(self.run_rx_calibration)

    def get_frequency4(self):
        return self.frequency4

    def set_frequency4(self, frequency4):
        self.frequency4 = frequency4
        self.sidekiq_sidekiq_rx_0.set_rx_frequency_for_hdl(3, self.frequency4)
        self.qtgui_freq_sink_x_3.set_frequency_range(self.frequency4, self.sample_rate)

    def get_frequency3(self):
        return self.frequency3

    def set_frequency3(self, frequency3):
        self.frequency3 = frequency3
        self.sidekiq_sidekiq_rx_0.set_rx_frequency_for_hdl(2, self.frequency3)
        self.qtgui_freq_sink_x_2.set_frequency_range(self.frequency3, self.sample_rate)

    def get_frequency2(self):
        return self.frequency2

    def set_frequency2(self, frequency2):
        self.frequency2 = frequency2
        self.qtgui_freq_sink_x_0_0.set_frequency_range(self.frequency2, self.sample_rate)
        self.sidekiq_sidekiq_rx_0.set_rx_frequency_for_hdl(1, self.frequency2)

    def get_frequency(self):
        return self.frequency

    def set_frequency(self, frequency):
        self.frequency = frequency
        self.qtgui_freq_sink_x_0.set_frequency_range(self.frequency, self.sample_rate)
        self.sidekiq_sidekiq_rx_0.set_rx_frequency_for_hdl(0, self.frequency)

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth
        self.sidekiq_sidekiq_rx_0.set_rx_bandwidth(self.bandwidth)




def main(top_block_cls=quad_source, options=None):

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
