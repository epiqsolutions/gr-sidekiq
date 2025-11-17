#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Sink Test
# GNU Radio version: v3.11.0.0git-605-g9b22fd38

from PyQt5 import Qt
from gnuradio import qtgui
from gnuradio import analog
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import soapy
from gnuradio.qtgui import Range, RangeWidget
from PyQt5 import QtCore



class sink_test(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Sink Test", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Sink Test")
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

        self.settings = Qt.QSettings("GNU Radio", "sink_test")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)

        ##################################################
        # Variables
        ##################################################
        self.sample_rate = sample_rate = 20e6
        self.tone_freq = tone_freq = 2e6
        self.run_tx_calibration = run_tx_calibration = 0
        self.min_buffer = min_buffer = 16380
        self.frequency = frequency = 1000e6
        self.bandwidth = bandwidth = sample_rate * 0.8
        self.attenuation = attenuation = 150

        ##################################################
        # Blocks
        ##################################################

        self._tone_freq_range = Range(1e6, 25e6, 1e6, 2e6, 200)
        self._tone_freq_win = RangeWidget(self._tone_freq_range, self.set_tone_freq, "'tone_freq'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._tone_freq_win, 10, 0, 2, 1)
        for r in range(10, 12):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._sample_rate_range = Range(1e6, 250e6, 1e6, 20e6, 200)
        self._sample_rate_win = RangeWidget(self._sample_rate_range, self.set_sample_rate, "'sample_rate'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._sample_rate_win, 0, 0, 2, 1)
        for r in range(0, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._frequency_range = Range(500e6, 6000e6, 1e6, 1000e6, 100)
        self._frequency_win = RangeWidget(self._frequency_range, self.set_frequency, "Frequency", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._frequency_win, 4, 0, 2, 1)
        for r in range(4, 6):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.soapy_custom_sink_0 = None
        dev = 'driver=' + 'sidekiq'
        stream_args = ''
        tune_args = ['']
        settings = ['']
        self.soapy_custom_sink_0 = soapy.sink(dev, "fc32",
                                1, 'card=2',
                                stream_args, tune_args, settings)
        self.soapy_custom_sink_0.set_sample_rate(0, sample_rate)
        self.soapy_custom_sink_0.set_bandwidth(0, 0)
        self.soapy_custom_sink_0.set_antenna(0, 'TX')
        self.soapy_custom_sink_0.set_frequency(0, frequency)
        self.soapy_custom_sink_0.set_frequency_correction(0, 0)
        self.soapy_custom_sink_0.set_gain(0, 60)
        self.soapy_custom_sink_0.set_dc_offset(0, 0)
        self.soapy_custom_sink_0.set_iq_balance(0, 0)
        _run_tx_calibration_push_button = Qt.QPushButton('')
        _run_tx_calibration_push_button = Qt.QPushButton('run_tx_calibration')
        self._run_tx_calibration_choices = {'Pressed': 1, 'Released': 0}
        _run_tx_calibration_push_button.pressed.connect(lambda: self.set_run_tx_calibration(self._run_tx_calibration_choices['Pressed']))
        _run_tx_calibration_push_button.released.connect(lambda: self.set_run_tx_calibration(self._run_tx_calibration_choices['Released']))
        self.top_grid_layout.addWidget(_run_tx_calibration_push_button, 12, 0, 1, 1)
        for r in range(12, 13):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._bandwidth_range = Range(1000, 250e6, 1e6, sample_rate * 0.8, 200)
        self._bandwidth_win = RangeWidget(self._bandwidth_range, self.set_bandwidth, "'bandwidth'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._bandwidth_win, 2, 0, 2, 1)
        for r in range(2, 4):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._attenuation_range = Range(0, 359, 10, 150, 100)
        self._attenuation_win = RangeWidget(self._attenuation_range, self.set_attenuation, "'attenuation'", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._attenuation_win, 8, 0, 2, 1)
        for r in range(8, 10):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self.analog_sig_source_x_0 = analog.sig_source_c(sample_rate, analog.GR_COS_WAVE, tone_freq, 1, 0, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.soapy_custom_sink_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "sink_test")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_bandwidth(self.sample_rate * 0.8)
        self.analog_sig_source_x_0.set_sampling_freq(self.sample_rate)

    def get_tone_freq(self):
        return self.tone_freq

    def set_tone_freq(self, tone_freq):
        self.tone_freq = tone_freq
        self.analog_sig_source_x_0.set_frequency(self.tone_freq)

    def get_run_tx_calibration(self):
        return self.run_tx_calibration

    def set_run_tx_calibration(self, run_tx_calibration):
        self.run_tx_calibration = run_tx_calibration

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_frequency(self):
        return self.frequency

    def set_frequency(self, frequency):
        self.frequency = frequency
        self.soapy_custom_sink_0.set_frequency(0, self.frequency)

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth

    def get_attenuation(self):
        return self.attenuation

    def set_attenuation(self, attenuation):
        self.attenuation = attenuation




def main(top_block_cls=sink_test, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()

    tb.start()

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
