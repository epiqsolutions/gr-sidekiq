#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Top Block
# GNU Radio version: v3.10-compat-xxx-xunknown

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print("Warning: failed to XInitThreads()")

from PyQt5 import Qt
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import analog
from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio.qtgui import Range, RangeWidget
from PyQt5 import QtCore
import sidekiq



from gnuradio import qtgui

class top_block(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Top Block", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Top Block")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
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

        self.settings = Qt.QSettings("GNU Radio", "top_block")

        try:
            if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
                self.restoreGeometry(self.settings.value("geometry").toByteArray())
            else:
                self.restoreGeometry(self.settings.value("geometry"))
        except:
            pass

        ##################################################
        # Variables
        ##################################################
        self.sample_rate = sample_rate = 10e6
        self.suppress_tune_transients_chooser = suppress_tune_transients_chooser = 0
        self.source_min_output_buffer = source_min_output_buffer = 2**14*4
        self.max_sample_rate = max_sample_rate = 122.88e6
        self.max_attenuation = max_attenuation = 5000
        self.center_freq = center_freq = 915e6
        self.bandwidth = bandwidth = sample_rate
        self.atten_quart_db = atten_quart_db = 50

        ##################################################
        # Blocks
        ##################################################
        self._sample_rate_range = Range(0.1e6, max_sample_rate, 0.1e6, 10e6, 200)
        self._sample_rate_win = RangeWidget(self._sample_rate_range, self.set_sample_rate, 'Sample Rate', "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._sample_rate_win, 1, 0, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._center_freq_range = Range(100e6, 6000e6, 1e6, 915e6, 200)
        self._center_freq_win = RangeWidget(self._center_freq_range, self.set_center_freq, 'Frequency', "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._center_freq_win, 0, 0, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(0, 1):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._bandwidth_range = Range(0.1e6, max_sample_rate, 0.1e6, sample_rate, 200)
        self._bandwidth_win = RangeWidget(self._bandwidth_range, self.set_bandwidth, 'Bandwidth', "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._bandwidth_win, 1, 1, 1, 1)
        for r in range(1, 2):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        self._atten_quart_db_range = Range(0, max_attenuation, 1, 50, 200)
        self._atten_quart_db_win = RangeWidget(self._atten_quart_db_range, self.set_atten_quart_db, 'Attenuation ', "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_grid_layout.addWidget(self._atten_quart_db_win, 0, 1, 1, 1)
        for r in range(0, 1):
            self.top_grid_layout.setRowStretch(r, 1)
        for c in range(1, 2):
            self.top_grid_layout.setColumnStretch(c, 1)
        # Create the options list
        self._suppress_tune_transients_chooser_options = [0, 1]
        # Create the labels list
        self._suppress_tune_transients_chooser_labels = ['False', 'True']
        # Create the combo box
        self._suppress_tune_transients_chooser_tool_bar = Qt.QToolBar(self)
        self._suppress_tune_transients_chooser_tool_bar.addWidget(Qt.QLabel('Suppress Tune Transients' + ": "))
        self._suppress_tune_transients_chooser_combo_box = Qt.QComboBox()
        self._suppress_tune_transients_chooser_tool_bar.addWidget(self._suppress_tune_transients_chooser_combo_box)
        for _label in self._suppress_tune_transients_chooser_labels: self._suppress_tune_transients_chooser_combo_box.addItem(_label)
        self._suppress_tune_transients_chooser_callback = lambda i: Qt.QMetaObject.invokeMethod(self._suppress_tune_transients_chooser_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._suppress_tune_transients_chooser_options.index(i)))
        self._suppress_tune_transients_chooser_callback(self.suppress_tune_transients_chooser)
        self._suppress_tune_transients_chooser_combo_box.currentIndexChanged.connect(
            lambda i: self.set_suppress_tune_transients_chooser(self._suppress_tune_transients_chooser_options[i]))
        # Create the radio buttons
        self.top_layout.addWidget(self._suppress_tune_transients_chooser_tool_bar)
        self.sidekiq_tx_0 = sidekiq.sidekiq_tx(sample_rate, atten_quart_db, center_freq, bandwidth, 1, 0, 0, 1020 )
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_float*1, sample_rate,True)
        self.blocks_message_debug_0 = blocks.message_debug(True)
        self.analog_sig_source_x_0 = analog.sig_source_f(sample_rate, analog.GR_COS_WAVE, 16000, 0.75, 0, 0)
        self.analog_sig_source_x_0.set_min_output_buffer(65536)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.sidekiq_tx_0, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "top_block")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_bandwidth(self.sample_rate)
        self.analog_sig_source_x_0.set_sampling_freq(self.sample_rate)
        self.blocks_throttle_0.set_sample_rate(self.sample_rate)
        self.sidekiq_tx_0.set_tx_sample_rate(self.sample_rate)

    def get_suppress_tune_transients_chooser(self):
        return self.suppress_tune_transients_chooser

    def set_suppress_tune_transients_chooser(self, suppress_tune_transients_chooser):
        self.suppress_tune_transients_chooser = suppress_tune_transients_chooser
        self._suppress_tune_transients_chooser_callback(self.suppress_tune_transients_chooser)

    def get_source_min_output_buffer(self):
        return self.source_min_output_buffer

    def set_source_min_output_buffer(self, source_min_output_buffer):
        self.source_min_output_buffer = source_min_output_buffer

    def get_max_sample_rate(self):
        return self.max_sample_rate

    def set_max_sample_rate(self, max_sample_rate):
        self.max_sample_rate = max_sample_rate

    def get_max_attenuation(self):
        return self.max_attenuation

    def set_max_attenuation(self, max_attenuation):
        self.max_attenuation = max_attenuation

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.sidekiq_tx_0.set_tx_frequency(self.center_freq)

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth
        self.sidekiq_tx_0.set_tx_bandwidth(self.bandwidth)

    def get_atten_quart_db(self):
        return self.atten_quart_db

    def set_atten_quart_db(self, atten_quart_db):
        self.atten_quart_db = atten_quart_db
        self.sidekiq_tx_0.set_tx_attenuation(self.atten_quart_db)




def main(top_block_cls=top_block, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
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
