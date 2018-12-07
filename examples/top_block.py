#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Top Block
# Generated: Fri Dec  7 08:53:56 2018
##################################################

from distutils.version import StrictVersion

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt5 import Qt
from PyQt5 import Qt, QtCore
from PyQt5.QtCore import QObject, pyqtSlot
from gnuradio import analog
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from gnuradio.qtgui import Range, RangeWidget
from optparse import OptionParser
import sidekiq
import sys
import threading
import time
from gnuradio import qtgui


class top_block(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Top Block")
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

        if StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
            self.restoreGeometry(self.settings.value("geometry").toByteArray())
        else:
            self.restoreGeometry(self.settings.value("geometry", type=QtCore.QByteArray))

        ##################################################
        # Variables
        ##################################################
        self.variable_function_probe_0 = variable_function_probe_0 = 0
        self.sample_rate = sample_rate = 1e6
        self.vector_length = vector_length = 1
        self.variable_qtgui_label_0 = variable_qtgui_label_0 = variable_function_probe_0
        self.suppress_tune_transients_chooser = suppress_tune_transients_chooser = 0
        self.source_min_output_buffer = source_min_output_buffer = 2**14*4
        self.max_sample_rate = max_sample_rate = 122.88e6
        self.max_attenuation = max_attenuation = 5000
        self.center_freq = center_freq = 915e6
        self.bandwidth = bandwidth = sample_rate
        self.atten_quart_db = atten_quart_db = 55

        ##################################################
        # Blocks
        ##################################################
        self._sample_rate_range = Range(0.1e6, max_sample_rate, 0.1e6, 1e6, 200)
        self._sample_rate_win = RangeWidget(self._sample_rate_range, self.set_sample_rate, 'Sample Rate', "counter_slider", float)
        self.top_grid_layout.addWidget(self._sample_rate_win, 1, 0, 1, 1)
        [self.top_grid_layout.setRowStretch(r,1) for r in range(1,2)]
        [self.top_grid_layout.setColumnStretch(c,1) for c in range(0,1)]
        self._suppress_tune_transients_chooser_options = (0, 1, )
        self._suppress_tune_transients_chooser_labels = ('False', 'True', )
        self._suppress_tune_transients_chooser_tool_bar = Qt.QToolBar(self)
        self._suppress_tune_transients_chooser_tool_bar.addWidget(Qt.QLabel('Suppress Tune Transients'+": "))
        self._suppress_tune_transients_chooser_combo_box = Qt.QComboBox()
        self._suppress_tune_transients_chooser_tool_bar.addWidget(self._suppress_tune_transients_chooser_combo_box)
        for label in self._suppress_tune_transients_chooser_labels: self._suppress_tune_transients_chooser_combo_box.addItem(label)
        self._suppress_tune_transients_chooser_callback = lambda i: Qt.QMetaObject.invokeMethod(self._suppress_tune_transients_chooser_combo_box, "setCurrentIndex", Qt.Q_ARG("int", self._suppress_tune_transients_chooser_options.index(i)))
        self._suppress_tune_transients_chooser_callback(self.suppress_tune_transients_chooser)
        self._suppress_tune_transients_chooser_combo_box.currentIndexChanged.connect(
        	lambda i: self.set_suppress_tune_transients_chooser(self._suppress_tune_transients_chooser_options[i]))
        self.top_layout.addWidget(self._suppress_tune_transients_chooser_tool_bar)
        self._center_freq_range = Range(100e6, 6000e6, 1e6, 915e6, 200)
        self._center_freq_win = RangeWidget(self._center_freq_range, self.set_center_freq, 'Frequency', "counter_slider", float)
        self.top_grid_layout.addWidget(self._center_freq_win, 0, 0, 1, 1)
        [self.top_grid_layout.setRowStretch(r,1) for r in range(0,1)]
        [self.top_grid_layout.setColumnStretch(c,1) for c in range(0,1)]
        self.blocks_probe_rate_0 = blocks.probe_rate(gr.sizeof_gr_complex*1, 500.0, 0.15)
        self._bandwidth_range = Range(0.1e6, max_sample_rate, 0.1e6, sample_rate, 200)
        self._bandwidth_win = RangeWidget(self._bandwidth_range, self.set_bandwidth, 'Bandwidth', "counter_slider", float)
        self.top_grid_layout.addWidget(self._bandwidth_win, 1, 1, 1, 1)
        [self.top_grid_layout.setRowStretch(r,1) for r in range(1,2)]
        [self.top_grid_layout.setColumnStretch(c,1) for c in range(1,2)]
        self._atten_quart_db_range = Range(1, max_attenuation, 1, 55, 200)
        self._atten_quart_db_win = RangeWidget(self._atten_quart_db_range, self.set_atten_quart_db, 'Attenuation ', "counter_slider", float)
        self.top_grid_layout.addWidget(self._atten_quart_db_win, 0, 1, 1, 1)
        [self.top_grid_layout.setRowStretch(r,1) for r in range(0,1)]
        [self.top_grid_layout.setColumnStretch(c,1) for c in range(1,2)]
        self._variable_qtgui_label_0_tool_bar = Qt.QToolBar(self)

        if None:
          self._variable_qtgui_label_0_formatter = None
        else:
          self._variable_qtgui_label_0_formatter = lambda x: eng_notation.num_to_str(x)

        self._variable_qtgui_label_0_tool_bar.addWidget(Qt.QLabel('Rx Rate'+": "))
        self._variable_qtgui_label_0_label = Qt.QLabel(str(self._variable_qtgui_label_0_formatter(self.variable_qtgui_label_0)))
        self._variable_qtgui_label_0_tool_bar.addWidget(self._variable_qtgui_label_0_label)
        self.top_grid_layout.addWidget(self._variable_qtgui_label_0_tool_bar, 5, 0, 1, 1)
        [self.top_grid_layout.setRowStretch(r,1) for r in range(5,6)]
        [self.top_grid_layout.setColumnStretch(c,1) for c in range(0,1)]

        def _variable_function_probe_0_probe():
            while True:
                val = self.blocks_probe_rate_0.rate()
                try:
                    self.set_variable_function_probe_0(val)
                except AttributeError:
                    pass
                time.sleep(1.0 / (10))
        _variable_function_probe_0_thread = threading.Thread(target=_variable_function_probe_0_probe)
        _variable_function_probe_0_thread.daemon = True
        _variable_function_probe_0_thread.start()

        self.sidekiq_sidekiq_tx_0 = sidekiq.sidekiq_tx(sample_rate, atten_quart_db, center_freq, bandwidth, 1, bool(suppress_tune_transients_chooser),
                0, 32764, ())

        self.blocks_tag_debug_0 = blocks.tag_debug(gr.sizeof_gr_complex*1, '', ""); self.blocks_tag_debug_0.set_display(True)
        self.blocks_message_debug_0 = blocks.message_debug()
        self.blocks_conjugate_cc_0 = blocks.conjugate_cc()
        self.analog_sig_source_x_0 = analog.sig_source_c(sample_rate, analog.GR_COS_WAVE, 1000, 0, 0)
        (self.analog_sig_source_x_0).set_min_output_buffer(65536)

        ##################################################
        # Connections
        ##################################################
        self.msg_connect((self.sidekiq_sidekiq_tx_0, 'telemetry'), (self.blocks_message_debug_0, 'print'))
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_conjugate_cc_0, 0))
        self.connect((self.blocks_conjugate_cc_0, 0), (self.blocks_probe_rate_0, 0))
        self.connect((self.blocks_conjugate_cc_0, 0), (self.blocks_tag_debug_0, 0))
        self.connect((self.blocks_conjugate_cc_0, 0), (self.sidekiq_sidekiq_tx_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "top_block")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_variable_function_probe_0(self):
        return self.variable_function_probe_0

    def set_variable_function_probe_0(self, variable_function_probe_0):
        self.variable_function_probe_0 = variable_function_probe_0
        self.set_variable_qtgui_label_0(self._variable_qtgui_label_0_formatter(self.variable_function_probe_0))

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_bandwidth(self.sample_rate)
        self.sidekiq_sidekiq_tx_0.set_tx_sample_rate(self.sample_rate)
        self.analog_sig_source_x_0.set_sampling_freq(self.sample_rate)

    def get_vector_length(self):
        return self.vector_length

    def set_vector_length(self, vector_length):
        self.vector_length = vector_length

    def get_variable_qtgui_label_0(self):
        return self.variable_qtgui_label_0

    def set_variable_qtgui_label_0(self, variable_qtgui_label_0):
        self.variable_qtgui_label_0 = variable_qtgui_label_0
        Qt.QMetaObject.invokeMethod(self._variable_qtgui_label_0_label, "setText", Qt.Q_ARG("QString", self.variable_qtgui_label_0))

    def get_suppress_tune_transients_chooser(self):
        return self.suppress_tune_transients_chooser

    def set_suppress_tune_transients_chooser(self, suppress_tune_transients_chooser):
        self.suppress_tune_transients_chooser = suppress_tune_transients_chooser
        self._suppress_tune_transients_chooser_callback(self.suppress_tune_transients_chooser)
        self.sidekiq_sidekiq_tx_0.set_tx_suppress_tune_transients(bool(self.suppress_tune_transients_chooser))

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
        self.sidekiq_sidekiq_tx_0.set_tx_frequency(self.center_freq)

    def get_bandwidth(self):
        return self.bandwidth

    def set_bandwidth(self, bandwidth):
        self.bandwidth = bandwidth
        self.sidekiq_sidekiq_tx_0.set_tx_bandwidth(self.bandwidth)

    def get_atten_quart_db(self):
        return self.atten_quart_db

    def set_atten_quart_db(self, atten_quart_db):
        self.atten_quart_db = atten_quart_db
        self.sidekiq_sidekiq_tx_0.set_tx_attenuation(self.atten_quart_db)


def main(top_block_cls=top_block, options=None):

    if StrictVersion("4.5.0") <= StrictVersion(Qt.qVersion()) < StrictVersion("5.0.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.aboutToQuit.connect(quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
