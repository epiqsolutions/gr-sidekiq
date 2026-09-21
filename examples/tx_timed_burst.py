#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: Sidekiq Timed TX Burst
# Author: Epiq Solutions
# Description: One RF-timestamped Sidekiq TX burst using UHD-compatible tx_time and length tags.
# GNU Radio version: v3.11.0.0git-605-g9b22fd38

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
from gnuradio import sidekiq
import tx_timed_burst_timed_burst_tagger as timed_burst_tagger  # embedded python block




class tx_timed_burst(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Sidekiq Timed TX Burst", catch_exceptions=True)

        ##################################################
        # Variables
        ##################################################
        self.sample_rate = sample_rate = 20e6
        self.tx_time_seconds = tx_time_seconds = 5
        self.tx_time_fraction = tx_time_fraction = 0.0
        self.tone_frequency = tone_frequency = 2e6
        self.frequency = frequency = 1000e6
        self.burst_len = burst_len = int(1.5*sample_rate)

        ##################################################
        # Blocks
        ##################################################

        self.timed_burst_tagger = timed_burst_tagger.blk(burst_len=burst_len, tx_time_seconds=tx_time_seconds, tx_time_fraction=tx_time_fraction)
        self.sidekiq_sidekiq_tx_0 = sidekiq.sidekiq_tx(1, 0, 0, sample_rate, (0.8*sample_rate), frequency, 100, 'packet_len', 1, 4092, 0, 1, 1)
        self.blocks_head_0 = blocks.head(gr.sizeof_gr_complex*1, burst_len)
        self.analog_sig_source_x_0 = analog.sig_source_c(sample_rate, analog.GR_COS_WAVE, tone_frequency, 0.9, 0, 0)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_head_0, 0))
        self.connect((self.blocks_head_0, 0), (self.timed_burst_tagger, 0))
        self.connect((self.timed_burst_tagger, 0), (self.sidekiq_sidekiq_tx_0, 0))


    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_burst_len(int(1.5*self.sample_rate))
        self.analog_sig_source_x_0.set_sampling_freq(self.sample_rate)
        self.sidekiq_sidekiq_tx_0.set_tx_sample_rate(self.sample_rate)
        self.sidekiq_sidekiq_tx_0.set_tx_bandwidth((0.8*self.sample_rate))

    def get_tx_time_seconds(self):
        return self.tx_time_seconds

    def set_tx_time_seconds(self, tx_time_seconds):
        self.tx_time_seconds = tx_time_seconds
        self.timed_burst_tagger.tx_time_seconds = self.tx_time_seconds

    def get_tx_time_fraction(self):
        return self.tx_time_fraction

    def set_tx_time_fraction(self, tx_time_fraction):
        self.tx_time_fraction = tx_time_fraction
        self.timed_burst_tagger.tx_time_fraction = self.tx_time_fraction

    def get_tone_frequency(self):
        return self.tone_frequency

    def set_tone_frequency(self, tone_frequency):
        self.tone_frequency = tone_frequency
        self.analog_sig_source_x_0.set_frequency(self.tone_frequency)

    def get_frequency(self):
        return self.frequency

    def set_frequency(self, frequency):
        self.frequency = frequency
        self.sidekiq_sidekiq_tx_0.set_tx_frequency(self.frequency)

    def get_burst_len(self):
        return self.burst_len

    def set_burst_len(self, burst_len):
        self.burst_len = burst_len
        self.blocks_head_0.set_length(self.burst_len)
        self.timed_burst_tagger.burst_len = self.burst_len




def main(top_block_cls=tx_timed_burst, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    tb.wait()


if __name__ == '__main__':
    main()
