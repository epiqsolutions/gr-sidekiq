import numpy as np
from gnuradio import gr
import pmt

class blk(gr.sync_block):
    """Attach packet_len and tx_time to the first sample of each run."""

    def __init__(self, burst_len=1, tx_time_seconds=0, tx_time_fraction=0.0):
        gr.sync_block.__init__(
            self,
            name='Timed Burst Tagger',
            in_sig=[np.complex64],
            out_sig=[np.complex64])
        self.burst_len = int(burst_len)
        self.tx_time_seconds = int(tx_time_seconds)
        self.tx_time_fraction = float(tx_time_fraction)
        self._tagged = False

    def start(self):
        self._tagged = False
        return True

    def set_burst_len(self, burst_len):
        self.burst_len = int(burst_len)

    def set_tx_time_seconds(self, seconds):
        self.tx_time_seconds = int(seconds)

    def set_tx_time_fraction(self, fraction):
        self.tx_time_fraction = float(fraction)

    def work(self, input_items, output_items):
        output_items[0][:] = input_items[0]
        if len(output_items[0]) and not self._tagged:
            offset = self.nitems_written(0)
            source = pmt.intern('tx_timed_burst')
            self.add_item_tag(
                0, offset, pmt.intern('packet_len'),
                pmt.from_uint64(self.burst_len), source)
            self.add_item_tag(
                0, offset, pmt.intern('tx_time'),
                pmt.make_tuple(
                    pmt.from_uint64(self.tx_time_seconds),
                    pmt.from_double(self.tx_time_fraction)),
                source)
            self._tagged = True
        return len(output_items[0])