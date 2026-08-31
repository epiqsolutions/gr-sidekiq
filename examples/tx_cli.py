#!/usr/bin/env python3
#

"""Command-line Sidekiq TX example
"""

import signal
import threading
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser

from gnuradio import analog
from gnuradio import gr
from gnuradio import sidekiq
from gnuradio.eng_arg import eng_float, intx


class tx_tone_cli(gr.top_block):
    def __init__(
        self,
        card,
        topology,
        handle,
        sample_rate,
        bandwidth,
        frequency,
        attenuation,
        tone_freq,
        burst_tag,
        threads,
        buffer_size,
        cal_mode,
        min_output_buffer,
    ):
        gr.top_block.__init__(self, "tx_tone_cli", catch_exceptions=True)

        self.analog_sig_source_x_0 = analog.sig_source_c(
            sample_rate,
            analog.GR_COS_WAVE,
            tone_freq,
            1,
            0,
            0,
        )
        self.analog_sig_source_x_0.set_min_output_buffer(min_output_buffer)

        self.sidekiq_sidekiq_tx_0 = sidekiq.sidekiq_tx(
            card,
            topology,
            handle,
            sample_rate,
            bandwidth,
            frequency,
            attenuation,
            burst_tag,
            threads,
            buffer_size,
            cal_mode,
        )

        self.connect((self.analog_sig_source_x_0, 0), (self.sidekiq_sidekiq_tx_0, 0))


def build_arg_parser():
    parser = ArgumentParser(
        description="Transmit a tone with the Sidekiq TX block.",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--card", type=intx, default=0, help="Sidekiq card index")
    parser.add_argument("--topology", type=intx, default=0, help="Sidekiq topology index")
    parser.add_argument(
        "--handle",
        default="TxA1",
        help="Sidekiq TX handle/port; accepts 0-3, TxA1, TxB2, A1, B2",
    )
    parser.add_argument(
        "--sample-rate",
        type=eng_float,
        default=10e6,
        help="Transmit sample rate in samples/sec",
    )
    parser.add_argument(
        "--bandwidth",
        type=eng_float,
        default=None,
        help="Transmit bandwidth in Hz; defaults to 0.8 * sample rate",
    )
    parser.add_argument(
        "--frequency",
        type=eng_float,
        default=1002e6,
        help="Transmit LO frequency in Hz",
    )
    parser.add_argument(
        "--attenuation",
        type=eng_float,
        default=125,
        help="Transmit attenuation",
    )
    parser.add_argument(
        "--tone-freq",
        type=eng_float,
        default=2e6,
        help="Baseband tone frequency in Hz",
    )
    parser.add_argument(
        "--burst-tag",
        default="",
        help="Enable burst mode by naming the burst-length tag key",
    )
    parser.add_argument(
        "--threads",
        type=intx,
        default=1,
        help="TX worker thread count; values > 1 use async mode",
    )
    parser.add_argument(
        "--buffer-size",
        type=intx,
        default=1020,
        help="TX buffer size in samples",
    )
    parser.add_argument(
        "--cal-mode",
        type=intx,
        default=1,
        help="Calibration mode passed to the TX block",
    )
    parser.add_argument(
        "--run-tx-calibration",
        action="store_true",
        help="Trigger a manual TX calibration after the flowgraph starts",
    )
    parser.add_argument(
        "--min-output-buffer",
        type=intx,
        default=32764 * 2 * 2,
        help="Minimum GNU Radio output buffer for the tone source",
    )
    parser.add_argument(
        "--duration",
        type=eng_float,
        default=None,
        help="Optional runtime in seconds; otherwise runs until Ctrl-C",
    )
    return parser


def main():
    args = build_arg_parser().parse_args()
    bandwidth = args.bandwidth if args.bandwidth is not None else args.sample_rate * 0.8

    tb = tx_tone_cli(
        card=args.card,
        topology=args.topology,
        handle=args.handle,
        sample_rate=args.sample_rate,
        bandwidth=bandwidth,
        frequency=args.frequency,
        attenuation=args.attenuation,
        tone_freq=args.tone_freq,
        burst_tag=args.burst_tag,
        threads=args.threads,
        buffer_size=args.buffer_size,
        cal_mode=args.cal_mode,
        min_output_buffer=args.min_output_buffer,
    )

    stop_event = threading.Event()

    def sig_handler(sig=None, frame=None):
        stop_event.set()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    if args.run_tx_calibration:
        tb.sidekiq_sidekiq_tx_0.run_tx_cal(1)

    print(
        "Starting TX:"
        f" card={args.card}"
        f" topology={args.topology}"
        f" handle={args.handle}"
        f" sample_rate={args.sample_rate}"
        f" bandwidth={bandwidth}"
        f" frequency={args.frequency}"
        f" attenuation={args.attenuation}"
        f" tone_freq={args.tone_freq}"
    )

    try:
        if args.duration is None:
            stop_event.wait()
        else:
            stop_event.wait(float(args.duration))
    finally:
        tb.stop()
        tb.wait()


if __name__ == "__main__":
    main()
