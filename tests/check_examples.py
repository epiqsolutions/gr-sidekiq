#!/usr/bin/env python3
"""Compile every example with this checkout's GRC block definitions."""

import ast
import sys
import tempfile
from pathlib import Path

from gnuradio import gr
from gnuradio.grc.core import Constants
from gnuradio.grc.core.platform import Platform


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    sources = sorted((root / "examples").rglob("*.grc"))
    if not sources:
        raise RuntimeError("No GRC examples found")
    for source in sorted((root / "examples").rglob("*.py")):
        tree = ast.parse(source.read_text(), filename=str(source))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and
                isinstance(node.func, ast.Attribute) and
                node.func.attr == "sidekiq_rx" and
                isinstance(node.func.value, ast.Name) and
                node.func.value.id == "sidekiq" and
                len(node.args) != 19):
                raise RuntimeError(f"Stale RX constructor in {source}:{node.lineno}")
    with tempfile.TemporaryDirectory(prefix="gr_sidekiq_examples_") as temporary:
        output = Path(temporary)
        # GNU Radio's normal cache is under the user's home directory.
        Constants.GRC_SUBDIR = str(output)
        platform = Platform(
            name="gr-sidekiq example validation",
            prefs=gr.prefs(),
            version=gr.version(),
            version_parts=(gr.major_version(), gr.api_version(), gr.minor_version()),
        )
        platform.build_library(
            path=[*platform.config.block_paths, str(root / "grc")]
        )
        for source in sources:
            destination = output / source.stem
            destination.mkdir()
            _, generated = platform.load_and_generate_flow_graph(
                str(source), str(destination)
            )
            if not generated:
                raise RuntimeError(f"GRC could not generate {source}")
            text = Path(generated).read_text()
            compile(text, str(generated), "exec")
    print(f"Validated {len(sources)} GRC examples")
    return 0


if __name__ == "__main__":
    sys.exit(main())
