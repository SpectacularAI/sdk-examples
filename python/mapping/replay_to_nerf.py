#!/usr/bin/env python
"""
Post-process data in Spectacular AI format and convert it to input
for NeRF or Gaussian Splatting methods.
"""

DEPRECATION_NOTE = """
Note: the replay_to_nerf.py script has been replaced by the sai-cli
tool in Spectacular AI Python package v1.25. Prefer

    sai-cli process [args]

as a drop-in replacement of

    python replay_to_nerf.py [args]
.
"""

# The code is still available and usable as a stand-alone script, see:
# https://github.com/SpectacularAI/sdk/blob/main/python/cli/process/process.py

import_success = False
try:
    from spectacularAI.cli.process.process import process, define_args
    import_success = True
except ImportError as e:
    print(e)

if not import_success:
    msg = """

        Unable to import new Spectacular AI CLI, please update to SDK version >= 1.25"
    """
    raise RuntimeError(msg)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=DEPRECATION_NOTE,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    define_args(parser)
    print(DEPRECATION_NOTE)
    process(parser.parse_args())
