"""Record data for later playback from OAK devices"""

DEPRECATION_NOTE = """
Note: the oak/vio_record.py script has been replaced by the sai-cli
tool in Spectacular AI Python package v1.25. Prefer

    sai-cli record oak [args]

as a drop-in replacement of

    python vio_record.py [args]

For more information, type: sai-cli record oak --help
"""

# The code is still available and usable as a stand-alone script, see:
# https://github.com/SpectacularAI/sdk/blob/main/python/cli/record/oak.py

import_success = False
try:
    from spectacularAI.cli.record.oak import record, define_args
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
    record(parser.parse_args())