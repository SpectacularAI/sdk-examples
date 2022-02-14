#!/usr/bin/env python3

"""Plot position trajectory from JSONL output."""

import json
import sys

import numpy as np

POSITION = 'position'

def read_jsonl(fn):
    with open(fn) as f:
        for l in f: yield(json.loads(l))

def read_data(fn):
    pos = []
    for o in read_jsonl(fn):
        if POSITION not in o: continue
        pos.append([o[POSITION][c] for c in 'xyz'])
    return np.array(pos)

if __name__ == '__main__':
    import argparse
    import matplotlib.pyplot as plt

    p = argparse.ArgumentParser(__doc__)
    p.add_argument('jsonl', help='VIO output file')
    p.add_argument('-image', help='Save image to this path instead of showing the plot')
    args = p.parse_args()

    pos = read_data(args.jsonl)
    if pos.size == 0:
        print("No data to plot.")
        sys.exit()

    plt.plot(pos[:,0], pos[:,1])
    plt.axis('equal')

    if args.image:
        plt.savefig(args.image)
    else:
        plt.show()
