#!/bin/bash
set -ex

git clone https://github.com/luxonis/depthai-python
cd depthai-python/examples
python install_requirements.py -sdai
cp -R models ../..
cd ../..
