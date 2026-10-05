#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
echo 'Analysis ready: .venv/bin/python identify.py --latest'
