#!/usr/bin/env bash
# Mac / Linux: pornește harta. Pentru date simulate: ./porneste.sh --demo
set -e
cd "$(dirname "$0")/server"

if ! command -v python3 >/dev/null; then
  echo "Python 3 nu este instalat: https://www.python.org/downloads/"
  exit 1
fi

if [ ! -x .venv/bin/python ]; then
  echo "Prima pornire: pregătesc mediul, durează un minut..."
  python3 -m venv .venv
  .venv/bin/python -m pip install --quiet --disable-pip-version-check -r requirements.txt
fi

exec .venv/bin/python run.py "$@"
