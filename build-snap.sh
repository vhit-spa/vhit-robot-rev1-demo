#!/bin/bash

set -e

./build-workspace.sh

snapcraft clean --destructive-mode

snapcraft pack --build-for=amd64 --verbosity=verbose --destructive-mode
