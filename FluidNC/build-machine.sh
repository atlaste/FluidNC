#!/bin/bash
# Quick build script for FluidNC with machine configurations
# Usage: ./build-machine.sh <machine_config> [additional idf.py args]
# Example: ./build-machine.sh lathe.yaml flash monitor

if [ -z "$1" ]; then
    echo "Usage: $0 <machine_config> [additional idf.py args]"
    echo ""
    echo "Examples:"
  echo "  $0 lathe.yaml build flash"
    echo "  $0 mill.yaml build flash monitor"
    echo "  $0 machine/custom.yaml build"
    echo ""
    echo "Available machine configs in machine/ directory:"
    ls -1 machine/*.yaml 2>/dev/null | sed 's/machine\//  /'
    exit 1
fi

MACHINE_CONFIG="$1"
shift

echo "Building FluidNC with machine configuration: $MACHINE_CONFIG"
echo "Additional arguments: $@"
echo ""

idf.py -DMACHINE="$MACHINE_CONFIG" "$@"
