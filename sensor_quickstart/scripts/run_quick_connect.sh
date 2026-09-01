#!/usr/bin/env bash
set -euo pipefail

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
TOP_DIR="$(dirname "$PARENT_DIR")"
VENV_DIR="$SCRIPT_DIR/.venvSimpleCheck"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "Activating virtual environment..."
    if ! source "$VENV_DIR/bin/activate" 2>/dev/null; then
        echo -e "${YELLOW}Warning: Failed to activate virtual environment, please run quick install script first.${NC}"
        exit
    fi

echo ""
echo "=========================================="
echo "Starting Sensor"
echo "=========================================="
echo ""
echo "Using Python from: $(which python3)"
echo "Virtual environment: ${VIRTUAL_ENV:-Not in venv}"
echo ""

# Step 7: Run the sensor checker
cd "$PARENT_DIR"
python3 sensor_monitor.py "$@"

# Cleanup message
echo ""
echo "=========================================="
echo "Sensor stopped."
echo "=========================================="
if [ -n "${VIRTUAL_ENV:-}" ]; then
    echo "Deactivating virtual environment..."
    deactivate 2>/dev/null || true
    echo -e "${GREEN}✓ Virtual environment deactivated${NC}"
else
    echo "No virtual environment was active."
fi
echo "Done."
