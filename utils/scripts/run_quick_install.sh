#!/usr/bin/env bash
set -euo pipefail

echo "=========================================="
echo "Robotiq Tactile Sensor Environment Setup"
echo "=========================================="
echo ""

# Get directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOP_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
PYTHON_DIR="$TOP_DIR/python"
VENV_DIR="$PYTHON_DIR/.venv"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if python3-venv is installed
check_venv_package() {
    echo "Checking for python3-venv package..."
    if ! dpkg -l | grep -q python3-venv 2>/dev/null && ! python3 -m venv --help &>/dev/null; then
        echo -e "${YELLOW}python3-venv not found. Installing...${NC}"
        sudo apt-get update
        sudo apt-get install -y python3-venv
        echo -e "${GREEN}✓ python3-venv installed${NC}"
    else
        echo -e "${GREEN}✓ python3-venv is available${NC}"
    fi
}

# Function to create/activate virtual environment
setup_venv() {
    if [ ! -d "$VENV_DIR" ]; then
        echo ""
        echo "Creating virtual environment at $VENV_DIR..."
        python3 -m venv "$VENV_DIR"
        echo -e "${GREEN}✓ Virtual environment created${NC}"
    else
        echo -e "${GREEN}✓ Virtual environment already exists${NC}"
    fi

    echo "Activating virtual environment..."
    # Disable nounset temporarily while sourcing activate
    set +u
    if ! source "$VENV_DIR/bin/activate" 2>/dev/null; then
        echo -e "${YELLOW}Warning: Failed to activate virtual environment, recreating...${NC}"
        rm -rf "$VENV_DIR"
        python3 -m venv "$VENV_DIR"
        source "$VENV_DIR/bin/activate"
    fi
    set -u

    echo -e "${GREEN}✓ Virtual environment activated${NC}"
    echo "  Python location: $(which python3)"
    echo "  Python version:  $(python3 --version)"
}

# Function to install python package
install_package() {
    echo ""
    echo "Installing requirements and Python package..."

    pip install --upgrade pip --quiet

    if [ -f "$PYTHON_DIR/requirements.txt" ]; then
        pip install -r "$PYTHON_DIR/requirements.txt" --quiet
        echo -e "${GREEN}✓ Requirements installed from requirements.txt${NC}"
    fi

    if [ -f "$PYTHON_DIR/setup.py" ]; then
        pip install -e "$PYTHON_DIR" --no-deps --quiet
        echo -e "${GREEN}✓ Installed robotiq_tactile package (-e)${NC}"
    fi
}

# Load helper scripts
echo "Loading helper scripts..."
if [ -f "${TOP_DIR}/utils/scripts/apply_udev_rule.sh" ]; then
    source "${TOP_DIR}/utils/scripts/apply_udev_rule.sh"
    echo -e "${GREEN}✓ Loaded apply_udev_rule.sh${NC}"
else
    echo -e "${YELLOW}Warning: apply_udev_rule.sh not found, skipping...${NC}"
    apply_udev_rule() { :; }
fi

if [ -f "${TOP_DIR}/utils/scripts/set_sensor_permissions.sh" ]; then
    source "${TOP_DIR}/utils/scripts/set_sensor_permissions.sh"
    echo -e "${GREEN}✓ Loaded set_sensor_permissions.sh${NC}"
else
    echo -e "${YELLOW}Warning: set_sensor_permissions.sh not found, skipping...${NC}"
    set_sensor_permissions() { :; }
fi

if [ -f "${TOP_DIR}/utils/scripts/find_sensor_devices.sh" ]; then
    source "${TOP_DIR}/utils/scripts/find_sensor_devices.sh"
    echo -e "${GREEN}✓ Loaded find_sensor_devices.sh${NC}"
else
    echo -e "${YELLOW}Warning: find_sensor_devices.sh not found, skipping...${NC}"
    find_sensor_devices() { echo ""; }
fi

echo ""
echo "=========================================="
echo "Setting Up Environment"
echo "=========================================="

# Step 1: Check for venv package
check_venv_package

# Step 2: Setup virtual environment
setup_venv

# Step 3: Install python package
install_package

echo ""
echo "=========================================="
echo "Configuring Sensor Permissions"
echo "=========================================="

# Step 4: Apply udev rules
echo ""
echo "[1/3] Applying udev rules..."
apply_udev_rule

# Step 5: Set sensor permissions
echo ""
echo "[2/3] Setting sensor permissions..."
set_sensor_permissions

# Step 6: Find sensor devices
echo ""
echo "[3/3] Finding sensor devices..."
sensor_devices=($(find_sensor_devices))

if ((${#sensor_devices[@]} == 0)); then
    echo ""
    echo -e "${YELLOW}=========================================="
    echo "Warning: No sensor devices detected"
    echo "==========================================${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure the sensor is plugged in"
    echo "2. Try unplugging and replugging the sensor"
    echo "3. Check if you're in the dialout group: groups"
    echo "4. You may need to log out and back in"
    echo ""
    read -p "Continue anyway? (y/N): " response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Exiting..."
        exit 1
    fi
else
    echo -e "${GREEN}✓ Found ${#sensor_devices[@]} sensor device(s):${NC}"
    for dev in "${sensor_devices[@]}"; do
        echo "  - $dev"
    done
fi

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo "To activate your environment, run:"
echo -e "${GREEN}  source $VENV_DIR/bin/activate${NC}"
