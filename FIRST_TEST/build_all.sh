#!/bin/bash

# ESP-IDF Build Script for FIRST_TEST (ESP32-P4 + LVGL)
# Usage: ./build_all.sh [command]
# Commands: build (default), clean, fullclean, flash, monitor, all

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ESP32-P4 LVGL Build Script${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if IDF_PATH is set
if [ -z "$IDF_PATH" ]; then
    echo -e "${YELLOW}IDF_PATH not set. Attempting to source ESP-IDF...${NC}"

    # Try common ESP-IDF locations
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        source "$HOME/esp/esp-idf/export.sh"
    elif [ -f "/opt/esp/idf/export.sh" ]; then
        source "/opt/esp/idf/export.sh"
    elif [ -f "$HOME/.espressif/esp-idf/export.sh" ]; then
        source "$HOME/.espressif/esp-idf/export.sh"
    else
        echo -e "${RED}ERROR: Could not find ESP-IDF. Please set IDF_PATH or source export.sh${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}Using ESP-IDF at: $IDF_PATH${NC}"
echo ""

# Parse command
COMMAND=${1:-build}

case "$COMMAND" in
    build)
        echo -e "${YELLOW}Building project...${NC}"
        idf.py build
        echo -e "${GREEN}Build complete!${NC}"
        ;;
    clean)
        echo -e "${YELLOW}Cleaning build...${NC}"
        idf.py clean
        echo -e "${GREEN}Clean complete!${NC}"
        ;;
    fullclean)
        echo -e "${YELLOW}Full clean (removing build directory)...${NC}"
        idf.py fullclean
        echo -e "${GREEN}Full clean complete!${NC}"
        ;;
    flash)
        echo -e "${YELLOW}Flashing to device...${NC}"
        idf.py flash
        echo -e "${GREEN}Flash complete!${NC}"
        ;;
    monitor)
        echo -e "${YELLOW}Starting serial monitor...${NC}"
        idf.py monitor
        ;;
    all)
        echo -e "${YELLOW}Building, flashing, and monitoring...${NC}"
        idf.py build flash monitor
        ;;
    menuconfig)
        echo -e "${YELLOW}Opening menuconfig...${NC}"
        idf.py menuconfig
        ;;
    size)
        echo -e "${YELLOW}Showing size info...${NC}"
        idf.py size
        ;;
    *)
        echo -e "${RED}Unknown command: $COMMAND${NC}"
        echo ""
        echo "Available commands:"
        echo "  build      - Build the project (default)"
        echo "  clean      - Clean build artifacts"
        echo "  fullclean  - Remove entire build directory"
        echo "  flash      - Flash to connected device"
        echo "  monitor    - Open serial monitor (Ctrl+] to exit)"
        echo "  all        - Build, flash, and monitor"
        echo "  menuconfig - Open ESP-IDF configuration menu"
        echo "  size       - Show binary size information"
        exit 1
        ;;
esac
