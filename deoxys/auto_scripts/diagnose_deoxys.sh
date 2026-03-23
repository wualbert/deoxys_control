#!/bin/bash

echo "============================================"
echo "Deoxys Diagnostic Script"
echo "$(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check CPU governor mode
echo -e "\n${YELLOW}Checking CPU Performance Mode:${NC}"
GOVERNORS=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort -u)
for gov in $GOVERNORS; do
    if [ "$gov" = "performance" ]; then
        echo -e "${GREEN}✓ CPU in performance mode${NC}"
    else
        echo -e "${RED}✗ CPU in $gov mode (should be performance)${NC}"
    fi
done

# Check network connectivity
echo -e "\n${YELLOW}Checking Network Connectivity:${NC}"

# Check NUC
echo -n "NUC (172.16.0.8): "
if ping -c 1 -W 1 172.16.0.8 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Reachable${NC}"
else
    echo -e "${RED}✗ Not reachable${NC}"
fi

# Check Robot
echo -n "Robot (172.16.0.6): "
if ping -c 1 -W 1 172.16.0.6 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Reachable${NC}"
else
    echo -e "${RED}✗ Not reachable${NC}"
fi

# Check PC
echo -n "PC (172.16.0.3): "
if ping -c 1 -W 1 172.16.0.3 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Reachable${NC}"
else
    echo -e "${RED}✗ Not reachable${NC}"
fi

# Check if franka-interface binary exists
echo -e "\n${YELLOW}Checking Deoxys Binary:${NC}"
if [ -f "bin/franka-interface" ]; then
    echo -e "${GREEN}✓ franka-interface binary exists${NC}"
    echo "  Binary info: $(file bin/franka-interface)"
    echo "  Size: $(du -h bin/franka-interface | cut -f1)"
else
    echo -e "${RED}✗ franka-interface binary not found${NC}"
fi

# Check for required libraries
echo -e "\n${YELLOW}Checking Required Libraries:${NC}"
if [ -f "bin/franka-interface" ]; then
    LDD_OUTPUT=$(ldd bin/franka-interface 2>&1)
    if echo "$LDD_OUTPUT" | grep -q "not found"; then
        echo -e "${RED}✗ Missing libraries:${NC}"
        echo "$LDD_OUTPUT" | grep "not found"
    else
        echo -e "${GREEN}✓ All required libraries found${NC}"
    fi
fi

# Check recent log files
echo -e "\n${YELLOW}Recent Log Entries:${NC}"
if [ -f "logs/debug.log" ]; then
    echo "Last 5 lines from debug.log:"
    tail -5 logs/debug.log
else
    echo "No debug.log found"
fi

if [ -f "logs/error.log" ]; then
    if [ -s "logs/error.log" ]; then
        echo "Last 5 lines from error.log:"
        tail -5 logs/error.log
    else
        echo "error.log exists but is empty"
    fi
else
    echo "No error.log found"
fi

# Check system resources
echo -e "\n${YELLOW}System Resources:${NC}"
echo "Memory usage:"
free -h | head -2

echo -e "\nCPU Load:"
uptime

# Check for core dumps
echo -e "\n${YELLOW}Checking for Core Dumps:${NC}"
CORE_PATTERN=$(cat /proc/sys/kernel/core_pattern)
echo "Core pattern: $CORE_PATTERN"

if [ -d "/var/crash" ]; then
    CRASHES=$(find /var/crash -name "*.crash" -mtime -1 2>/dev/null | wc -l)
    if [ $CRASHES -gt 0 ]; then
        echo -e "${RED}✗ Found $CRASHES crash files in last 24 hours${NC}"
    else
        echo -e "${GREEN}✓ No recent crash files${NC}"
    fi
fi

# Check if deoxys process is currently running
echo -e "\n${YELLOW}Current Deoxys Processes:${NC}"
DEOXYS_PROCS=$(pgrep -af "franka-interface" 2>/dev/null)
if [ -n "$DEOXYS_PROCS" ]; then
    echo "Found running processes:"
    echo "$DEOXYS_PROCS"
else
    echo "No franka-interface processes currently running"
fi

echo -e "\n${YELLOW}Recommendations:${NC}"
echo "1. If CPU is not in performance mode, run:"
echo "   sudo cpupower frequency-set -g performance"
echo ""
echo "2. If network connectivity fails, check:"
echo "   - Cable connections"
echo "   - Network interface configuration"
echo "   - Firewall settings"
echo ""
echo "3. To capture more detailed crash information:"
echo "   - Run franka-interface directly (not through auto_arm.sh)"
echo "   - Check dmesg for kernel messages: sudo dmesg | tail -50"
echo "   - Enable core dumps: ulimit -c unlimited"
echo ""
echo "4. To test without auto-restart:"
echo "   cd $(dirname $0)/.."
echo "   bin/franka-interface <your-config-file>"
echo "============================================"