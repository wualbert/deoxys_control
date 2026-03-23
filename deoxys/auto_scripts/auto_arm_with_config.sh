#!/bin/bash
. $(dirname "$0")/color_variables.sh
. $(dirname "$0")/fix_ld_issue.sh

printf "${BIRed} Make sure you are in the Performance Mode!!! ${Color_Off} \n"

RTOS_MODE=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)

echo $RTOS_MODE

for mode in $RTOS_MODE
do
    if [ "${mode}" = "powersave" ]; then
	printf "${BIRed} Not in Performance Mode, will cause errors for franka codebase!!! ${Color_Off} \n"
    fi
done

# Default config file - can be overridden by command line argument
CONFIG_FILE="${1:-config/charmander.yml}"

echo "Using configuration: $CONFIG_FILE"

CRASH_COUNT=0
MAX_CRASHES=5
CRASH_THRESHOLD_TIME=60

while true
do
    echo "$(date '+%Y-%m-%d %H:%M:%S') - Starting franka-interface with config: $CONFIG_FILE"
    START_TIME=$(date +%s)

    # Pass the config file to franka-interface
    bin/franka-interface "$CONFIG_FILE"
    EXIT_CODE=$?

    END_TIME=$(date +%s)
    RUNTIME=$((END_TIME - START_TIME))

    # Check if process exited too quickly (whether crash or normal exit)
    if [ $RUNTIME -lt $CRASH_THRESHOLD_TIME ]; then
        CRASH_COUNT=$((CRASH_COUNT + 1))

        if [ $EXIT_CODE -ne 0 ]; then
            echo "$(date '+%Y-%m-%d %H:%M:%S') - ERROR: franka-interface CRASHED with exit code $EXIT_CODE after only ${RUNTIME}s"
        else
            echo "$(date '+%Y-%m-%d %H:%M:%S') - WARNING: franka-interface exited with code 0 after only ${RUNTIME}s (should stay running!)"
        fi

        echo "$(date '+%Y-%m-%d %H:%M:%S') - Quick termination detected ($CRASH_COUNT/$MAX_CRASHES)"

        if [ $CRASH_COUNT -ge $MAX_CRASHES ]; then
            echo "$(date '+%Y-%m-%d %H:%M:%S') - FATAL: Too many quick terminations in succession."
            echo "$(date '+%Y-%m-%d %H:%M:%S') - franka-interface is not staying alive as expected."
            echo "$(date '+%Y-%m-%d %H:%M:%S') - Possible causes:"
            echo "  - No client connecting to listen for robot state"
            echo "  - Configuration issue causing immediate shutdown"
            echo "  - Network/communication problem with robot"
            echo "$(date '+%Y-%m-%d %H:%M:%S') - Stopping auto-restart. Please investigate."
            exit 1
        fi
    else
        # Reset crash counter if it ran for a while
        CRASH_COUNT=0
        if [ $EXIT_CODE -ne 0 ]; then
            echo "$(date '+%Y-%m-%d %H:%M:%S') - franka-interface crashed with exit code $EXIT_CODE after ${RUNTIME}s (long-running, resetting counter)"
        else
            echo "$(date '+%Y-%m-%d %H:%M:%S') - franka-interface exited normally after ${RUNTIME}s (long-running, resetting counter)"
        fi
    fi

    echo "$(date '+%Y-%m-%d %H:%M:%S') - Waiting 1 second before restart..."
    sleep 1
done