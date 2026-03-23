#!/usr/bin/env python3
"""
Monitor ZMQ messages being sent to the franka-interface on the NUC.
This will help identify what's causing the rapid controller switching.
"""

import zmq
import time
from deoxys import config_root
from deoxys.utils import YamlConfig

def monitor_messages():
    # Load configuration
    config_path = config_root + "/charmander.yml"
    config = YamlConfig(config_path).as_easydict()

    print(f"Monitoring ZMQ messages on {config.NUC.IP}:{config.NUC.SUB_PORT}")
    print("This will show what commands are being sent to franka-interface...")
    print("-" * 60)

    # Create ZMQ subscriber to listen to the same port as franka-interface
    context = zmq.Context()
    socket = context.socket(zmq.SUB)

    # Connect to the publisher port (where commands are sent)
    socket.connect(f"tcp://{config.NUC.IP}:{config.NUC.SUB_PORT}")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

    print("Listening for messages...")

    message_count = 0
    controller_types = {}
    last_print_time = time.time()

    try:
        while True:
            # Try to receive message with timeout
            if socket.poll(100):  # 100ms timeout
                message = socket.recv()
                message_count += 1

                # Try to parse message to identify controller type
                try:
                    # This is a protobuf message, try to decode what we can
                    msg_str = str(message)

                    # Look for controller type indicators in the binary data
                    if b'JOINT_POSITION' in message:
                        controller_type = "JOINT_POSITION"
                    elif b'JOINT_IMPEDANCE' in message:
                        controller_type = "JOINT_IMPEDANCE"
                    elif b'OSC_POSE' in message:
                        controller_type = "OSC_POSE"
                    elif b'OSC_POSITION' in message:
                        controller_type = "OSC_POSITION"
                    elif b'CARTESIAN_VELOCITY' in message:
                        controller_type = "CARTESIAN_VELOCITY"
                    else:
                        controller_type = "UNKNOWN"

                    controller_types[controller_type] = controller_types.get(controller_type, 0) + 1

                    print(f"[{time.strftime('%H:%M:%S')}] Message #{message_count}: Controller={controller_type}, Size={len(message)} bytes")

                except Exception as e:
                    print(f"[{time.strftime('%H:%M:%S')}] Message #{message_count}: Could not parse (size={len(message)})")

            # Print summary every 5 seconds
            if time.time() - last_print_time > 5:
                if controller_types:
                    print("\n--- Summary ---")
                    for ct, count in controller_types.items():
                        print(f"  {ct}: {count} messages")
                    print(f"  Total: {message_count} messages")
                    print("-" * 60)
                last_print_time = time.time()

    except KeyboardInterrupt:
        print("\n\nStopping monitor...")
        print(f"Total messages received: {message_count}")
        if controller_types:
            print("Controller type breakdown:")
            for ct, count in sorted(controller_types.items()):
                print(f"  {ct}: {count} messages ({100*count/message_count:.1f}%)")

    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    monitor_messages()