#!/usr/bin/env python3
"""
Enhanced script to test MAVLink communication with Pixhawk
"""
from pymavlink import mavutil
import time
import sys
import signal

# Handle timeout
def handler(signum, frame):
    print("\n⚠️  Timeout reached. Script execution terminated.")
    sys.exit(1)

def test_connection(device='/dev/ttyACM0', baud=115200, timeout=30):
    print(f"Connecting to {device} at {baud} baud...")
    
    try:
        # Connect to the autopilot
        master = mavutil.mavlink_connection(device, baud=baud)
        
        # Wait for the heartbeat message to confirm the connection
        print("Waiting for heartbeat...")
        heartbeat = master.wait_heartbeat(timeout=10)
        if heartbeat:
            print(f"✅ Heartbeat received from system {master.target_system} component {master.target_component}")
            print(f"   Autopilot type: {master.mav.WIRE_PROTOCOL_VERSION}")
        else:
            print("❌ No heartbeat received, check connections")
            return False
        
        # Request all data streams
        print("Setting up data streams...")
        for stream_id in range(0, 6):  # All available data streams
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                stream_id,
                10,  # 10 Hz
                1   # Start
            )
        
        # Dictionary to track received message types
        received_messages = {}
        
        print(f"\nListening for MAVLink messages for {timeout} seconds...")
        print("Press Ctrl+C to stop listening early\n")
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Non-blocking receive with a short timeout
            msg = master.recv_match(blocking=True, timeout=0.1)
            if msg:
                msg_type = msg.get_type()
                # Only print the first instance of each message type
                if msg_type not in received_messages:
                    received_messages[msg_type] = 1
                    print(f"✅ Received message: {msg_type}")
                    
                    # Print some details for specific message types
                    if msg_type == 'HEARTBEAT':
                        print(f"   Mode: {master.flightmode}")
                    elif msg_type == 'GPS_RAW_INT':
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        alt = msg.alt / 1000.0
                        fix_type = msg.fix_type
                        fix_status = ["No GPS", "No Fix", "2D Fix", "3D Fix", "DGPS", "RTK Float", "RTK Fixed"][min(fix_type, 6)]
                        print(f"   GPS: {fix_status}, Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m, Satellites: {msg.satellites_visible}")
                    elif msg_type == 'SYS_STATUS':
                        print(f"   Battery: {msg.voltage_battery/1000.0:.2f}V, Current: {msg.current_battery/100.0:.2f}A")
                        print(f"   CPU Load: {msg.load/10.0:.1f}%")
                    elif msg_type == 'ATTITUDE':
                        print(f"   Roll: {msg.roll*57.3:.1f}°, Pitch: {msg.pitch*57.3:.1f}°, Yaw: {msg.yaw*57.3:.1f}°")
                else:
                    received_messages[msg_type] += 1
        
        # Summary
        print("\n--- Summary of received messages ---")
        total_msgs = sum(received_messages.values())
        print(f"Total messages received: {total_msgs}")
        print(f"Unique message types: {len(received_messages)}")
        
        if total_msgs > 0:
            print("\nTop message types by frequency:")
            sorted_msgs = sorted(received_messages.items(), key=lambda x: x[1], reverse=True)
            for msg_type, count in sorted_msgs[:10]:  # Show top 10
                print(f"  {msg_type}: {count} messages")
                
        critical_msgs = {'HEARTBEAT', 'GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'SYS_STATUS', 'ATTITUDE'}
        missing = critical_msgs - received_messages.keys()
        
        if missing:
            print("\n⚠️  Missing critical message types:")
            for msg in missing:
                print(f"  - {msg}")
        
        return len(received_messages) > 0
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    # Set a timeout for the entire script
    signal.signal(signal.SIGALRM, handler)
    signal.alarm(40)  # 40 second timeout for the entire script
    
    # Check if an alternative device was specified
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    if test_connection(device, timeout=30):
        print("\n✅ MAVLink connection test successful")
    else:
        print("\n❌ MAVLink connection test failed")
