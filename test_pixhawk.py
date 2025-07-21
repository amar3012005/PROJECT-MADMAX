#!/usr/bin/env python3
"""
Simple script to test MAVLink communication with Pixhawk
"""
from pymavlink import mavutil
import time
import sys

def test_connection(device='/dev/ttyACM0', baud=115200):
    print(f"Connecting to {device} at {baud} baud...")
    
    try:
        # Connect to the autopilot
        master = mavutil.mavlink_connection(device, baud=baud)
        
        # Wait for the heartbeat message to confirm the connection
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        print(f"✅ Heartbeat received from system {master.target_system} component {master.target_component}")
        
        # Request stream rates
        print("Setting up data streams...")
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,  # 4 Hz
            1   # Start
        )
        
        # Wait for GPS data
        print("Waiting for GPS data (10 seconds)...")
        start_time = time.time()
        gps_received = False
        while time.time() - start_time < 10:
            msg = master.recv_match(type=['GPS_RAW_INT', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1)
            if msg is not None:
                print(f"✅ Received {msg.get_type()} message")
                if msg.get_type() == 'GPS_RAW_INT':
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0
                    print(f"   Position: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
                    gps_received = True
                    break
        
        if not gps_received:
            print("⚠️  No GPS data received within timeout period")
        
        # Check for system status
        print("Requesting system status...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,  # Message ID for SYS_STATUS
            0, 0, 0, 0, 0, 0  # Parameter 2-7 (not used)
        )
        
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if msg is not None:
            print(f"✅ System status received:")
            print(f"   Battery: {msg.voltage_battery/1000.0:.2f}V, {msg.current_battery/100.0:.2f}A")
            print(f"   CPU Load: {msg.load/10.0:.1f}%")
        else:
            print("⚠️  No system status received")
            
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    # Check if an alternative device was specified
    device = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    if test_connection(device):
        print("\n✅ MAVLink connection test successful")
    else:
        print("\n❌ MAVLink connection test failed")
