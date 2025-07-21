#!/usr/bin/env python3
"""
Simple focused script to test MAVLink communication with Pixhawk
This script will try different settings to establish communication
"""
from pymavlink import mavutil
import time
import sys
import os

# Debug mode - set to True to see more output
DEBUG = True

def log(msg):
    """Print a log message with timestamp"""
    if DEBUG:
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")

def try_connection(device, baud_rate=115200, timeout=5):
    """Try to connect with specific settings"""
    log(f"Attempting connection to {device} at {baud_rate} baud...")
    
    try:
        # Connect with the specified parameters
        master = mavutil.mavlink_connection(
            device,
            baud=baud_rate,
            source_system=255,  # Using 255 as source (Ground Control)
            source_component=0,
            autoreconnect=True,
            force_connected=False
        )
        
        # Wait for heartbeat with timeout
        log("Waiting for heartbeat...")
        start_time = time.time()
        heartbeat_received = False
        
        while time.time() - start_time < timeout:
            msg = master.recv_msg()
            if msg is not None and msg.get_type() == "HEARTBEAT":
                heartbeat_received = True
                log(f"✅ Heartbeat from system {msg.get_srcSystem()}, component {msg.get_srcComponent()}")
                log(f"   Type: {msg.type}, Autopilot: {msg.autopilot}, Base mode: {msg.base_mode:08b}")
                break
            time.sleep(0.1)
        
        if not heartbeat_received:
            log("❌ No heartbeat received within timeout")
            return False
            
        # Request parameter list
        log("Requesting parameters...")
        master.param_fetch_all()
        
        # Try to get some parameters
        start_time = time.time()
        param_received = False
        
        while time.time() - start_time < timeout:
            msg = master.recv_msg()
            if msg is not None and msg.get_type() == "PARAM_VALUE":
                param_received = True
                log(f"✅ Parameter received: {msg.param_id} = {msg.param_value}")
                break
            time.sleep(0.1)
            
        if not param_received:
            log("⚠️ No parameters received within timeout")
            
        # Try to get GPS data
        log("Requesting GPS data...")
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10,  # 10 Hz
            1    # Start
        )
        
        start_time = time.time()
        gps_received = False
        
        while time.time() - start_time < timeout:
            msg = master.recv_msg()
            if msg is not None and msg.get_type() in ["GPS_RAW_INT", "GLOBAL_POSITION_INT"]:
                gps_received = True
                if msg.get_type() == "GPS_RAW_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0
                    log(f"✅ GPS data: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
                else:
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000.0
                    log(f"✅ Position data: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m")
                break
            time.sleep(0.1)
            
        if not gps_received:
            log("⚠️ No GPS data received within timeout")
            
        return heartbeat_received
        
    except Exception as e:
        log(f"❌ Connection error: {str(e)}")
        return False
    finally:
        try:
            master.close()
        except:
            pass

def main():
    """Main function - try different connection methods"""
    
    # Default device to try
    default_device = '/dev/ttyACM0'
    
    # Try to find available devices
    log("Looking for possible Pixhawk connection points...")
    possible_devices = []
    
    # Check for ACM devices (common for PX4/Pixhawk over USB)
    for i in range(10):
        acm_device = f'/dev/ttyACM{i}'
        if os.path.exists(acm_device):
            possible_devices.append(acm_device)
            log(f"Found potential device: {acm_device}")
    
    # Check for USB-to-Serial devices (sometimes used for Pixhawk)
    for i in range(10):
        usb_device = f'/dev/ttyUSB{i}'
        if os.path.exists(usb_device):
            possible_devices.append(usb_device)
            log(f"Found potential device: {usb_device}")
    
    # Try serial ports (for direct UART connections)
    for i in range(10):
        serial_device = f'/dev/ttyS{i}'
        if os.path.exists(serial_device):
            possible_devices.append(serial_device)
            log(f"Found potential device: {serial_device}")
    
    # Add any custom device from command line
    if len(sys.argv) > 1:
        custom_device = sys.argv[1]
        if custom_device not in possible_devices:
            possible_devices.insert(0, custom_device)
    
    # Make sure default is first if available
    if default_device in possible_devices:
        possible_devices.remove(default_device)
        possible_devices.insert(0, default_device)
    
    # Common baud rates for Pixhawk
    baud_rates = [115200, 57600, 921600, 1500000]
    
    # Try each device
    successful_connection = False
    for device in possible_devices:
        for baud in baud_rates:
            if try_connection(device, baud):
                log(f"✅ Successfully connected to {device} at {baud} baud!")
                print(f"\n### SUCCESS ###")
                print(f"Device: {device}")
                print(f"Baud rate: {baud}")
                print(f"For run_flight04.py, use these settings in the mavutil.mavlink_connection() function.")
                successful_connection = True
                break
        
        if successful_connection:
            break
    
    if not successful_connection:
        print("\n### NO CONNECTION ESTABLISHED ###")
        print("Troubleshooting tips:")
        print("1. Check that the Pixhawk is powered on")
        print("2. Confirm the USB cable is firmly connected")
        print("3. Verify that the user has permission to access the serial port (you're in the dialout group)")
        print("4. Try rebooting the Pixhawk")
        print("5. Check if QGroundControl or Mission Planner is already connected to the device")

if __name__ == "__main__":
    main()
