#!/usr/bin/env python3
"""
Script to connect to a running PX4 SITL instance using MAVLink
"""
from pymavlink import mavutil
import time
import sys
import signal

# Set up a signal handler for clean exit
def signal_handler(sig, frame):
    print("\nExiting...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def connect_to_sitl():
    # SITL default connection details
    connection_string = "udp:localhost:14540"
    
    print(f"Connecting to PX4 SITL on {connection_string}...")
    try:
        # Connect to the vehicle
        master = mavutil.mavlink_connection(connection_string)
        
        # Wait for the heartbeat message to confirm the connection
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        print(f"✅ Heartbeat received from system {master.target_system} component {master.target_component}")
        print(f"   Vehicle type: {master.mav_type}, Autopilot: {master.mav_autopilot}")
        print(f"   System status: {master.system_status}, MAVLink version: {master.mav_version}")
        
        # Request all data streams
        print("Setting up data streams...")
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10 Hz
            1    # Start
        )
        
        # Main loop - receiving and displaying messages
        print("\nReceiving telemetry data: (Press Ctrl+C to exit)")
        print("-" * 80)
        
        msg_counts = {}
        
        while True:
            # Wait for a message
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg:
                msg_type = msg.get_type()
                
                if msg_type not in msg_counts:
                    msg_counts[msg_type] = 0
                msg_counts[msg_type] += 1
                
                # Print specific data for interesting message types
                if msg_type == "HEARTBEAT":
                    mode = mavutil.mode_string_v10(msg)
                    is_armed = "ARMED" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
                    print(f"Mode: {mode}, Status: {is_armed}")
                    
                elif msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7  # Convert to degrees
                    lon = msg.lon / 1e7
                    alt = msg.alt / 1000  # Convert to meters
                    rel_alt = msg.relative_alt / 1000
                    vx = msg.vx / 100  # Convert to m/s
                    vy = msg.vy / 100
                    vz = msg.vz / 100
                    hdg = msg.hdg / 100  # Convert to degrees
                    print(f"Position: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m (Rel: {rel_alt:.2f}m)")
                    print(f"Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} m/s, Heading: {hdg:.1f}°")
                    
                elif msg_type == "ATTITUDE":
                    roll = msg.roll * 57.3  # Convert to degrees
                    pitch = msg.pitch * 57.3
                    yaw = msg.yaw * 57.3
                    print(f"Attitude: Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°")
                    
                elif msg_type == "SYS_STATUS":
                    voltage = msg.voltage_battery / 1000.0  # Convert to volts
                    current = msg.current_battery / 100.0   # Convert to amps
                    battery_remaining = msg.battery_remaining  # Percentage
                    print(f"Battery: {voltage:.2f}V, {current:.2f}A, {battery_remaining}% remaining")
                
                # Print summary every 50 heartbeats
                if msg_type == "HEARTBEAT" and msg_counts["HEARTBEAT"] % 50 == 0:
                    print("\n--- Message Statistics ---")
                    for mtype, count in sorted(msg_counts.items(), key=lambda x: x[1], reverse=True)[:10]:
                        print(f"{mtype}: {count} messages")
                    print("-" * 30)
            
            # Sleep to avoid saturating the CPU
            time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    connect_to_sitl()
