from pymavlink import mavutil

# Connect to the Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for heartbeat to make sure connection is live
master.wait_heartbeat()
print(f"✅ Connected: System {master.target_system}, Component {master.target_component}")

# Continuously listen for GPS_RAW_INT messages
while True:
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        print(f"📍 Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")

