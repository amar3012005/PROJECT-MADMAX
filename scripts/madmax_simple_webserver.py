#!/usr/bin/env python3

from flask import Flask, render_template, send_from_directory, jsonify, request
import json
import os
import random
import threading
import time
from datetime import datetime

app = Flask(__name__, static_folder="static", template_folder=".")

# Simulation data (this would eventually be connected to your actual drone data)
drone_status = {
    "armed": False,
    "mode": "HOLD",
    "battery": 95.4,
    "altitude": 0.0,         # Relative altitude (AGL - Above Ground Level)
    "altitude_asl": 15.0,    # Absolute altitude (ASL - Above Sea Level, starting at 15m)
    "position": {"lat": 47.641468, "lon": -122.140165},
    "connected": True,
    "gps_satellites": 10,
    "signal_strength": 87,
    "cpu_temp": 54,
    "memory_usage": 32,
    "autopilot": "PX4 SITL",
    "velocity": 0.0,
    "heading": 127.4,
    "voltage": 16.2,
    "current": 2.1,
    "vertical_speed": 0.0,   # Vertical speed in m/s
    "ground_pressure": 1013.25, # Standard atmospheric pressure at sea level (hPa)
}

status_messages = []
simulation_step = 0

def add_status_message(message, level="INFO"):
    """Add a status message to the log with timestamp"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    status_messages.append({
        "timestamp": timestamp,
        "level": level,
        "message": message
    })
    # Keep only the last 100 messages
    while len(status_messages) > 100:
        status_messages.pop(0)

# ------------- Web Routes -------------

@app.route('/')
def index():
    """Serve the main dashboard HTML file"""
    return send_from_directory(".", "dashboard.html")

# API endpoints for AJAX requests
@app.route('/api/status')
def get_status():
    """Return current drone status as JSON"""
    # Calculate the ASL altitude by adding the relative altitude to the base ASL
    drone_status["altitude_asl"] = 15.0 + drone_status["altitude"]
    
    # Calculate barometric pressure based on altitude (simplified model)
    # Pressure decreases ~11.3 Pa per meter in first 1000m
    altitude_change_m = drone_status["altitude"]
    pressure_change = altitude_change_m * 0.113  # hPa per meter
    drone_status["ground_pressure"] = max(1013.25 - pressure_change, 900)
    
    # Update vertical speed based on altitude changes
    if drone_status["mode"] == "GUIDED" and drone_status["altitude"] < 10.0:
        drone_status["vertical_speed"] = drone_status["velocity"]
    elif drone_status["mode"] == "LAND":
        drone_status["vertical_speed"] = -drone_status["velocity"]
    else:
        # Small random fluctuation when hovering
        drone_status["vertical_speed"] = random.uniform(-0.05, 0.05)
    
    return jsonify(drone_status)

@app.route('/api/logs')
def get_logs():
    """Return recent logs as JSON"""
    return jsonify(status_messages[-50:] if status_messages else [])

@app.route('/api/command', methods=['POST'])
def execute_command():
    """Handle commands from web client"""
    data = request.get_json()
    if not data or 'command' not in data:
        return jsonify({"error": "Invalid command format"}), 400
    
    command = data['command'].strip().lower()
    print(f"Received command: {command}")
    
    add_status_message(f"Web client sent command: {command}", "INFO")
    
    # Special voice command for automatic arm and takeoff
    if command in ["hey ! max let go for a ride", "hey! max let go for a ride", "hey max let go for a ride"]:
        add_status_message("🎤 VOICE COMMAND DETECTED: Auto arm and takeoff sequence", "SYSTEM")
        add_status_message("👨‍✈️ AMAR: Voice authorization accepted", "SUCCESS")
        
        if drone_status["armed"]:
            add_status_message("PROJECT MADMAX already armed - proceeding to takeoff", "WARN")
        else:
            # Automatic arming sequence
            add_status_message("🔐 STEP 1: Automatic arming sequence initiated...", "SYSTEM")
            time.sleep(0.3)
            add_status_message("MAVLink: Sending COMMAND_LONG (CMD_COMPONENT_ARM_DISARM)", "DEBUG")
            time.sleep(0.2)
            add_status_message("Pre-arm checks: GPS ✓ IMU ✓ Battery ✓ Radio ✓", "SUCCESS")
            time.sleep(0.2)
            
            # Set system to armed state
            drone_status["armed"] = True
            drone_status["mode"] = "GUIDED"
            
            add_status_message("🚁 SYSTEM ARMED: Motors now active", "WARN")
            time.sleep(0.3)
        
        # Automatic takeoff sequence
        add_status_message("🚀 STEP 2: Automatic takeoff sequence initiated...", "SYSTEM")
        time.sleep(0.3)
        add_status_message("MAVLink: Sending NAV_TAKEOFF command (ALT: 10m)", "DEBUG")
        time.sleep(0.2)
        add_status_message("Spooling up motors for vertical ascent...", "INFO")
        time.sleep(0.3)
        
        drone_status["mode"] = "GUIDED"
        add_status_message("🎯 TARGET: Ascending to 10m relative altitude", "SUCCESS")
        add_status_message("✈️ PROJECT MADMAX is now airborne! Enjoy your ride!", "SUCCESS")
        add_status_message(f"Current ASL: {drone_status['altitude'] + 15.0:.1f} meters", "INFO")
    
    elif command == "arm":
        if drone_status["armed"]:
            add_status_message("PROJECT MADMAX is already armed!", "WARN")
        else:
            # Enhanced MAVLink-style arming sequence (simplified for demo)
            add_status_message("Initializing arming sequence for PROJECT MADMAX...", "SYSTEM")
            time.sleep(0.2)  # Brief delay for visual feedback
            add_status_message("MAVLink: Sending ARM command", "DEBUG")
            time.sleep(0.2)
            add_status_message("All pre-arm checks complete", "SYSTEM")
            
            # Set system to armed state
            drone_status["armed"] = True
            drone_status["mode"] = "LOITER"
            
            add_status_message("SYSTEM ARMED: Vehicle is now in LOITER mode", "WARN")
            add_status_message("!! CAUTION: MOTORS ARE ACTIVE !!", "WARN")
    
    elif command == "disarm":
        if not drone_status["armed"]:
            add_status_message("PROJECT MADMAX is already disarmed", "INFO")
        else:
            # Disarming sequence
            add_status_message("Initiating disarm sequence...", "WARN")
            time.sleep(0.2)
            add_status_message("MAVLink: Sending DISARM command", "DEBUG")
            time.sleep(0.2)
            
            # Update drone status
            drone_status["armed"] = False
            drone_status["mode"] = "HOLD"
            
            add_status_message("SYSTEM DISARMED: All motors stopped", "SUCCESS")
    
    elif command == "takeoff":
        if not drone_status["armed"]:
            add_status_message("Cannot takeoff - system not armed", "ERROR")
        else:
            add_status_message("Initiating takeoff sequence...", "WARN")
            drone_status["mode"] = "GUIDED"
            # The altitude will increase in the simulation task
            add_status_message("Taking off to 10m relative altitude", "INFO")
            add_status_message("Current ASL: {} meters".format(drone_status["altitude"] + 15.0), "INFO")
    
    elif command == "land":
        add_status_message("Initiating landing sequence...", "WARN")
        drone_status["mode"] = "LAND"
        # The altitude will decrease in the simulation task
        add_status_message("Landing at current location", "INFO")
    
    elif command == "hover":
        if drone_status["armed"]:
            add_status_message("Switching to LOITER mode (hover)", "INFO")
            drone_status["mode"] = "LOITER"
            drone_status["velocity"] = 0.0
        else:
            add_status_message("Cannot hover - system not armed", "ERROR")
    
    elif command == "status":
        status_msg = (f"PROJECT MADMAX Status: {'Armed' if drone_status['armed'] else 'Disarmed'}, "
                     f"Mode: {drone_status['mode']}, "
                     f"Battery: {drone_status['battery']:.1f}%, "
                     f"Altitude: {drone_status['altitude']:.1f}m")
        add_status_message(status_msg, "INFO")
    
    else:
        add_status_message(f"Unknown command: {command}", "ERROR")
    
    return jsonify({"status": "success", "message": f"Command '{command}' executed"})

# ------------- Simulation Task -------------

def simulate_drone():
    """Simulate drone state changes"""
    global simulation_step
    
    # Add initial boot messages
    boot_messages = [
        ("Initializing PROJECT-MADMAX Web Console...", "SYSTEM"),
        ("Boot sequence started.", "SYSTEM"),
        ("Establishing connection to Pixhawk...", "INFO"),
        ("GPS module initializing...", "INFO"),
        ("Calibrating sensors...", "INFO"),
        ("All systems nominal. Listening for commands.", "SUCCESS")
    ]
    
    for msg, level in boot_messages:
        add_status_message(msg, level)
        time.sleep(0.5)
    
    while True:
        simulation_step += 1
        
        # Update drone_status based on current state and mode
        
        # Battery drain - more realistic based on mode
        if drone_status["armed"]:
            if drone_status["mode"] == "GUIDED" and drone_status["altitude"] < 10.0:
                # Taking off uses more power
                battery_drain = 0.04 + (drone_status["altitude"] / 100)  # More drain as altitude increases
            elif drone_status["mode"] == "LAND":
                # Landing uses moderate power
                battery_drain = 0.03
            else:
                # Hovering uses standard power
                battery_drain = 0.02
            
            # Add small random fluctuation
            battery_drain += random.uniform(-0.005, 0.005)
        else:
            # Minimal drain when disarmed
            battery_drain = 0.005 + random.uniform(-0.001, 0.001)
            
        drone_status["battery"] = max(0, drone_status["battery"] - battery_drain)
        
        # Adjust altitude based on mode
        if drone_status["armed"]:
            if drone_status["mode"] == "GUIDED" and drone_status["altitude"] < 10.0:
                # Takeoff - simulate realistic ascent with varying velocity
                takeoff_velocity = 0
                prev_altitude = drone_status["altitude"]
                
                if drone_status["altitude"] < 1.5:
                    # Initial acceleration phase - faster
                    takeoff_velocity = 1.2 + (drone_status["altitude"] * 0.2)
                    altitude_increment = 0.4
                    
                    # Log takeoff initiation
                    if prev_altitude < 0.1 and drone_status["altitude"] + altitude_increment >= 0.1:
                        add_status_message("Initiating vertical ascent", "INFO")
                        add_status_message("Motors at 90% throttle", "DEBUG")
                        
                elif drone_status["altitude"] < 7.0:
                    # Steady climb phase - much faster
                    takeoff_velocity = 2.0
                    altitude_increment = 0.8
                    
                    # Log middle of climb
                    if prev_altitude < 4.0 and drone_status["altitude"] + altitude_increment >= 4.0:
                        add_status_message("Passing 4.0m altitude, rapid ascent continuing", "INFO")
                        add_status_message("Vertical speed: 2.0 m/s", "DEBUG")
                    
                else:
                    # Final deceleration/positioning phase - quicker positioning
                    takeoff_velocity = 1.5 - ((drone_status["altitude"] - 7.0) * 0.3)
                    altitude_increment = 0.5
                    
                    # Log approach to target altitude
                    if prev_altitude < 9.0 and drone_status["altitude"] + altitude_increment >= 9.0:
                        add_status_message("Approaching target altitude, reducing ascent rate", "INFO")
                
                drone_status["altitude"] += altitude_increment
                drone_status["velocity"] = takeoff_velocity
                
                # Log when reaching target altitude
                if prev_altitude < 9.95 and drone_status["altitude"] >= 9.95:
                    add_status_message("Target altitude reached: 10.0m", "SUCCESS")
                    add_status_message("Altitude ASL: {}m".format(round(drone_status["altitude"] + 15.0, 1)), "INFO")
                    add_status_message("Switching to position hold", "INFO")
                    drone_status["mode"] = "LOITER"
                
            elif drone_status["mode"] == "LAND" and drone_status["altitude"] > 0.1:
                # Landing - simulate realistic descent with varying velocity
                prev_altitude = drone_status["altitude"]
                
                if drone_status["altitude"] > 5.0:
                    # Initial descent phase - faster
                    land_velocity = 0.8
                    altitude_decrement = 0.2
                    
                    # Log initial descent
                    if prev_altitude > 9.9 and drone_status["altitude"] - altitude_decrement <= 9.9:
                        add_status_message("Beginning descent from {}m".format(round(prev_altitude, 1)), "INFO")
                        add_status_message("Initial descent rate: 0.8 m/s", "DEBUG")
                    
                elif drone_status["altitude"] > 2.0:
                    # Mid descent phase - moderate speed
                    land_velocity = 0.5
                    altitude_decrement = 0.12
                    
                    # Log mid-descent
                    if prev_altitude > 5.0 and drone_status["altitude"] - altitude_decrement <= 5.0:
                        add_status_message("Passing 5.0m altitude, reducing descent rate", "INFO")
                        add_status_message("Current ASL: {}m".format(round(drone_status["altitude"] + 15.0 - altitude_decrement, 1)), "DEBUG")
                    
                else:
                    # Final approach - slow and careful
                    land_velocity = 0.3
                    altitude_decrement = 0.08
                    
                    # Log final approach
                    if prev_altitude > 2.0 and drone_status["altitude"] - altitude_decrement <= 2.0:
                        add_status_message("Final approach, altitude 2.0m", "WARN")
                        add_status_message("Preparing for touchdown", "INFO")
                    
                    # Log near touchdown
                    if prev_altitude > 0.5 and drone_status["altitude"] - altitude_decrement <= 0.5:
                        add_status_message("Touchdown imminent", "WARN")
                
                drone_status["altitude"] = max(0, drone_status["altitude"] - altitude_decrement)
                drone_status["velocity"] = -land_velocity
                
                # Auto-disarm when landed
                if drone_status["altitude"] <= 0.1:
                    drone_status["altitude"] = 0.0
                    drone_status["velocity"] = 0.0
                    
                    # Sequence the landing/disarming for more realism
                    add_status_message("Touchdown detected", "INFO")
                    add_status_message("Weight on wheels confirmed", "DEBUG")
                    add_status_message("Reducing motor power to idle", "DEBUG")
                    
                    # Wait a moment before disarming by continuing for one more cycle
                    if drone_status["armed"]:
                        add_status_message("Stabilizing position", "INFO")
                        # A small delay before actual disarm will be simulated by waiting for next cycle
                    else:
                        drone_status["armed"] = False
                        drone_status["mode"] = "HOLD"
                        add_status_message("Landing complete - motors disarmed", "SUCCESS")
                        add_status_message("Mission statistics available", "INFO")
        
        # Random fluctuations in telemetry data with more realistic changes during flight
        if drone_status["armed"]:
            # When flying, the drone uses more resources and GPS signal might fluctuate more
            drone_status["gps_satellites"] = min(12, max(6, drone_status["gps_satellites"] + random.randint(-1, 1)))
            drone_status["signal_strength"] = min(100, max(45, drone_status["signal_strength"] + random.randint(-3, 2)))
            
            # More dramatic resource usage during fast ascent (GUIDED mode)
            if drone_status["mode"] == "GUIDED" and drone_status["altitude"] < 10.0:
                # Higher CPU usage during rapid ascent
                drone_status["cpu_temp"] = min(95, max(65, drone_status["cpu_temp"] + random.randint(0, 3)))  
                # Higher memory usage for intensive flight calculations
                drone_status["memory_usage"] = min(100, max(50, drone_status["memory_usage"] + random.randint(0, 4)))
                # More current draw during powerful takeoff
                drone_status["current"] = 5.0 + random.uniform(-0.5, 0.5)
            else:
                drone_status["cpu_temp"] = min(90, max(50, drone_status["cpu_temp"] + random.randint(-1, 2)))
                drone_status["memory_usage"] = min(100, max(35, drone_status["memory_usage"] + random.randint(-1, 2)))
            
            # Heading changes more during flight
            if drone_status["mode"] == "GUIDED" or drone_status["mode"] == "LAND":
                # During takeoff/landing, small heading deviations that drift slowly
                drone_status["heading"] = (drone_status["heading"] + random.uniform(-0.5, 0.5)) % 360
            else:
                # More stable heading when in position hold
                drone_status["heading"] = (drone_status["heading"] + random.uniform(-0.2, 0.2)) % 360
        else:
            # When disarmed, systems are more stable
            drone_status["gps_satellites"] = min(12, max(8, drone_status["gps_satellites"] + random.randint(-1, 1)))
            drone_status["signal_strength"] = min(100, max(60, drone_status["signal_strength"] + random.randint(-1, 1)))
            drone_status["cpu_temp"] = min(75, max(30, drone_status["cpu_temp"] + random.randint(-1, 1)))
            drone_status["memory_usage"] = min(100, max(20, drone_status["memory_usage"] + random.randint(-1, 1)))
            drone_status["heading"] = (drone_status["heading"] + random.uniform(-0.1, 0.1)) % 360
        
        if drone_status["armed"]:
            # More battery drain when armed
            drone_status["battery"] = max(0, drone_status["battery"] - 0.02)
            drone_status["current"] = 2.0 + random.uniform(-0.2, 0.2)
        else:
            drone_status["current"] = 0.2 + random.uniform(-0.05, 0.05)
        
        drone_status["voltage"] = 15.0 + (drone_status["battery"] / 100) * 1.8
        
        # Small position changes
        drone_status["position"]["lat"] += 0.000001 * random.random()
        drone_status["position"]["lon"] += 0.000001 * random.random()
        
        # Occasional system messages with lower frequency
        if random.random() < 0.05:
            messages = [
                ("GPS signal strength stable", "INFO"),
                ("CPU temperature normal", "DEBUG"),
                ("Memory usage optimal", "DEBUG"),
                ("LoRa packet received", "INFO"),
                ("Battery monitoring active", "INFO"),
                ("Navigation system online", "SUCCESS")
            ]
            msg, level = random.choice(messages)
            add_status_message(msg, level)
            
        time.sleep(1)  # Update every second

if __name__ == "__main__":
    # Start simulation in a separate thread
    simulation_thread = threading.Thread(target=simulate_drone)
    simulation_thread.daemon = True  # This ensures the thread will close when the main program closes
    simulation_thread.start()
    
    # Start web server
    print("======== Web server starting at https://127.0.0.1:5001 ========")
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
