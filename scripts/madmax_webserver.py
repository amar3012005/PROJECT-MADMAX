#!/usr/bin/env python3

from flask import Flask, render_template, send_from_directory, jsonify, request
from flask_socketio import SocketIO
import json
import os
import random
import threading
import time
from datetime import datetime
import asyncio

app = Flask(__name__, static_folder="static", template_folder=".")
socketio = SocketIO(app)

# Simulation data (this would eventually be connected to your actual drone data)
drone_status = {
    "armed": False,
    "mode": "HOLD",
    "battery": 95.4,
    "altitude": 0.0,
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
    return render_template('dashboard.html')

@app.route('/static/<path:path>')
def static_files(path):
    """Serve static files (CSS, JS, images)"""
    return send_from_directory('static', path)

# ------------- Socket.IO Events -------------

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f"Client connected")
    add_status_message("Web client connected", "SYSTEM")
    # Send initial data
    socketio.emit('status_update', drone_status)
    socketio.emit('log_update', status_messages[-50:] if status_messages else [])

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f"Client disconnected")
    add_status_message("Web client disconnected", "SYSTEM")

@socketio.on('command')
def handle_command(data):
    """Handle commands from web client"""
    command = data.get('command', '').strip().lower()
    print(f"Received command: {command}")
    
    add_status_message(f"Web client sent command: {command}", "INFO")
    
    if command == "arm":
        if drone_status["armed"]:
            add_status_message("PROJECT MADMAX is already armed!", "WARN")
        else:
            # Enhanced MAVLink-style arming sequence (simplified for demo)
            add_status_message("Initializing arming sequence for PROJECT MADMAX...", "SYSTEM")
            time.sleep(0.5)
            add_status_message("MAVLink: Sending ARM command", "DEBUG")
            time.sleep(0.5)
            add_status_message("All pre-arm checks complete", "SYSTEM")
            time.sleep(0.5)
            
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
            time.sleep(0.5)
            add_status_message("MAVLink: Sending DISARM command", "DEBUG")
            time.sleep(0.5)
            
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
            add_status_message("Taking off to 5m altitude", "INFO")
    
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
    
    # Send updated status and logs to all clients
    socketio.emit('status_update', drone_status)
    socketio.emit('log_update', status_messages[-50:])

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
        
        # Battery drain
        drone_status["battery"] = max(0, drone_status["battery"] - 0.01)
        
        # Adjust altitude based on mode
        if drone_status["armed"]:
            if drone_status["mode"] == "GUIDED" and drone_status["altitude"] < 5.0:
                # Takeoff
                drone_status["altitude"] += 0.1
                drone_status["velocity"] = 0.5
                
            elif drone_status["mode"] == "LAND" and drone_status["altitude"] > 0.1:
                # Landing
                drone_status["altitude"] = max(0, drone_status["altitude"] - 0.15)
                drone_status["velocity"] = -0.5
                
                # Auto-disarm when landed
                if drone_status["altitude"] <= 0.1:
                    drone_status["altitude"] = 0.0
                    drone_status["velocity"] = 0.0
                    drone_status["armed"] = False
                    drone_status["mode"] = "HOLD"
                    add_status_message("Landing complete - motors disarmed", "SUCCESS")
        
        # Random fluctuations in telemetry data
        drone_status["gps_satellites"] = min(12, max(8, drone_status["gps_satellites"] + random.randint(-1, 1)))
        drone_status["signal_strength"] = min(100, max(50, drone_status["signal_strength"] + random.randint(-2, 2)))
        drone_status["cpu_temp"] = min(90, max(30, drone_status["cpu_temp"] + random.randint(-1, 1)))
        drone_status["memory_usage"] = min(100, max(20, drone_status["memory_usage"] + random.randint(-1, 1)))
        drone_status["heading"] = (drone_status["heading"] + random.uniform(-1, 1)) % 360
        
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
        
        # Send updated status to all clients
        socketio.emit('status_update', drone_status)
        socketio.emit('log_update', status_messages[-50:])
        
        time.sleep(1)  # Update every second

if __name__ == "__main__":
    # Start simulation in a separate thread
    simulation_thread = threading.Thread(target=simulate_drone, daemon=True)
    simulation_thread.start()
    
    # Start the Flask server
    print("======== Web server started at http://localhost:8080 ========")
    socketio.run(app, host='localhost', port=8080, debug=True)
