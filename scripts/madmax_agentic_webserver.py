#!/usr/bin/env python3
"""
MADMAX AI Agentic Aerial Robotics - Web Server
Flask web server that provides a web interface for the AI agentic drone control system.
"""

import asyncio
import json
import os
import threading
import time
from datetime import datetime
from typing import Dict, Any, List

from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit
from loguru import logger

# Import AI agentic components
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'ai_agents'))

from ai_agents.agentic_controller import AgenticController, SystemStatus

app = Flask(__name__, static_folder="static", template_folder=".")
app.config['SECRET_KEY'] = 'madmax_agentic_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global agentic controller
agentic_controller: AgenticController = None
system_ready = False

# Web interface state
connected_clients = set()
status_messages = []

def add_status_message(message: str, level: str = "INFO"):
    """Add a status message to the web interface log"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    status_messages.append({
        "timestamp": timestamp,
        "level": level,
        "message": message
    })
    
    # Keep only the last 100 messages
    while len(status_messages) > 100:
        status_messages.pop(0)
    
    # Emit to connected clients
    socketio.emit('status_update', {
        'timestamp': timestamp,
        'level': level,
        'message': message
    })

def on_system_state_update(system_state):
    """Callback for system state updates"""
    socketio.emit('system_state_update', {
        'status': system_state.status.value,
        'nlp_ready': system_state.nlp_ready,
        'agent_ready': system_state.agent_ready,
        'mavlink_connected': system_state.mavlink_connected,
        'last_command': system_state.last_command,
        'last_response': system_state.last_response,
        'error_message': system_state.error_message,
        'uptime': system_state.uptime,
        'vehicle_state': agentic_controller.mavlink_interface.to_dict() if agentic_controller.mavlink_interface else None
    })

def on_command_processed(command: str, response: Dict[str, Any]):
    """Callback for processed commands"""
    socketio.emit('command_processed', {
        'command': command,
        'response': response,
        'timestamp': datetime.now().isoformat()
    })
    
    # Add to status messages
    if response['success']:
        add_status_message(f"✓ Command executed: {command}", "SUCCESS")
    else:
        add_status_message(f"✗ Command failed: {command} - {response.get('error', 'Unknown error')}", "ERROR")

async def initialize_agentic_system():
    """Initialize the AI agentic system"""
    global agentic_controller, system_ready
    
    try:
        add_status_message("Initializing MADMAX AI Agentic System...", "INFO")
        
        # Get configuration from environment or use defaults
        mavlink_connection = os.getenv("MAVLINK_CONNECTION", "/dev/ttyUSB0")
        openai_api_key = os.getenv("OPENAI_API_KEY")
        
        # Initialize controller
        agentic_controller = AgenticController(
            mavlink_connection=mavlink_connection,
            openai_api_key=openai_api_key
        )
        
        # Add callbacks
        agentic_controller.add_state_callback(on_system_state_update)
        agentic_controller.add_command_callback(on_command_processed)
        
        # Initialize system
        success = await agentic_controller.initialize()
        
        if success:
            system_ready = True
            add_status_message("🚁 MADMAX AI Agentic System READY!", "SUCCESS")
        else:
            add_status_message("❌ System initialization failed", "ERROR")
            
    except Exception as e:
        logger.error(f"Failed to initialize agentic system: {e}")
        add_status_message(f"❌ Initialization error: {str(e)}", "ERROR")

# ------------- Web Routes -------------

@app.route('/')
def index():
    """Serve the main dashboard HTML file"""
    return render_template('agentic_dashboard.html')

@app.route('/static/<path:path>')
def static_files(path):
    """Serve static files"""
    return send_from_directory('static', path)

@app.route('/api/system_status')
def get_system_status():
    """Get current system status"""
    if agentic_controller:
        return jsonify(agentic_controller.to_dict())
    else:
        return jsonify({
            'system_state': {
                'status': 'offline',
                'nlp_ready': False,
                'agent_ready': False,
                'mavlink_connected': False,
                'error_message': 'System not initialized'
            }
        })

@app.route('/api/command_history')
def get_command_history():
    """Get command history"""
    if agentic_controller:
        return jsonify(agentic_controller.get_command_history())
    else:
        return jsonify([])

@app.route('/api/mission_history')
def get_mission_history():
    """Get mission history"""
    if agentic_controller:
        return jsonify(agentic_controller.get_mission_history())
    else:
        return jsonify([])

@app.route('/api/logs')
def get_logs():
    """Get recent status messages"""
    return jsonify(status_messages)

@app.route('/api/process_command', methods=['POST'])
def process_command():
    """Process a natural language command"""
    if not system_ready or not agentic_controller:
        return jsonify({
            'success': False,
            'error': 'System not ready',
            'response': 'AI agentic system is not ready. Please wait for initialization to complete.'
        })
    
    data = request.get_json()
    command = data.get('command', '').strip()
    
    if not command:
        return jsonify({
            'success': False,
            'error': 'Empty command',
            'response': 'Please enter a command.'
        })
    
    # Process command asynchronously
    async def process_async():
        return await agentic_controller.process_natural_language_command(command)
    
    # Run in event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        result = loop.run_until_complete(process_async())
        return jsonify(result)
    finally:
        loop.close()

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """Execute emergency stop"""
    if not system_ready or not agentic_controller:
        return jsonify({
            'success': False,
            'error': 'System not ready'
        })
    
    # Execute emergency stop asynchronously
    async def emergency_async():
        return await agentic_controller.emergency_stop()
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        success = loop.run_until_complete(emergency_async())
        return jsonify({'success': success})
    finally:
        loop.close()

@app.route('/api/reconnect_mavlink', methods=['POST'])
def reconnect_mavlink():
    """Attempt to reconnect MAVLink"""
    if not agentic_controller:
        return jsonify({
            'success': False,
            'error': 'System not initialized'
        })
    
    async def reconnect_async():
        return await agentic_controller.reconnect_mavlink()
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        success = loop.run_until_complete(reconnect_async())
        return jsonify({'success': success})
    finally:
        loop.close()

# ------------- Socket.IO Events -------------

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    connected_clients.add(request.sid)
    add_status_message(f"Web client connected: {request.sid}", "INFO")
    
    # Send current system status to newly connected client
    if agentic_controller:
        emit('system_state_update', agentic_controller.to_dict())

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    connected_clients.discard(request.sid)
    add_status_message(f"Web client disconnected: {request.sid}", "INFO")

@socketio.on('send_command')
def handle_send_command(data):
    """Handle command from web client via Socket.IO"""
    command = data.get('command', '').strip()
    
    if not system_ready or not agentic_controller:
        emit('command_result', {
            'success': False,
            'error': 'System not ready',
            'response': 'AI agentic system is not ready.'
        })
        return
    
    if not command:
        emit('command_result', {
            'success': False,
            'error': 'Empty command',
            'response': 'Please enter a command.'
        })
        return
    
    # Process command in background thread
    def process_command_thread():
        async def process_async():
            result = await agentic_controller.process_natural_language_command(command)
            socketio.emit('command_result', result, room=request.sid)
        
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(process_async())
        finally:
            loop.close()
    
    threading.Thread(target=process_command_thread, daemon=True).start()

def run_initialization():
    """Run initialization in background thread"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(initialize_agentic_system())
    finally:
        loop.close()

if __name__ == "__main__":
    # Add initial status message
    add_status_message("MADMAX AI Agentic Web Server starting...", "INFO")
    
    # Start initialization in background thread
    init_thread = threading.Thread(target=run_initialization, daemon=True)
    init_thread.start()
    
    # Start web server
    print("=" * 60)
    print("🚁 MADMAX AI AGENTIC AERIAL ROBOTICS SYSTEM")
    print("=" * 60)
    print(f" * Web Interface: http://127.0.0.1:5000")
    print(f" * MAVLink Connection: {os.getenv('MAVLINK_CONNECTION', '/dev/ttyUSB0')}")
    print(f" * OpenAI API: {'Enabled' if os.getenv('OPENAI_API_KEY') else 'Disabled (using rule-based system)'}")
    print("=" * 60)
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
