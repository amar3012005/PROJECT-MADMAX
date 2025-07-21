#!/usr/bin/env python3
"""
MADMAX AI Agentic Aerial Robotics - MAVLink Interface
Handles communication with Pixhawk 6C flight controller via MAVLink protocol.
"""

import asyncio
import time
import threading
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum
import json
from loguru import logger

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink
except ImportError:
    logger.error("pymavlink not installed. Install with: pip install pymavlink")
    mavutil = None
    mavlink = None

from flight_agent import MAVLinkCommand, MissionPlan, MissionStatus

class ConnectionStatus(Enum):
    """Connection status with flight controller"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"

class VehicleMode(Enum):
    """Vehicle flight modes"""
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    BRAKE = 17
    THROW = 18

@dataclass
class VehicleState:
    """Current vehicle state from telemetry"""
    armed: bool = False
    mode: str = "UNKNOWN"
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: float = 0.0
    gps_fix_type: int = 0
    gps_satellites: int = 0
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_relative: float = 0.0
    altitude_absolute: float = 0.0
    heading: float = 0.0
    groundspeed: float = 0.0
    airspeed: float = 0.0
    vertical_speed: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    system_status: str = "UNKNOWN"
    last_heartbeat: float = 0.0
    home_latitude: float = 0.0
    home_longitude: float = 0.0
    home_altitude: float = 0.0

class MAVLinkInterface:
    """Interface for communicating with Pixhawk 6C via MAVLink"""
    
    def __init__(self, connection_string: str = "/dev/ttyUSB0", baud_rate: int = 57600):
        """
        Initialize MAVLink interface
        
        Args:
            connection_string: Serial port or network connection string
            baud_rate: Baud rate for serial connection
        """
        self.logger = logger.bind(module="MAVLinkInterface")
        
        if not mavutil:
            raise ImportError("pymavlink is required but not installed")
        
        self.connection_string = connection_string
        self.baud_rate = baud_rate
        self.connection = None
        self.status = ConnectionStatus.DISCONNECTED
        
        # Vehicle state
        self.vehicle_state = VehicleState()
        self.state_callbacks: List[Callable[[VehicleState], None]] = []
        
        # Message handling
        self.message_handlers = {}
        self.setup_message_handlers()
        
        # Threading
        self.telemetry_thread = None
        self.running = False
        
        # Command tracking
        self.command_ack_callbacks = {}
        self.mission_execution = {}
        
        self.logger.info(f"MAVLink Interface initialized for {connection_string}")
    
    def setup_message_handlers(self):
        """Setup handlers for different MAVLink message types"""
        self.message_handlers = {
            'HEARTBEAT': self._handle_heartbeat,
            'SYS_STATUS': self._handle_sys_status,
            'BATTERY_STATUS': self._handle_battery_status,
            'GPS_RAW_INT': self._handle_gps_raw,
            'GLOBAL_POSITION_INT': self._handle_global_position,
            'ATTITUDE': self._handle_attitude,
            'VFR_HUD': self._handle_vfr_hud,
            'COMMAND_ACK': self._handle_command_ack,
            'MISSION_ACK': self._handle_mission_ack,
            'HOME_POSITION': self._handle_home_position,
        }
    
    async def connect(self) -> bool:
        """
        Connect to the flight controller
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.status = ConnectionStatus.CONNECTING
            self.logger.info(f"Connecting to flight controller: {self.connection_string}")
            
            # Create MAVLink connection
            self.connection = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baud_rate,
                autoreconnect=True
            )
            
            # Wait for heartbeat
            self.logger.info("Waiting for heartbeat...")
            heartbeat = self.connection.wait_heartbeat(timeout=10)
            
            if heartbeat:
                self.status = ConnectionStatus.CONNECTED
                self.logger.info(f"Connected to system {heartbeat.get_srcSystem()}")
                
                # Start telemetry thread
                self.running = True
                self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
                self.telemetry_thread.start()
                
                # Request data streams
                await self._request_data_streams()
                
                return True
            else:
                self.status = ConnectionStatus.ERROR
                self.logger.error("No heartbeat received")
                return False
                
        except Exception as e:
            self.status = ConnectionStatus.ERROR
            self.logger.error(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from flight controller"""
        self.running = False
        self.status = ConnectionStatus.DISCONNECTED
        
        if self.telemetry_thread and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=2)
        
        if self.connection:
            self.connection.close()
            self.connection = None
        
        self.logger.info("Disconnected from flight controller")
    
    async def _request_data_streams(self):
        """Request telemetry data streams from flight controller"""
        if not self.connection:
            return
        
        # Request various data streams
        streams = [
            (mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
            (mavlink.MAV_DATA_STREAM_POSITION, 3),
            (mavlink.MAV_DATA_STREAM_EXTRA1, 4),
            (mavlink.MAV_DATA_STREAM_EXTRA2, 4),
            (mavlink.MAV_DATA_STREAM_EXTRA3, 2),
        ]
        
        for stream_id, rate in streams:
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                stream_id,
                rate,
                1  # start_stop (1=start, 0=stop)
            )
    
    def _telemetry_loop(self):
        """Main telemetry processing loop"""
        self.logger.info("Telemetry loop started")
        
        while self.running and self.connection:
            try:
                # Receive message with timeout
                msg = self.connection.recv_match(timeout=1.0)
                
                if msg:
                    # Handle message
                    msg_type = msg.get_type()
                    if msg_type in self.message_handlers:
                        self.message_handlers[msg_type](msg)
                    
                    # Update last heartbeat time
                    if msg_type == 'HEARTBEAT':
                        self.vehicle_state.last_heartbeat = time.time()
                
                # Check connection health
                if time.time() - self.vehicle_state.last_heartbeat > 5.0:
                    self.logger.warning("Heartbeat timeout - connection may be lost")
                    
            except Exception as e:
                self.logger.error(f"Telemetry loop error: {e}")
                time.sleep(0.1)
        
        self.logger.info("Telemetry loop stopped")
    
    def _handle_heartbeat(self, msg):
        """Handle HEARTBEAT message"""
        self.vehicle_state.armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.vehicle_state.system_status = mavlink.enums['MAV_STATE'][msg.system_status].name
        
        # Decode flight mode
        if msg.custom_mode in [mode.value for mode in VehicleMode]:
            for mode in VehicleMode:
                if mode.value == msg.custom_mode:
                    self.vehicle_state.mode = mode.name
                    break
        
        self._notify_state_callbacks()
    
    def _handle_sys_status(self, msg):
        """Handle SYS_STATUS message"""
        self.vehicle_state.battery_voltage = msg.voltage_battery / 1000.0  # mV to V
        self.vehicle_state.battery_current = msg.current_battery / 100.0  # cA to A
        self.vehicle_state.battery_remaining = msg.battery_remaining
        self._notify_state_callbacks()
    
    def _handle_battery_status(self, msg):
        """Handle BATTERY_STATUS message"""
        if len(msg.voltages) > 0:
            self.vehicle_state.battery_voltage = msg.voltages[0] / 1000.0
        self.vehicle_state.battery_current = msg.current_battery / 100.0
        self.vehicle_state.battery_remaining = msg.battery_remaining
        self._notify_state_callbacks()
    
    def _handle_gps_raw(self, msg):
        """Handle GPS_RAW_INT message"""
        self.vehicle_state.gps_fix_type = msg.fix_type
        self.vehicle_state.gps_satellites = msg.satellites_visible
        self._notify_state_callbacks()
    
    def _handle_global_position(self, msg):
        """Handle GLOBAL_POSITION_INT message"""
        self.vehicle_state.latitude = msg.lat / 1e7
        self.vehicle_state.longitude = msg.lon / 1e7
        self.vehicle_state.altitude_absolute = msg.alt / 1000.0
        self.vehicle_state.altitude_relative = msg.relative_alt / 1000.0
        self.vehicle_state.heading = msg.hdg / 100.0
        self.vehicle_state.vertical_speed = msg.vz / 100.0
        self._notify_state_callbacks()
    
    def _handle_attitude(self, msg):
        """Handle ATTITUDE message"""
        import math
        self.vehicle_state.roll = math.degrees(msg.roll)
        self.vehicle_state.pitch = math.degrees(msg.pitch)
        self.vehicle_state.yaw = math.degrees(msg.yaw)
        self._notify_state_callbacks()
    
    def _handle_vfr_hud(self, msg):
        """Handle VFR_HUD message"""
        self.vehicle_state.airspeed = msg.airspeed
        self.vehicle_state.groundspeed = msg.groundspeed
        self._notify_state_callbacks()
    
    def _handle_command_ack(self, msg):
        """Handle COMMAND_ACK message"""
        command_id = msg.command
        result = msg.result
        
        self.logger.info(f"Command ACK: {command_id}, Result: {result}")
        
        # Notify callbacks
        if command_id in self.command_ack_callbacks:
            callback = self.command_ack_callbacks.pop(command_id)
            callback(result == mavlink.MAV_RESULT_ACCEPTED)
    
    def _handle_mission_ack(self, msg):
        """Handle MISSION_ACK message"""
        mission_type = msg.type
        result = msg.type
        self.logger.info(f"Mission ACK: Type={mission_type}, Result={result}")
    
    def _handle_home_position(self, msg):
        """Handle HOME_POSITION message"""
        self.vehicle_state.home_latitude = msg.latitude / 1e7
        self.vehicle_state.home_longitude = msg.longitude / 1e7
        self.vehicle_state.home_altitude = msg.altitude / 1000.0
        self._notify_state_callbacks()
    
    def _notify_state_callbacks(self):
        """Notify all registered state callbacks"""
        for callback in self.state_callbacks:
            try:
                callback(self.vehicle_state)
            except Exception as e:
                self.logger.error(f"State callback error: {e}")
    
    def add_state_callback(self, callback: Callable[[VehicleState], None]):
        """Add a callback for vehicle state updates"""
        self.state_callbacks.append(callback)
    
    def remove_state_callback(self, callback: Callable[[VehicleState], None]):
        """Remove a state callback"""
        if callback in self.state_callbacks:
            self.state_callbacks.remove(callback)
    
    async def send_command(self, command: MAVLinkCommand, wait_ack: bool = True) -> bool:
        """
        Send a MAVLink command to the flight controller
        
        Args:
            command: MAVLink command to send
            wait_ack: Whether to wait for acknowledgment
            
        Returns:
            True if command sent successfully (and acknowledged if wait_ack=True)
        """
        if not self.connection or self.status != ConnectionStatus.CONNECTED:
            self.logger.error("Not connected to flight controller")
            return False
        
        try:
            self.logger.info(f"Sending command: {command.command_id}")
            
            # Send command
            self.connection.mav.command_long_send(
                command.target_system,
                command.target_component,
                command.command_id,
                0,  # confirmation
                command.param1,
                command.param2,
                command.param3,
                command.param4,
                command.param5,
                command.param6,
                command.param7
            )
            
            if wait_ack:
                # Wait for acknowledgment
                ack_received = asyncio.Event()
                ack_result = [False]
                
                def ack_callback(success: bool):
                    ack_result[0] = success
                    ack_received.set()
                
                self.command_ack_callbacks[command.command_id] = ack_callback
                
                # Wait for ACK with timeout
                try:
                    await asyncio.wait_for(ack_received.wait(), timeout=5.0)
                    return ack_result[0]
                except asyncio.TimeoutError:
                    self.logger.error(f"Command ACK timeout for command {command.command_id}")
                    if command.command_id in self.command_ack_callbacks:
                        del self.command_ack_callbacks[command.command_id]
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send command: {e}")
            return False
    
    async def execute_mission(self, mission_plan: MissionPlan) -> bool:
        """
        Execute a complete mission plan
        
        Args:
            mission_plan: Mission plan to execute
            
        Returns:
            True if mission executed successfully
        """
        if not self.connection or self.status != ConnectionStatus.CONNECTED:
            self.logger.error("Not connected to flight controller")
            return False
        
        mission_id = mission_plan.mission_id
        self.logger.info(f"Executing mission: {mission_id}")
        
        # Update mission status
        mission_plan.status = MissionStatus.EXECUTING
        self.mission_execution[mission_id] = mission_plan
        
        try:
            # Execute commands sequentially
            for i, command in enumerate(mission_plan.mavlink_commands):
                self.logger.info(f"Executing command {i+1}/{len(mission_plan.mavlink_commands)}")
                
                success = await self.send_command(command, wait_ack=True)
                if not success:
                    self.logger.error(f"Command {i+1} failed")
                    mission_plan.status = MissionStatus.FAILED
                    return False
                
                # Add delay between commands if needed
                await asyncio.sleep(0.5)
            
            mission_plan.status = MissionStatus.COMPLETED
            self.logger.info(f"Mission {mission_id} completed successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Mission execution failed: {e}")
            mission_plan.status = MissionStatus.FAILED
            return False
        
        finally:
            if mission_id in self.mission_execution:
                del self.mission_execution[mission_id]
    
    def get_vehicle_state(self) -> VehicleState:
        """Get current vehicle state"""
        return self.vehicle_state
    
    def is_connected(self) -> bool:
        """Check if connected to flight controller"""
        return self.status == ConnectionStatus.CONNECTED
    
    def get_connection_status(self) -> ConnectionStatus:
        """Get current connection status"""
        return self.status
    
    async def emergency_stop(self) -> bool:
        """Send emergency stop command"""
        emergency_cmd = MAVLinkCommand(
            command_id=mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            param1=1.0  # Terminate flight
        )
        
        self.logger.warning("EMERGENCY STOP ACTIVATED")
        return await self.send_command(emergency_cmd, wait_ack=True)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert current state to dictionary for JSON serialization"""
        return {
            'connection_status': self.status.value,
            'connection_string': self.connection_string,
            'vehicle_state': {
                'armed': self.vehicle_state.armed,
                'mode': self.vehicle_state.mode,
                'battery_voltage': self.vehicle_state.battery_voltage,
                'battery_current': self.vehicle_state.battery_current,
                'battery_remaining': self.vehicle_state.battery_remaining,
                'gps_fix_type': self.vehicle_state.gps_fix_type,
                'gps_satellites': self.vehicle_state.gps_satellites,
                'latitude': self.vehicle_state.latitude,
                'longitude': self.vehicle_state.longitude,
                'altitude_relative': self.vehicle_state.altitude_relative,
                'altitude_absolute': self.vehicle_state.altitude_absolute,
                'heading': self.vehicle_state.heading,
                'groundspeed': self.vehicle_state.groundspeed,
                'airspeed': self.vehicle_state.airspeed,
                'vertical_speed': self.vehicle_state.vertical_speed,
                'roll': self.vehicle_state.roll,
                'pitch': self.vehicle_state.pitch,
                'yaw': self.vehicle_state.yaw,
                'system_status': self.vehicle_state.system_status,
                'last_heartbeat': self.vehicle_state.last_heartbeat,
                'home_latitude': self.vehicle_state.home_latitude,
                'home_longitude': self.vehicle_state.home_longitude,
                'home_altitude': self.vehicle_state.home_altitude,
            }
        }

# Example usage
if __name__ == "__main__":
    import asyncio
    
    async def test_mavlink_interface():
        # Initialize interface (use SITL for testing)
        interface = MAVLinkInterface("tcp:127.0.0.1:5760")
        
        # State callback
        def on_state_update(state: VehicleState):
            print(f"Armed: {state.armed}, Mode: {state.mode}, Battery: {state.battery_voltage:.1f}V")
        
        interface.add_state_callback(on_state_update)
        
        # Connect
        if await interface.connect():
            print("Connected successfully!")
            
            # Wait for some telemetry
            await asyncio.sleep(5)
            
            # Test command
            arm_command = MAVLinkCommand(
                command_id=mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                param1=1.0  # Arm
            )
            
            success = await interface.send_command(arm_command)
            print(f"Arm command result: {success}")
            
            # Wait a bit more
            await asyncio.sleep(2)
            
            # Disconnect
            interface.disconnect()
        else:
            print("Connection failed!")
    
    # Run test
    asyncio.run(test_mavlink_interface())
