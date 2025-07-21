#!/usr/bin/env python3
"""
MADMAX AI Agentic Aerial Robotics - Main Agentic Controller
Orchestrates NLP processing, AI agents, and MAVLink communication for autonomous drone control.
"""

import asyncio
import json
import time
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from enum import Enum
import os
from loguru import logger

from nlp_intent_extractor import NLPIntentExtractor, IntentResult, FlightIntent
from flight_agent import FlightAgent, MissionPlan, MissionStatus
from mavlink_interface import MAVLinkInterface, VehicleState, ConnectionStatus

class SystemStatus(Enum):
    """Overall system status"""
    INITIALIZING = "initializing"
    READY = "ready"
    PROCESSING = "processing"
    EXECUTING = "executing"
    ERROR = "error"
    OFFLINE = "offline"

@dataclass
class SystemState:
    """Complete system state"""
    status: SystemStatus = SystemStatus.INITIALIZING
    nlp_ready: bool = False
    agent_ready: bool = False
    mavlink_connected: bool = False
    vehicle_state: Optional[VehicleState] = None
    current_mission: Optional[MissionPlan] = None
    last_command: str = ""
    last_response: str = ""
    error_message: str = ""
    uptime: float = 0.0

class AgenticController:
    """Main controller for AI agentic aerial robotics system"""
    
    def __init__(self, 
                 mavlink_connection: str = "/dev/ttyUSB0",
                 openai_api_key: Optional[str] = None):
        """
        Initialize the agentic controller
        
        Args:
            mavlink_connection: MAVLink connection string for Pixhawk 6C
            openai_api_key: OpenAI API key for advanced AI features
        """
        self.logger = logger.bind(module="AgenticController")
        
        # System state
        self.system_state = SystemState()
        self.start_time = time.time()
        
        # Initialize components
        self.nlp_extractor = None
        self.flight_agent = None
        self.mavlink_interface = None
        
        # Configuration
        self.mavlink_connection = mavlink_connection
        self.openai_api_key = openai_api_key
        
        # Command history and mission tracking
        self.command_history: List[Dict[str, Any]] = []
        self.mission_history: List[MissionPlan] = []
        self.active_missions: Dict[str, MissionPlan] = {}
        
        # Callbacks for external systems (web interface, etc.)
        self.state_callbacks: List[callable] = []
        self.command_callbacks: List[callable] = []
        
        self.logger.info("Agentic Controller initialized")
    
    async def initialize(self) -> bool:
        """
        Initialize all system components
        
        Returns:
            True if initialization successful
        """
        self.logger.info("Initializing MADMAX AI Agentic System...")
        self.system_state.status = SystemStatus.INITIALIZING
        
        try:
            # Initialize NLP Intent Extractor
            self.logger.info("Initializing NLP Intent Extractor...")
            self.nlp_extractor = NLPIntentExtractor()
            self.system_state.nlp_ready = True
            self.logger.info("✓ NLP Intent Extractor ready")
            
            # Initialize Flight Agent
            self.logger.info("Initializing Flight Agent...")
            self.flight_agent = FlightAgent(api_key=self.openai_api_key)
            self.system_state.agent_ready = True
            self.logger.info("✓ Flight Agent ready")
            
            # Initialize MAVLink Interface
            self.logger.info(f"Initializing MAVLink Interface ({self.mavlink_connection})...")
            self.mavlink_interface = MAVLinkInterface(self.mavlink_connection)
            
            # Add vehicle state callback
            self.mavlink_interface.add_state_callback(self._on_vehicle_state_update)
            
            # Attempt connection to flight controller
            connected = await self.mavlink_interface.connect()
            if connected:
                self.system_state.mavlink_connected = True
                self.logger.info("✓ MAVLink Interface connected")
            else:
                self.logger.warning("⚠ MAVLink Interface failed to connect (will retry)")
            
            # System ready
            self.system_state.status = SystemStatus.READY
            self.logger.info("🚁 MADMAX AI Agentic System READY!")
            
            # Notify callbacks
            self._notify_state_callbacks()
            
            return True
            
        except Exception as e:
            self.system_state.status = SystemStatus.ERROR
            self.system_state.error_message = str(e)
            self.logger.error(f"Initialization failed: {e}")
            return False
    
    async def process_natural_language_command(self, command: str) -> Dict[str, Any]:
        """
        Process a natural language command through the full AI pipeline
        
        Args:
            command: Natural language command from user
            
        Returns:
            Dictionary containing processing results and system response
        """
        if self.system_state.status != SystemStatus.READY:
            return {
                'success': False,
                'error': 'System not ready',
                'response': 'System is not ready to process commands. Please check system status.'
            }
        
        self.logger.info(f"Processing command: '{command}'")
        self.system_state.status = SystemStatus.PROCESSING
        self.system_state.last_command = command
        
        try:
            # Step 1: Extract intent from natural language
            self.logger.info("Step 1: Extracting intent...")
            intent_result = self.nlp_extractor.extract_intent(command)
            
            if intent_result.intent == FlightIntent.UNKNOWN:
                response = {
                    'success': False,
                    'error': 'Unknown command',
                    'response': f"I couldn't understand the command '{command}'. Please try rephrasing or use commands like 'take off', 'land', 'move to coordinates', etc.",
                    'intent': intent_result.intent.value,
                    'confidence': intent_result.confidence
                }
                self._log_command(command, response)
                return response
            
            # Step 2: Validate parameters
            validation_errors = self.nlp_extractor.validate_parameters(
                intent_result.intent, intent_result.parameters
            )
            
            if validation_errors:
                error_msg = "; ".join(validation_errors.values())
                response = {
                    'success': False,
                    'error': 'Parameter validation failed',
                    'response': f"Command parameters are invalid: {error_msg}",
                    'intent': intent_result.intent.value,
                    'validation_errors': validation_errors
                }
                self._log_command(command, response)
                return response
            
            # Step 3: Plan mission using AI agent
            self.logger.info("Step 2: Planning mission with AI agent...")
            mission_plan = await self.flight_agent.plan_mission(intent_result)
            
            if mission_plan.status == MissionStatus.FAILED:
                response = {
                    'success': False,
                    'error': 'Mission planning failed',
                    'response': f"Cannot execute command due to safety concerns: {'; '.join(mission_plan.safety_notes)}",
                    'intent': intent_result.intent.value,
                    'safety_assessment': mission_plan.safety_assessment.value
                }
                self._log_command(command, response)
                return response
            
            # Step 4: Execute mission if MAVLink is connected
            if self.system_state.mavlink_connected:
                self.logger.info("Step 3: Executing mission...")
                self.system_state.status = SystemStatus.EXECUTING
                self.system_state.current_mission = mission_plan
                
                # Add to active missions
                self.active_missions[mission_plan.mission_id] = mission_plan
                
                # Execute mission
                execution_success = await self.mavlink_interface.execute_mission(mission_plan)
                
                if execution_success:
                    response = {
                        'success': True,
                        'response': f"Successfully executed: {command}",
                        'intent': intent_result.intent.value,
                        'mission_id': mission_plan.mission_id,
                        'safety_assessment': mission_plan.safety_assessment.value,
                        'estimated_duration': mission_plan.estimated_duration,
                        'commands_sent': len(mission_plan.mavlink_commands)
                    }
                else:
                    response = {
                        'success': False,
                        'error': 'Mission execution failed',
                        'response': f"Failed to execute command: {command}. Check MAVLink connection and vehicle status.",
                        'mission_id': mission_plan.mission_id
                    }
                
                # Remove from active missions
                if mission_plan.mission_id in self.active_missions:
                    del self.active_missions[mission_plan.mission_id]
                
                # Add to mission history
                self.mission_history.append(mission_plan)
                
            else:
                # MAVLink not connected - return plan without execution
                response = {
                    'success': True,
                    'response': f"Mission planned for: {command} (MAVLink not connected - simulation mode)",
                    'intent': intent_result.intent.value,
                    'mission_id': mission_plan.mission_id,
                    'safety_assessment': mission_plan.safety_assessment.value,
                    'estimated_duration': mission_plan.estimated_duration,
                    'commands_planned': len(mission_plan.mavlink_commands),
                    'simulation_mode': True
                }
                
                self.mission_history.append(mission_plan)
            
            # Log command and response
            self._log_command(command, response)
            
            # Update system state
            self.system_state.status = SystemStatus.READY
            self.system_state.last_response = response['response']
            self.system_state.current_mission = None
            
            # Notify callbacks
            self._notify_state_callbacks()
            self._notify_command_callbacks(command, response)
            
            return response
            
        except Exception as e:
            self.system_state.status = SystemStatus.ERROR
            self.system_state.error_message = str(e)
            self.logger.error(f"Command processing failed: {e}")
            
            response = {
                'success': False,
                'error': 'System error',
                'response': f"An error occurred while processing the command: {str(e)}"
            }
            
            self._log_command(command, response)
            return response
    
    def _on_vehicle_state_update(self, vehicle_state: VehicleState):
        """Handle vehicle state updates from MAVLink interface"""
        self.system_state.vehicle_state = vehicle_state
        self._notify_state_callbacks()
    
    def _log_command(self, command: str, response: Dict[str, Any]):
        """Log command and response to history"""
        log_entry = {
            'timestamp': time.time(),
            'command': command,
            'response': response,
            'system_status': self.system_state.status.value
        }
        
        self.command_history.append(log_entry)
        
        # Keep only last 100 commands
        if len(self.command_history) > 100:
            self.command_history.pop(0)
    
    def _notify_state_callbacks(self):
        """Notify all registered state callbacks"""
        for callback in self.state_callbacks:
            try:
                callback(self.system_state)
            except Exception as e:
                self.logger.error(f"State callback error: {e}")
    
    def _notify_command_callbacks(self, command: str, response: Dict[str, Any]):
        """Notify all registered command callbacks"""
        for callback in self.command_callbacks:
            try:
                callback(command, response)
            except Exception as e:
                self.logger.error(f"Command callback error: {e}")
    
    def add_state_callback(self, callback: callable):
        """Add a callback for system state updates"""
        self.state_callbacks.append(callback)
    
    def add_command_callback(self, callback: callable):
        """Add a callback for command processing events"""
        self.command_callbacks.append(callback)
    
    def get_system_state(self) -> SystemState:
        """Get current system state"""
        self.system_state.uptime = time.time() - self.start_time
        return self.system_state
    
    def get_command_history(self) -> List[Dict[str, Any]]:
        """Get command history"""
        return self.command_history.copy()
    
    def get_mission_history(self) -> List[Dict[str, Any]]:
        """Get mission history"""
        return [self.flight_agent.to_dict(mission) for mission in self.mission_history]
    
    def get_active_missions(self) -> Dict[str, Dict[str, Any]]:
        """Get currently active missions"""
        return {
            mission_id: self.flight_agent.to_dict(mission)
            for mission_id, mission in self.active_missions.items()
        }
    
    async def emergency_stop(self) -> bool:
        """Execute emergency stop"""
        self.logger.warning("EMERGENCY STOP REQUESTED")
        
        if self.mavlink_interface and self.system_state.mavlink_connected:
            success = await self.mavlink_interface.emergency_stop()
            
            # Clear active missions
            self.active_missions.clear()
            self.system_state.current_mission = None
            
            return success
        else:
            self.logger.warning("Emergency stop requested but MAVLink not connected")
            return False
    
    async def reconnect_mavlink(self) -> bool:
        """Attempt to reconnect MAVLink interface"""
        if self.mavlink_interface:
            self.logger.info("Attempting MAVLink reconnection...")
            connected = await self.mavlink_interface.connect()
            self.system_state.mavlink_connected = connected
            
            if connected:
                self.logger.info("MAVLink reconnected successfully")
            else:
                self.logger.error("MAVLink reconnection failed")
            
            self._notify_state_callbacks()
            return connected
        
        return False
    
    def shutdown(self):
        """Shutdown the agentic controller"""
        self.logger.info("Shutting down MADMAX AI Agentic System...")
        
        # Disconnect MAVLink
        if self.mavlink_interface:
            self.mavlink_interface.disconnect()
        
        # Clear active missions
        self.active_missions.clear()
        
        # Update status
        self.system_state.status = SystemStatus.OFFLINE
        self._notify_state_callbacks()
        
        self.logger.info("System shutdown complete")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert system state to dictionary for JSON serialization"""
        return {
            'system_state': {
                'status': self.system_state.status.value,
                'nlp_ready': self.system_state.nlp_ready,
                'agent_ready': self.system_state.agent_ready,
                'mavlink_connected': self.system_state.mavlink_connected,
                'last_command': self.system_state.last_command,
                'last_response': self.system_state.last_response,
                'error_message': self.system_state.error_message,
                'uptime': time.time() - self.start_time
            },
            'vehicle_state': self.mavlink_interface.to_dict() if self.mavlink_interface else None,
            'active_missions': self.get_active_missions(),
            'command_history_count': len(self.command_history),
            'mission_history_count': len(self.mission_history)
        }

# Example usage and testing
if __name__ == "__main__":
    import asyncio
    import os
    
    async def test_agentic_controller():
        # Initialize controller
        controller = AgenticController(
            mavlink_connection="tcp:127.0.0.1:5760",  # SITL for testing
            openai_api_key=os.getenv("OPENAI_API_KEY")
        )
        
        # Initialize system
        if await controller.initialize():
            print("System initialized successfully!")
            
            # Test commands
            test_commands = [
                "Take off to 10 meters",
                "Move to coordinates 47.641468, -122.140165",
                "Hover for 30 seconds",
                "Return home",
                "Land"
            ]
            
            for command in test_commands:
                print(f"\n--- Testing: {command} ---")
                result = await controller.process_natural_language_command(command)
                print(f"Success: {result['success']}")
                print(f"Response: {result['response']}")
                
                if not result['success']:
                    print(f"Error: {result.get('error', 'Unknown error')}")
                
                # Wait between commands
                await asyncio.sleep(2)
            
            # Show system state
            print(f"\n--- System State ---")
            state = controller.get_system_state()
            print(f"Status: {state.status.value}")
            print(f"Uptime: {state.uptime:.1f}s")
            print(f"Commands processed: {len(controller.get_command_history())}")
            
            # Shutdown
            controller.shutdown()
        else:
            print("System initialization failed!")
    
    # Run test
    asyncio.run(test_agentic_controller())
