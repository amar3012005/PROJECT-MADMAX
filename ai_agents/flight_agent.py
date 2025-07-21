#!/usr/bin/env python3
"""
MADMAX AI Agentic Aerial Robotics - Flight Agent
AI agent that plans and executes flight missions based on extracted intents.
"""

import asyncio
import json
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, asdict
from enum import Enum
import time
from loguru import logger
from crewai import Agent, Task, Crew, Process
from langchain.llms import OpenAI
from langchain.tools import Tool

from nlp_intent_extractor import FlightIntent, FlightParameters, IntentResult

class MissionStatus(Enum):
    """Mission execution status"""
    PENDING = "pending"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"

class SafetyLevel(Enum):
    """Safety assessment levels"""
    SAFE = "safe"
    CAUTION = "caution"
    DANGEROUS = "dangerous"
    PROHIBITED = "prohibited"

@dataclass
class MAVLinkCommand:
    """MAVLink command structure"""
    command_id: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    param5: float = 0.0  # x/lat
    param6: float = 0.0  # y/lon
    param7: float = 0.0  # z/alt
    target_system: int = 1
    target_component: int = 1
    frame: int = 0

@dataclass
class MissionPlan:
    """Flight mission plan"""
    mission_id: str
    intent: FlightIntent
    parameters: FlightParameters
    mavlink_commands: List[MAVLinkCommand]
    safety_assessment: SafetyLevel
    estimated_duration: float
    pre_flight_checks: List[str]
    safety_notes: List[str]
    status: MissionStatus = MissionStatus.PENDING

class FlightAgent:
    """AI agent for flight mission planning and execution"""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the flight agent"""
        self.logger = logger.bind(module="FlightAgent")
        
        # Initialize LLM (OpenAI or local model)
        if api_key:
            self.llm = OpenAI(openai_api_key=api_key, temperature=0.1)
        else:
            # Fallback to rule-based system if no API key
            self.llm = None
            self.logger.warning("No API key provided, using rule-based planning")
        
        # Initialize CrewAI agents
        self._setup_agents()
        
        # MAVLink command mappings
        self.mavlink_commands = {
            # Navigation commands
            'NAV_TAKEOFF': 22,
            'NAV_LAND': 21,
            'NAV_WAYPOINT': 16,
            'NAV_RETURN_TO_LAUNCH': 20,
            'NAV_LOITER_UNLIM': 17,
            'NAV_LOITER_TIME': 19,
            
            # Vehicle commands
            'CMD_COMPONENT_ARM_DISARM': 400,
            'CMD_SET_MODE': 176,
            'CMD_MISSION_START': 300,
            'CMD_DO_SET_HOME': 179,
            
            # Emergency commands
            'CMD_DO_FLIGHTTERMINATION': 185,
        }
        
        # Flight modes
        self.flight_modes = {
            'MANUAL': 0,
            'STABILIZE': 1,
            'ACRO': 2,
            'ALT_HOLD': 3,
            'AUTO': 4,
            'GUIDED': 5,
            'LOITER': 6,
            'RTL': 7,
            'CIRCLE': 8,
            'LAND': 9,
            'BRAKE': 17,
            'THROW': 18,
        }
        
        self.logger.info("Flight Agent initialized successfully")
    
    def _setup_agents(self):
        """Setup CrewAI agents for different aspects of flight planning"""
        
        # Safety Assessment Agent
        self.safety_agent = Agent(
            role='Flight Safety Officer',
            goal='Assess flight safety and identify potential risks',
            backstory="""You are an experienced drone safety officer responsible for 
            evaluating flight plans for safety compliance and risk assessment.""",
            verbose=True,
            allow_delegation=False,
            llm=self.llm
        )
        
        # Mission Planner Agent
        self.planner_agent = Agent(
            role='Mission Planner',
            goal='Create detailed flight plans and MAVLink command sequences',
            backstory="""You are an expert flight planner who converts high-level 
            flight intentions into precise MAVLink command sequences.""",
            verbose=True,
            allow_delegation=False,
            llm=self.llm
        )
        
        # Execution Monitor Agent
        self.monitor_agent = Agent(
            role='Flight Monitor',
            goal='Monitor flight execution and handle contingencies',
            backstory="""You are responsible for monitoring flight execution and 
            making real-time adjustments when needed.""",
            verbose=True,
            allow_delegation=False,
            llm=self.llm
        )
    
    async def plan_mission(self, intent_result: IntentResult) -> MissionPlan:
        """
        Plan a flight mission based on extracted intent
        
        Args:
            intent_result: Result from NLP intent extraction
            
        Returns:
            Complete mission plan with MAVLink commands
        """
        self.logger.info(f"Planning mission for intent: {intent_result.intent.value}")
        
        mission_id = f"mission_{int(time.time())}"
        
        # Assess safety first
        safety_assessment = await self._assess_safety(intent_result)
        
        if safety_assessment == SafetyLevel.PROHIBITED:
            self.logger.error("Mission prohibited due to safety concerns")
            return MissionPlan(
                mission_id=mission_id,
                intent=intent_result.intent,
                parameters=intent_result.parameters,
                mavlink_commands=[],
                safety_assessment=safety_assessment,
                estimated_duration=0,
                pre_flight_checks=[],
                safety_notes=["Mission prohibited due to safety violations"],
                status=MissionStatus.FAILED
            )
        
        # Generate MAVLink commands
        mavlink_commands = await self._generate_mavlink_commands(intent_result)
        
        # Estimate mission duration
        duration = self._estimate_duration(intent_result.intent, intent_result.parameters)
        
        # Generate pre-flight checks
        pre_flight_checks = self._generate_preflight_checks(intent_result.intent)
        
        # Generate safety notes
        safety_notes = self._generate_safety_notes(intent_result, safety_assessment)
        
        mission_plan = MissionPlan(
            mission_id=mission_id,
            intent=intent_result.intent,
            parameters=intent_result.parameters,
            mavlink_commands=mavlink_commands,
            safety_assessment=safety_assessment,
            estimated_duration=duration,
            pre_flight_checks=pre_flight_checks,
            safety_notes=safety_notes,
            status=MissionStatus.PLANNING
        )
        
        self.logger.info(f"Mission plan created: {mission_id}")
        return mission_plan
    
    async def _assess_safety(self, intent_result: IntentResult) -> SafetyLevel:
        """Assess safety of the requested mission"""
        
        # Rule-based safety assessment
        safety_level = SafetyLevel.SAFE
        
        # Check altitude limits
        if intent_result.parameters.altitude:
            if intent_result.parameters.altitude > 120:  # FAA limit
                safety_level = SafetyLevel.PROHIBITED
            elif intent_result.parameters.altitude > 100:
                safety_level = SafetyLevel.DANGEROUS
            elif intent_result.parameters.altitude > 50:
                safety_level = SafetyLevel.CAUTION
        
        # Check speed limits
        if intent_result.parameters.speed:
            if intent_result.parameters.speed > 20:  # 20 m/s max
                safety_level = SafetyLevel.DANGEROUS
            elif intent_result.parameters.speed > 15:
                safety_level = SafetyLevel.CAUTION
        
        # Emergency commands are always allowed
        if intent_result.intent == FlightIntent.EMERGENCY_STOP:
            safety_level = SafetyLevel.SAFE
        
        # Use AI agent for advanced safety assessment if available
        if self.llm and safety_level != SafetyLevel.PROHIBITED:
            try:
                safety_task = Task(
                    description=f"""Assess the safety of this flight mission:
                    Intent: {intent_result.intent.value}
                    Parameters: {asdict(intent_result.parameters)}
                    
                    Consider factors like:
                    - Weather conditions
                    - Airspace restrictions
                    - Battery requirements
                    - Equipment limitations
                    
                    Return only: SAFE, CAUTION, DANGEROUS, or PROHIBITED""",
                    agent=self.safety_agent
                )
                
                crew = Crew(
                    agents=[self.safety_agent],
                    tasks=[safety_task],
                    process=Process.sequential
                )
                
                result = crew.kickoff()
                ai_assessment = result.strip().upper()
                
                if ai_assessment in [level.name for level in SafetyLevel]:
                    ai_safety_level = SafetyLevel[ai_assessment]
                    # Take the more conservative assessment
                    if ai_safety_level.value > safety_level.value:
                        safety_level = ai_safety_level
                        
            except Exception as e:
                self.logger.warning(f"AI safety assessment failed: {e}")
        
        return safety_level
    
    async def _generate_mavlink_commands(self, intent_result: IntentResult) -> List[MAVLinkCommand]:
        """Generate MAVLink commands for the given intent"""
        
        commands = []
        intent = intent_result.intent
        params = intent_result.parameters
        
        if intent == FlightIntent.ARM:
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['CMD_COMPONENT_ARM_DISARM'],
                param1=1.0  # 1 = arm, 0 = disarm
            ))
        
        elif intent == FlightIntent.DISARM:
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['CMD_COMPONENT_ARM_DISARM'],
                param1=0.0  # 1 = arm, 0 = disarm
            ))
        
        elif intent == FlightIntent.TAKEOFF:
            altitude = params.altitude or 10.0  # Default 10m
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['NAV_TAKEOFF'],
                param7=altitude
            ))
        
        elif intent == FlightIntent.LAND:
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['NAV_LAND']
            ))
        
        elif intent == FlightIntent.MOVE_TO:
            if params.latitude and params.longitude:
                altitude = params.altitude or 20.0  # Default 20m
                commands.append(MAVLinkCommand(
                    command_id=self.mavlink_commands['NAV_WAYPOINT'],
                    param5=params.latitude,
                    param6=params.longitude,
                    param7=altitude
                ))
        
        elif intent == FlightIntent.HOVER:
            duration = params.duration or 30.0  # Default 30 seconds
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['NAV_LOITER_TIME'],
                param1=duration
            ))
        
        elif intent == FlightIntent.RETURN_HOME:
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['NAV_RETURN_TO_LAUNCH']
            ))
        
        elif intent == FlightIntent.EMERGENCY_STOP:
            commands.append(MAVLinkCommand(
                command_id=self.mavlink_commands['CMD_DO_FLIGHTTERMINATION'],
                param1=1.0  # Terminate flight
            ))
        
        elif intent == FlightIntent.SET_MODE:
            if params.mode and params.mode in self.flight_modes:
                commands.append(MAVLinkCommand(
                    command_id=self.mavlink_commands['CMD_SET_MODE'],
                    param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                    param2=self.flight_modes[params.mode]
                ))
        
        elif intent == FlightIntent.ORBIT:
            if params.radius and params.latitude and params.longitude:
                # Set up circular loiter
                commands.append(MAVLinkCommand(
                    command_id=self.mavlink_commands['NAV_LOITER_UNLIM'],
                    param3=params.radius,
                    param5=params.latitude,
                    param6=params.longitude,
                    param7=params.altitude or 20.0
                ))
        
        return commands
    
    def _estimate_duration(self, intent: FlightIntent, params: FlightParameters) -> float:
        """Estimate mission duration in seconds"""
        
        base_durations = {
            FlightIntent.TAKEOFF: 30.0,
            FlightIntent.LAND: 30.0,
            FlightIntent.HOVER: params.duration or 30.0,
            FlightIntent.MOVE_TO: 60.0,  # Base time, adjusted by distance
            FlightIntent.ORBIT: 120.0,
            FlightIntent.RETURN_HOME: 60.0,
            FlightIntent.ARM: 5.0,
            FlightIntent.DISARM: 5.0,
            FlightIntent.EMERGENCY_STOP: 10.0,
            FlightIntent.PATROL: 300.0,
            FlightIntent.SURVEY: 600.0,
        }
        
        duration = base_durations.get(intent, 60.0)
        
        # Adjust for distance if moving
        if intent == FlightIntent.MOVE_TO and params.distance:
            speed = params.speed or 5.0  # Default 5 m/s
            travel_time = params.distance / speed
            duration = max(duration, travel_time)
        
        return duration
    
    def _generate_preflight_checks(self, intent: FlightIntent) -> List[str]:
        """Generate pre-flight checklist based on intent"""
        
        base_checks = [
            "Verify GPS lock (minimum 6 satellites)",
            "Check battery level (minimum 30%)",
            "Confirm clear airspace",
            "Verify home position is set",
            "Check weather conditions"
        ]
        
        intent_specific_checks = {
            FlightIntent.TAKEOFF: [
                "Ensure takeoff area is clear",
                "Verify sufficient altitude clearance"
            ],
            FlightIntent.MOVE_TO: [
                "Validate destination coordinates",
                "Check route for obstacles"
            ],
            FlightIntent.ORBIT: [
                "Verify orbit center coordinates",
                "Ensure sufficient space for orbit radius"
            ]
        }
        
        checks = base_checks.copy()
        if intent in intent_specific_checks:
            checks.extend(intent_specific_checks[intent])
        
        return checks
    
    def _generate_safety_notes(self, intent_result: IntentResult, safety_level: SafetyLevel) -> List[str]:
        """Generate safety notes based on assessment"""
        
        notes = []
        
        if safety_level == SafetyLevel.DANGEROUS:
            notes.append("⚠️ HIGH RISK: This mission involves dangerous parameters")
        elif safety_level == SafetyLevel.CAUTION:
            notes.append("⚠️ CAUTION: Exercise extra care during this mission")
        
        # Parameter-specific notes
        if intent_result.parameters.altitude and intent_result.parameters.altitude > 50:
            notes.append(f"High altitude operation: {intent_result.parameters.altitude}m")
        
        if intent_result.parameters.speed and intent_result.parameters.speed > 10:
            notes.append(f"High speed operation: {intent_result.parameters.speed} m/s")
        
        return notes
    
    def to_dict(self, mission_plan: MissionPlan) -> Dict[str, Any]:
        """Convert mission plan to dictionary for JSON serialization"""
        return {
            'mission_id': mission_plan.mission_id,
            'intent': mission_plan.intent.value,
            'parameters': asdict(mission_plan.parameters),
            'mavlink_commands': [asdict(cmd) for cmd in mission_plan.mavlink_commands],
            'safety_assessment': mission_plan.safety_assessment.value,
            'estimated_duration': mission_plan.estimated_duration,
            'pre_flight_checks': mission_plan.pre_flight_checks,
            'safety_notes': mission_plan.safety_notes,
            'status': mission_plan.status.value
        }

# Example usage
if __name__ == "__main__":
    import asyncio
    from nlp_intent_extractor import NLPIntentExtractor
    
    async def test_flight_agent():
        # Initialize components
        extractor = NLPIntentExtractor()
        agent = FlightAgent()
        
        # Test command
        command = "Take off to 15 meters altitude"
        
        # Extract intent
        intent_result = extractor.extract_intent(command)
        print(f"Intent: {intent_result.intent.value}")
        
        # Plan mission
        mission_plan = await agent.plan_mission(intent_result)
        
        # Display results
        print(f"\nMission Plan:")
        print(f"ID: {mission_plan.mission_id}")
        print(f"Safety: {mission_plan.safety_assessment.value}")
        print(f"Duration: {mission_plan.estimated_duration}s")
        print(f"Commands: {len(mission_plan.mavlink_commands)}")
        
        for i, cmd in enumerate(mission_plan.mavlink_commands):
            print(f"  Command {i+1}: ID={cmd.command_id}, Alt={cmd.param7}")
    
    # Run test
    asyncio.run(test_flight_agent())
