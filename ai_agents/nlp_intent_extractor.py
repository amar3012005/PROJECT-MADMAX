#!/usr/bin/env python3
"""
MADMAX AI Agentic Aerial Robotics - NLP Intent Extractor
Processes natural language input and extracts flight intentions and parameters.
"""

import re
import json
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import spacy
from transformers import pipeline
from loguru import logger

class FlightIntent(Enum):
    """Enumeration of supported flight intentions"""
    TAKEOFF = "takeoff"
    LAND = "land"
    HOVER = "hover"
    MOVE_TO = "move_to"
    FOLLOW = "follow"
    ORBIT = "orbit"
    RETURN_HOME = "return_home"
    ARM = "arm"
    DISARM = "disarm"
    EMERGENCY_STOP = "emergency_stop"
    SET_MODE = "set_mode"
    PATROL = "patrol"
    SURVEY = "survey"
    UNKNOWN = "unknown"

@dataclass
class FlightParameters:
    """Parameters extracted from natural language input"""
    altitude: Optional[float] = None
    speed: Optional[float] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    heading: Optional[float] = None
    distance: Optional[float] = None
    radius: Optional[float] = None
    duration: Optional[float] = None
    waypoints: Optional[List[Tuple[float, float]]] = None
    mode: Optional[str] = None

@dataclass
class IntentResult:
    """Result of intent extraction"""
    intent: FlightIntent
    parameters: FlightParameters
    confidence: float
    raw_text: str
    processed_text: str

class NLPIntentExtractor:
    """Natural Language Processing Intent Extractor for drone commands"""
    
    def __init__(self):
        """Initialize the NLP intent extractor"""
        self.logger = logger.bind(module="NLPIntentExtractor")
        
        # Load spaCy model for NER and linguistic processing
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.logger.warning("spaCy model not found. Install with: python -m spacy download en_core_web_sm")
            self.nlp = None
        
        # Initialize transformer pipeline for intent classification
        try:
            self.classifier = pipeline(
                "text-classification",
                model="microsoft/DialoGPT-medium",
                return_all_scores=True
            )
        except Exception as e:
            self.logger.warning(f"Could not load transformer model: {e}")
            self.classifier = None
        
        # Define intent patterns
        self.intent_patterns = {
            FlightIntent.TAKEOFF: [
                r"take\s*off", r"launch", r"lift\s*off", r"ascend", r"go\s*up"
            ],
            FlightIntent.LAND: [
                r"land", r"descend", r"come\s*down", r"touch\s*down"
            ],
            FlightIntent.HOVER: [
                r"hover", r"stay\s*in\s*place", r"hold\s*position", r"maintain\s*altitude"
            ],
            FlightIntent.MOVE_TO: [
                r"go\s*to", r"move\s*to", r"fly\s*to", r"navigate\s*to", r"head\s*to"
            ],
            FlightIntent.FOLLOW: [
                r"follow", r"track", r"pursue", r"shadow"
            ],
            FlightIntent.ORBIT: [
                r"orbit", r"circle", r"revolve\s*around", r"loop\s*around"
            ],
            FlightIntent.RETURN_HOME: [
                r"return\s*home", r"go\s*back", r"rtl", r"return\s*to\s*launch"
            ],
            FlightIntent.ARM: [
                r"arm", r"enable\s*motors", r"prepare\s*for\s*flight"
            ],
            FlightIntent.DISARM: [
                r"disarm", r"disable\s*motors", r"shut\s*down\s*motors"
            ],
            FlightIntent.EMERGENCY_STOP: [
                r"emergency\s*stop", r"stop\s*immediately", r"abort", r"kill\s*switch"
            ],
            FlightIntent.PATROL: [
                r"patrol", r"surveillance", r"monitor\s*area", r"guard"
            ],
            FlightIntent.SURVEY: [
                r"survey", r"map", r"scan\s*area", r"inspect"
            ]
        }
        
        # Parameter extraction patterns
        self.parameter_patterns = {
            'altitude': r'(?:altitude|height|elevation)\s*(?:of\s*)?(\d+(?:\.\d+)?)\s*(?:m|meters|ft|feet)?',
            'speed': r'(?:speed|velocity)\s*(?:of\s*)?(\d+(?:\.\d+)?)\s*(?:m/s|mps|mph|kmh)?',
            'distance': r'(?:distance|range)\s*(?:of\s*)?(\d+(?:\.\d+)?)\s*(?:m|meters|km|miles)?',
            'heading': r'(?:heading|direction|bearing)\s*(?:of\s*)?(\d+(?:\.\d+)?)\s*(?:degrees?|deg)?',
            'radius': r'(?:radius)\s*(?:of\s*)?(\d+(?:\.\d+)?)\s*(?:m|meters)?',
            'duration': r'(?:for|duration)\s*(\d+(?:\.\d+)?)\s*(?:seconds?|minutes?|hours?)',
            'coordinates': r'(?:coordinates?|position|location)\s*(?:at\s*)?(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)'
        }
        
        self.logger.info("NLP Intent Extractor initialized successfully")
    
    def extract_intent(self, text: str) -> IntentResult:
        """
        Extract flight intent and parameters from natural language text
        
        Args:
            text: Natural language input from user
            
        Returns:
            IntentResult containing intent, parameters, and confidence
        """
        self.logger.info(f"Processing input: {text}")
        
        # Preprocess text
        processed_text = self._preprocess_text(text)
        
        # Extract intent
        intent, confidence = self._classify_intent(processed_text)
        
        # Extract parameters
        parameters = self._extract_parameters(processed_text)
        
        result = IntentResult(
            intent=intent,
            parameters=parameters,
            confidence=confidence,
            raw_text=text,
            processed_text=processed_text
        )
        
        self.logger.info(f"Extracted intent: {intent.value} (confidence: {confidence:.2f})")
        return result
    
    def _preprocess_text(self, text: str) -> str:
        """Preprocess and normalize input text"""
        # Convert to lowercase
        text = text.lower().strip()
        
        # Remove extra whitespace
        text = re.sub(r'\s+', ' ', text)
        
        # Handle common abbreviations
        text = text.replace('rtl', 'return to launch')
        text = text.replace('alt', 'altitude')
        text = text.replace('pos', 'position')
        
        return text
    
    def _classify_intent(self, text: str) -> Tuple[FlightIntent, float]:
        """Classify the flight intent from preprocessed text"""
        best_intent = FlightIntent.UNKNOWN
        best_confidence = 0.0
        
        # Pattern-based classification
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                if re.search(pattern, text, re.IGNORECASE):
                    # Calculate confidence based on pattern match strength
                    match_strength = len(re.findall(pattern, text, re.IGNORECASE))
                    confidence = min(0.9, 0.6 + (match_strength * 0.1))
                    
                    if confidence > best_confidence:
                        best_intent = intent
                        best_confidence = confidence
        
        # Use transformer model if available for additional validation
        if self.classifier and best_confidence < 0.8:
            try:
                results = self.classifier(text)
                # This is a simplified approach - in practice, you'd train a custom model
                # for drone command classification
                if results and len(results) > 0:
                    transformer_confidence = max([r['score'] for r in results[0]])
                    if transformer_confidence > best_confidence:
                        best_confidence = min(0.95, transformer_confidence)
            except Exception as e:
                self.logger.warning(f"Transformer classification failed: {e}")
        
        return best_intent, best_confidence
    
    def _extract_parameters(self, text: str) -> FlightParameters:
        """Extract flight parameters from preprocessed text"""
        params = FlightParameters()
        
        # Extract numerical parameters
        for param_name, pattern in self.parameter_patterns.items():
            matches = re.findall(pattern, text, re.IGNORECASE)
            if matches:
                if param_name == 'coordinates':
                    # Handle coordinate pairs
                    if len(matches[0]) == 2:
                        params.latitude = float(matches[0][0])
                        params.longitude = float(matches[0][1])
                else:
                    # Handle single numerical values
                    value = float(matches[0])
                    setattr(params, param_name, value)
        
        # Extract mode if specified
        mode_match = re.search(r'(?:mode|flight\s*mode)\s*(?:to\s*)?(\w+)', text, re.IGNORECASE)
        if mode_match:
            params.mode = mode_match.group(1).upper()
        
        # Extract waypoints if multiple coordinates are specified
        waypoint_pattern = r'waypoint\s*(?:at\s*)?(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)'
        waypoint_matches = re.findall(waypoint_pattern, text, re.IGNORECASE)
        if waypoint_matches:
            params.waypoints = [(float(lat), float(lon)) for lat, lon in waypoint_matches]
        
        return params
    
    def validate_parameters(self, intent: FlightIntent, parameters: FlightParameters) -> Dict[str, str]:
        """
        Validate extracted parameters against intent requirements
        
        Returns:
            Dictionary of validation errors (empty if all valid)
        """
        errors = {}
        
        # Validation rules based on intent
        if intent == FlightIntent.TAKEOFF:
            if parameters.altitude is not None and parameters.altitude > 120:  # FAA limit
                errors['altitude'] = "Altitude exceeds maximum allowed (120m)"
        
        elif intent == FlightIntent.MOVE_TO:
            if parameters.latitude is None or parameters.longitude is None:
                errors['coordinates'] = "Move to command requires coordinates"
        
        elif intent == FlightIntent.ORBIT:
            if parameters.radius is None:
                errors['radius'] = "Orbit command requires radius"
            if parameters.radius is not None and parameters.radius < 5:
                errors['radius'] = "Orbit radius too small (minimum 5m)"
        
        # General parameter validation
        if parameters.altitude is not None and parameters.altitude < 0:
            errors['altitude'] = "Altitude cannot be negative"
        
        if parameters.speed is not None and parameters.speed > 20:  # 20 m/s max
            errors['speed'] = "Speed exceeds maximum allowed (20 m/s)"
        
        return errors

# Example usage and testing
if __name__ == "__main__":
    extractor = NLPIntentExtractor()
    
    # Test cases
    test_commands = [
        "Take off to 10 meters altitude",
        "Fly to coordinates 47.641468, -122.140165 at speed 5 m/s",
        "Land immediately",
        "Orbit around current position with radius 20 meters",
        "Return home",
        "Hover at current position for 30 seconds",
        "Emergency stop now!"
    ]
    
    for command in test_commands:
        result = extractor.extract_intent(command)
        print(f"\nCommand: {command}")
        print(f"Intent: {result.intent.value}")
        print(f"Confidence: {result.confidence:.2f}")
        print(f"Parameters: {result.parameters}")
        
        # Validate parameters
        errors = extractor.validate_parameters(result.intent, result.parameters)
        if errors:
            print(f"Validation errors: {errors}")
