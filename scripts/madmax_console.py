#!/usr/bin/env python3

from rich.console import Console
from rich.panel import Panel
from rich.layout import Layout
from rich.live import Live
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn
from rich.prompt import Prompt
from rich import box
from rich.text import Text
from rich.style import Style
import time
from datetime import datetime
import asyncio
from typing import List, Dict
import json
import os
import random
import signal
import sys
from asyncio import CancelledError

# Fix for asyncio on Windows if needed
if sys.platform == "win32":
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

# Initialize Rich console with custom styling
console = Console(style="bold cyan")

# ASCII Art Banner
BANNER = r"""
██████╗ ██████╗  ██████╗ ██╗  ██╗███████╗ ██████╗████████╗    ███╗   ███╗ █████╗ ██████╗ ███╗   ██╗
██╔══██╗██╔══██╗██╔═══██╗██║ ██╔╝██╔════╝██╔════╝╚══██╔══╝    ████╗ ████║██╔══██╗██╔══██╗████╗  ██║
██████╔╝██████╔╝██║   ██║█████╔╝ █████╗  ██║        ██║       ██╔████╔██║███████║██████╔╝██╔██╗ ██║
██╔═══╝ ██╔══██╗██║   ██║██╔═██╗ ██╔══╝  ██║        ██║       ██║╚██╔╝██║██╔══██║██╔═══╝ ██║╚██╗█
██║     ██║  ██║╚██████╔╝██║  ██╗███████╗╚██████╗   ██║       ██║ ╚═╝ ██║██║  ██║██║     ██║ ╚████║
╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝ ╚═════╝   ╚═╝       ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝  ╚═══╝
                                                                                                  
                         VISUAL COMMAND LINE - CONSOLE LOGGER FOR PROJECT-MADMAX
"""

class MadMaxConsole:
    def __init__(self):
        self.console = Console()
        self.layout = Layout()
        self.status_messages: List[str] = []
        self.command_history: List[str] = []
        self.drone_status = {
            "armed": False,
            "mode": "HOLD",
            "battery": 95.4,
            "altitude": 0.0,
            "position": {"lat": 47.641468, "lon": -122.140165},
            "connected": False,
            "gps_satellites": 10,
            "signal_strength": 87,
            "cpu_temp": 54,
            "memory_usage": 32,
            "autopilot": "PX4 SITL"
        }
        self.boot_sequence_complete = False
        self.simulation_step = 0
        self.sitl_running = False
        self.running = True
        self.setup_signal_handlers()
        
    def setup_signal_handlers(self):
        """Set up signal handlers for clean exit"""
        signal.signal(signal.SIGINT, self.handle_exit)
        signal.signal(signal.SIGTERM, self.handle_exit)
        
    def handle_exit(self, sig, frame):
        """Handle exit signals gracefully"""
        self.running = False
        print("\n\nExiting Project Madmax Console...\n")
        sys.exit(0)
        
    def create_header(self) -> Panel:
        """Create the header panel with project title and status"""
        header = Table.grid(padding=1)
        header.add_column("Title", style="bold cyan")
        header.add_column("Status", justify="right", style="bold green")
        
        status_text = "🟢 SYSTEM READY" if self.drone_status["connected"] else "🔴 OFFLINE"
        if not self.boot_sequence_complete:
            status_text = "⚡ BOOTING..."
            
        header.add_row(
            "🚀 PROJECT-MADMAX",
            status_text
        )
        
        return Panel(header, title="System Status", border_style="blue", box=box.DOUBLE)

    def create_status_panel(self) -> Panel:
        """Create the status panel showing drone telemetry"""
        status_table = Table.grid(padding=1)
        status_table.add_column("Metric", style="bold yellow")
        status_table.add_column("Value", justify="right", style="cyan")
        
        # Add cyberpunk-style status indicators
        armed_status = "[green]ACTIVE[/green]" if self.drone_status["armed"] else "[red]STANDBY[/red]"
        mode_status = f"[cyan]{self.drone_status['mode']}[/cyan]"
        battery_status = f"[{'green' if self.drone_status['battery'] > 20 else 'red'}]{self.drone_status['battery']:.1f}%[/]"
        altitude_status = f"[cyan]{self.drone_status['altitude']:.1f}m[/cyan]"
        pos_status = f"[cyan]Lat: {self.drone_status['position']['lat']:.6f}\nLon: {self.drone_status['position']['lon']:.6f}[/cyan]"
        gps_status = f"[cyan]{self.drone_status['gps_satellites']} satellites[/cyan]"
        signal_status = f"[{'green' if self.drone_status['signal_strength'] > 50 else 'yellow'}]{self.drone_status['signal_strength']}%[/]"
        temp_status = f"[{'red' if self.drone_status['cpu_temp'] > 70 else 'green'}]{self.drone_status['cpu_temp']}°C[/]"
        mem_status = f"[cyan]{self.drone_status['memory_usage']}%[/cyan]"
        autopilot_status = f"[cyan]{self.drone_status.get('autopilot', 'Unknown')}[/cyan]"
        
        status_table.add_row("AUTOPILOT", autopilot_status)
        status_table.add_row("SYSTEM STATUS", armed_status)
        status_table.add_row("FLIGHT MODE", mode_status)
        status_table.add_row("POWER LEVEL", battery_status)
        status_table.add_row("ALTITUDE", altitude_status)
        status_table.add_row("POSITION", pos_status)
        status_table.add_row("GPS LOCK", gps_status)
        status_table.add_row("SIGNAL STRENGTH", signal_status)
        status_table.add_row("CPU TEMP", temp_status)
        status_table.add_row("MEMORY USAGE", mem_status)
        
        return Panel(status_table, title="Drone Telemetry", border_style="green", box=box.DOUBLE)

    def create_log_panel(self) -> Panel:
        """Create the log panel showing recent messages"""
        log_text = "\n".join(self.status_messages[-10:])  # Show last 10 messages
        return Panel(log_text, title="System Log", border_style="yellow", box=box.DOUBLE)

    def add_status_message(self, message: str, level: str = "INFO"):
        """Add a new status message with timestamp and level"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        level_colors = {
            "INFO": "cyan",
            "WARN": "yellow",
            "ERROR": "red",
            "SUCCESS": "green",
            "SYSTEM": "blue",
            "DEBUG": "magenta"
        }
        color = level_colors.get(level, "white")
        self.status_messages.append(f"[{timestamp}] [{color}]{level}[/{color}] {message}")

    def simulate_system_state(self):
        """Simulate changing system states"""
        self.simulation_step += 1
        
        # Simulate GPS satellites
        if self.simulation_step > 5:
            self.drone_status["gps_satellites"] = min(12, self.drone_status["gps_satellites"] + random.randint(-1, 1))
            
        # Simulate signal strength
        self.drone_status["signal_strength"] = min(100, max(0, 
            self.drone_status["signal_strength"] + random.randint(-2, 2)))
            
        # Simulate CPU temperature
        self.drone_status["cpu_temp"] = min(90, max(30,
            self.drone_status["cpu_temp"] + random.randint(-1, 1)))
            
        # Simulate memory usage
        self.drone_status["memory_usage"] = min(100, max(0,
            self.drone_status["memory_usage"] + random.randint(-1, 1)))
            
        # Simulate battery drain
        if self.simulation_step > 10:
            self.drone_status["battery"] = max(0, self.drone_status["battery"] - 0.05)
            
        # Simulate altitude changes
        if self.simulation_step > 15 and self.drone_status["armed"]:
            self.drone_status["altitude"] += 0.05
            
        # Simulate position changes
        if self.simulation_step > 20:
            self.drone_status["position"]["lat"] += 0.000001 * random.random()
            self.drone_status["position"]["lon"] += 0.000001 * random.random()

    def create_command_panel(self) -> Panel:
        """Create the command input panel"""
        command_help = "[cyan]Ctrl+C to exit[/cyan]  |  [yellow]Type 'help' for commands[/yellow]"
        return Panel(
            command_help,
            title="Command Interface",
            border_style="magenta",
            box=box.DOUBLE
        )

    def display(self):
        """Display the full console interface"""
        # Split the layout into sections
        self.layout.split(
            Layout(name="header", size=3),
            Layout(name="main"),
            Layout(name="footer", size=3)
        )
        
        # Split main section into columns
        self.layout["main"].split_row(
            Layout(name="status", ratio=1),
            Layout(name="logs", ratio=1)
        )
        
        # Add content to each section
        self.layout["header"].update(self.create_header())
        self.layout["status"].update(self.create_status_panel())
        self.layout["logs"].update(self.create_log_panel())
        self.layout["footer"].update(self.create_command_panel())
        
        return self.layout

    async def run(self):
        """Main console loop"""
        try:
            # Clear screen and show banner
            os.system('clear' if os.name == 'posix' else 'cls')
            console.print(BANNER, style="bold cyan")

            # Simulate boot sequence
            self.add_status_message("Initializing PROJECT-MADMAX Visual CLI Console...", "SYSTEM")
            await asyncio.sleep(1)
            self.add_status_message("Boot sequence started.", "SYSTEM")
            await asyncio.sleep(0.5)
            self.add_status_message("Initializing LoRa Meshtastic module...", "INFO")
            await asyncio.sleep(0.5)
            self.add_status_message("Establishing connection to Pixhawk...", "INFO")
            await asyncio.sleep(0.5)
            self.add_status_message("GPS module initializing...", "INFO")
            await asyncio.sleep(0.5)
            self.add_status_message("Calibrating sensors...", "INFO")
            await asyncio.sleep(0.5)
            self.add_status_message("All systems nominal. Listening for real-time control inputs.", "SUCCESS")
            self.boot_sequence_complete = True
            self.drone_status["connected"] = True
            
            # Main display loop with simplified input handling (more robust)
            with Live(self.display(), refresh_per_second=1, screen=True) as live:
                # After setup is complete, show a manual prompt
                print("\nType commands directly in terminal. Available commands: arm, disarm, status, view, exit")
                
                while self.running:
                    # Update the layout
                    live.update(self.display())
                    
                    # Simulate system state changes
                    self.simulate_system_state()
                    
                    # Occasional system messages with lower frequency
                    if random.random() < 0.03:
                        messages = [
                            ("GPS signal strength stable", "INFO"),
                            ("CPU temperature normal", "DEBUG"),
                            ("Memory usage optimal", "DEBUG"),
                            ("LoRa packet received", "INFO"),
                            ("Battery monitoring active", "INFO"),
                            ("Navigation system online", "SUCCESS")
                        ]
                        msg, level = random.choice(messages)
                        self.add_status_message(msg, level)
                    
                    # Process manual input commands 
                    # This is a simplified approach to prevent asyncio issues
                    await asyncio.sleep(0.5)
                    
        except KeyboardInterrupt:
            # Handle Ctrl+C gracefully
            self.add_status_message("User requested shutdown", "SYSTEM")
            self.running = False
            print("\n\nShutdown complete\n")
        except Exception as e:
            # Log any unexpected errors
            error_msg = f"Console error: {str(e)}"
            self.add_status_message(error_msg, "ERROR")
            print(f"\nError: {error_msg}\n")
            import traceback
            traceback.print_exc()

    # ASCII Art Hexacopter (top view)
    HEXACOPTER_ASCII = r"""
          ╭- - -╮
        ╭-O     O-╮
       /           \
      /             \
     |               |
     |               |
╭- - O     HEXA      O- - ╮
|    |               |    |
|    |               |    |
╰- - O               O- - ╯
      \             /
       \           /
        ╰-O     O-╯
          ╰- - -╯

    O = Motors     - = Arms     HEXA = Flight Controller
    """

    async def process_user_input(self, queue):
        """Process user input from stdin in a non-blocking way"""
        # This method is simplified to avoid asyncio issues
        while self.running:
            try:
                print("\n[PROJECT MADMAX] Enter command: ", end="", flush=True)
                command = input()
                if command.strip():
                    await self.execute_command(command)
            except Exception as e:
                print(f"Input error: {str(e)}")
            await asyncio.sleep(0.1)

    async def execute_command(self, command):
        """Execute user commands"""
        command = command.strip().lower()
        self.command_history.append(command)
        
        if command == "help":
            self.add_status_message("Available commands: arm, disarm, mode, status, view, exit", "INFO")
        elif command == "arm":
            if self.drone_status["armed"]:
                self.add_status_message("PROJECT MADMAX is already armed!", "WARN")
            else:
                # Enhanced MAVLink-style arming sequence
                self.add_status_message("Initializing arming sequence for PROJECT MADMAX...", "SYSTEM")
                await asyncio.sleep(0.3)
                
                # Simulate MAVLink protocol communications
                self.add_status_message("MAVLink: Sending COMMAND_LONG (CMD_COMPONENT_ARM_DISARM)", "DEBUG")
                await asyncio.sleep(0.2)
                self.add_status_message("MAVLink: TX: 0xFE 0x16 0x01 0x9C 0x7E 0x00 0x01...", "DEBUG")
                await asyncio.sleep(0.3)
                self.add_status_message("MAVLink: RX: ACK received (0x01 0x9C 0x00)", "DEBUG")
                await asyncio.sleep(0.2)
                
                # Simulate pre-arm checks with protocol details
                self.add_status_message("Running FC pre-arm safety checks...", "SYSTEM")
                await asyncio.sleep(0.4)
                self.add_status_message("Telemetry: Verifying sensor calibration status...", "INFO")
                await asyncio.sleep(0.3)
                self.add_status_message("Telemetry: IMU calibration validated [PASSED]", "SUCCESS")
                await asyncio.sleep(0.2)
                self.add_status_message("Telemetry: Compass calibration validated [PASSED]", "SUCCESS")
                await asyncio.sleep(0.2)
                
                # Battery checks
                self.add_status_message("MAVLink: Request BATTERY_STATUS", "DEBUG")
                await asyncio.sleep(0.2)
                self.add_status_message(f"Telemetry: Battery level {self.drone_status['battery']:.1f}% [PASSED]", "INFO")
                await asyncio.sleep(0.2)
                
                # GPS and position checks
                self.add_status_message("MAVLink: Request GPS_RAW_INT", "DEBUG")
                await asyncio.sleep(0.3)
                self.add_status_message(f"Telemetry: GPS satellites: {self.drone_status['gps_satellites']} [PASSED]", "INFO")
                await asyncio.sleep(0.2)
                self.add_status_message(f"Telemetry: Position accuracy: 0.8m [PASSED]", "INFO")
                await asyncio.sleep(0.3)
                
                # RC checks
                self.add_status_message("MAVLink: Request RC_CHANNELS", "DEBUG")
                await asyncio.sleep(0.25)
                self.add_status_message("Telemetry: RC control inputs verified [PASSED]", "SUCCESS")
                await asyncio.sleep(0.2)
                
                # Final arming sequence
                self.add_status_message("All pre-arm checks complete. Authorization code: 0xA7F9D1", "SYSTEM")
                await asyncio.sleep(0.5)
                self.add_status_message("MAVLink: Sending ARM command with parameters (force=0, confirm=1)", "DEBUG")
                await asyncio.sleep(0.4)
                self.add_status_message("MAVLink: Received ARM_ACK (result=0, success)", "SUCCESS")
                await asyncio.sleep(0.3)
                
                # ESC calibration and motor initialization
                self.add_status_message("Initializing ESC protocols and motor controllers...", "SYSTEM")
                await asyncio.sleep(0.4)
                self.add_status_message("ESC: Calibration signals sent to all 6 motor controllers", "INFO")
                await asyncio.sleep(0.3)
                self.add_status_message("Motors 1-6: Initialization tone sequence complete", "SUCCESS")
                await asyncio.sleep(0.2)
                
                # Set system to armed state
                self.drone_status["armed"] = True
                self.drone_status["mode"] = "LOITER"
                
                # Final armed confirmation
                self.add_status_message("SYSTEM ARMED: Vehicle is now in LOITER mode", "WARN")
                self.add_status_message("MAVLink: Heartbeat showing armed status confirmed", "SUCCESS")
                self.add_status_message("!! CAUTION: MOTORS ARE ACTIVE !!", "WARN")
                
                # Ready for flight
                self.add_status_message("PROJECT MADMAX ready for flight operations", "INFO")
                
        elif command == "disarm":
            if not self.drone_status["armed"]:
                self.add_status_message("PROJECT MADMAX is already disarmed", "INFO")
            else:
                # Enhanced disarming sequence with protocol details
                self.add_status_message("Initiating disarm sequence...", "WARN")
                await asyncio.sleep(0.3)
                
                self.add_status_message("MAVLink: Sending COMMAND_LONG (CMD_COMPONENT_ARM_DISARM, param1=0)", "DEBUG")
                await asyncio.sleep(0.4)
                self.add_status_message("MAVLink: RX: Command ACK (DISARM accepted)", "SUCCESS")
                await asyncio.sleep(0.3)
                
                # Motor shutdown sequence
                self.add_status_message("ESC: Sending motor stop commands to all controllers", "INFO")
                await asyncio.sleep(0.3)
                self.add_status_message("Motors 1-6: Powering down", "INFO")
                await asyncio.sleep(0.5)
                
                # Update drone status
                self.drone_status["armed"] = False
                self.drone_status["mode"] = "HOLD"
                
                self.add_status_message("SYSTEM DISARMED: All motors stopped", "SUCCESS")
                self.add_status_message("MAVLink: Heartbeat showing disarmed status confirmed", "SUCCESS")
                
        elif command == "status":
            self.add_status_message(f"PROJECT MADMAX Status: {'Armed' if self.drone_status['armed'] else 'Disarmed'}, " + 
                                   f"Mode: {self.drone_status['mode']}, " +
                                   f"Battery: {self.drone_status['battery']:.1f}%, " +
                                   f"Altitude: {self.drone_status['altitude']:.1f}m", "INFO")
        elif command == "view":
            # Show hexacopter ASCII art view
            self.add_status_message("Displaying hexacopter top view", "INFO")
            for line in self.HEXACOPTER_ASCII.strip().split("\n"):
                self.add_status_message(line, "SYSTEM")
        elif command == "exit" or command == "quit":
            self.add_status_message("Exiting PROJECT MADMAX Console", "SYSTEM")
            self.running = False
        else:
            self.add_status_message(f"Unknown command: {command}", "ERROR")