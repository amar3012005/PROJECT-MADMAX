#!/usr/bin/env python3

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich import box
import time
from datetime import datetime
import os
import random
import signal
import sys

# Initialize Rich console
console = Console()

# ASCII Art Banner
BANNER = r"""
██████╗ ██████╗  ██████╗ ██╗  ██╗███████╗ ██████╗████████╗    ███╗   ███╗ █████╗ ██████╗ ███╗   ██╗
██╔══██╗██╔══██╗██╔═══██╗██║ ██╔╝██╔════╝██╔════╝╚══██╔══╝    ████╗ ████║██╔══██╗██╔══██╗████╗  ██║
██████╔╝██████╔╝██║   ██║█████╔╝ █████╗  ██║        ██║       ██╔████╔██║███████║██████╔╝██╔██╗ ██║
██╔═══╝ ██╔══██╗██║   ██║██╔═██╗ ██╔══╝  ██║        ██║       ██║╚██╔╝██║██╔══██║██╔═══╝ ██║╚██╗██║
██║     ██║  ██║╚██████╔╝██║  ██╗███████╗╚██████╗   ██║       ██║ ╚═╝ ██║██║  ██║██║     ██║ ╚████║
╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝ ╚═════╝   ╚═╝       ╚═╝     ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝  ╚═══╝
                                                                                                  
                         VISUAL COMMAND LINE - CONSOLE LOGGER FOR PROJECT-MADMAX
"""

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

class MadMaxSimpleConsole:
    def __init__(self):
        self.status_messages = []
        self.drone_status = {
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
            "autopilot": "PX4 SITL"
        }
        self.running = True
        
    def add_message(self, message, level="INFO"):
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
        if len(self.status_messages) > 15:
            self.status_messages.pop(0)
    
    def simulate_system_state(self):
        # Update system states
        self.drone_status["signal_strength"] = min(100, max(0, 
            self.drone_status["signal_strength"] + random.randint(-2, 2)))
        self.drone_status["cpu_temp"] = min(90, max(30,
            self.drone_status["cpu_temp"] + random.randint(-1, 1)))
        self.drone_status["memory_usage"] = min(100, max(0,
            self.drone_status["memory_usage"] + random.randint(-1, 1)))
        self.drone_status["battery"] = max(0, self.drone_status["battery"] - 0.05)
        
        if self.drone_status["armed"]:
            self.drone_status["altitude"] += 0.1
            self.drone_status["position"]["lat"] += 0.000001
            self.drone_status["position"]["lon"] += 0.000001
    
    def display_status(self):
        os.system('clear' if os.name == 'posix' else 'cls')
        console.print(BANNER, style="bold cyan")
        
        # Status panel
        status_table = Table(title="Drone Telemetry", box=box.DOUBLE)
        status_table.add_column("Metric", style="bold yellow")
        status_table.add_column("Value", style="cyan")
        
        armed_status = "[green]ACTIVE[/green]" if self.drone_status["armed"] else "[red]STANDBY[/red]"
        
        status_table.add_row("SYSTEM STATUS", armed_status)
        status_table.add_row("FLIGHT MODE", self.drone_status["mode"])
        status_table.add_row("BATTERY", f"{self.drone_status['battery']:.1f}%")
        status_table.add_row("ALTITUDE", f"{self.drone_status['altitude']:.1f}m")
        status_table.add_row("POSITION", f"Lat: {self.drone_status['position']['lat']:.6f}\nLon: {self.drone_status['position']['lon']:.6f}")
        
        console.print(status_table)
        
        # Log panel
        log_panel = Panel("\n".join(self.status_messages), 
                         title="System Log", 
                         border_style="yellow", 
                         box=box.DOUBLE)
        console.print(log_panel)
        
        # Command help
        console.print("[cyan]Available commands: arm, disarm, status, view, exit[/cyan]")
        
    def run(self):
        # Boot sequence
        os.system('clear' if os.name == 'posix' else 'cls')
        console.print(BANNER, style="bold cyan")
        
        self.add_message("Initializing PROJECT-MADMAX Visual CLI Console...", "SYSTEM")
        time.sleep(1)
        self.add_message("Boot sequence started.", "SYSTEM")
        time.sleep(0.5)
        self.add_message("Establishing connection to Pixhawk...", "INFO")
        time.sleep(0.5)
        self.add_message("All systems nominal.", "SUCCESS")
        
        # Main loop
        while self.running:
            self.simulate_system_state()
            self.display_status()
            
            if random.random() < 0.3:
                messages = [
                    ("GPS signal strength stable", "INFO"),
                    ("CPU temperature normal", "DEBUG"),
                    ("Memory usage optimal", "DEBUG"),
                ]
                msg, level = random.choice(messages)
                self.add_message(msg, level)
            
            # Get command
            try:
                command = input("\n[PROJECT MADMAX] Enter command: ").strip().lower()
                
                if command == "arm":
                    if self.drone_status["armed"]:
                        self.add_message("PROJECT MADMAX is already armed!", "WARN")
                    else:
                        self.add_message("Pre-arm checks in progress...", "SYSTEM")
                        time.sleep(1)
                        self.add_message("All checks passed. Arming motors!", "SUCCESS")
                        self.drone_status["armed"] = True
                        self.drone_status["mode"] = "LOITER"
                        self.add_message("PROJECT MADMAX ARMED - MOTORS ACTIVE", "WARN")
                
                elif command == "disarm":
                    if not self.drone_status["armed"]:
                        self.add_message("PROJECT MADMAX is already disarmed", "INFO")
                    else:
                        self.add_message("Disarming motors...", "WARN")
                        time.sleep(0.5)
                        self.drone_status["armed"] = False
                        self.drone_status["mode"] = "HOLD"
                        self.add_message("PROJECT MADMAX DISARMED - Motors stopped", "SUCCESS")
                
                elif command == "status":
                    self.add_message(f"PROJECT MADMAX Status: {'Armed' if self.drone_status['armed'] else 'Disarmed'}, " + 
                                    f"Mode: {self.drone_status['mode']}, " +
                                    f"Battery: {self.drone_status['battery']:.1f}%", "INFO")
                
                elif command == "view":
                    self.add_message("Displaying hexacopter top view", "INFO")
                    for line in HEXACOPTER_ASCII.strip().split("\n"):
                        self.add_message(line, "SYSTEM")
                
                elif command in ["exit", "quit"]:
                    self.add_message("Exiting PROJECT MADMAX Console", "SYSTEM")
                    self.running = False
                    break
                
                elif command == "help":
                    self.add_message("Available commands: arm, disarm, status, view, exit", "INFO")
                
                else:
                    self.add_message(f"Unknown command: {command}", "ERROR")
                    
                time.sleep(1)  # Give time to see the command result
                
            except KeyboardInterrupt:
                self.add_message("User requested shutdown", "SYSTEM")
                self.running = False
                break
            except Exception as e:
                self.add_message(f"Error: {str(e)}", "ERROR")

def main():
    try:
        console = MadMaxSimpleConsole()
        console.run()
    except KeyboardInterrupt:
        print("\nShutting down MadMax Console...")
    except Exception as e:
        print(f"\nUnexpected error: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
