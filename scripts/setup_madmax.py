#!/usr/bin/env python3
"""
MADMAX AI Agentic Aerial Robotics - Setup Script
Automated setup and initialization script for the MADMAX system.
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

def print_banner():
    """Print the MADMAX banner"""
    banner = """
    ╔══════════════════════════════════════════════════════════════╗
    ║                                                              ║
    ║    🚁 MADMAX AI AGENTIC AERIAL ROBOTICS SETUP 🚁            ║
    ║                                                              ║
    ║    NVIDIA Jetson Orin Nano + Pixhawk 6C                     ║
    ║    Natural Language → AI Agents → MAVLink Commands          ║
    ║                                                              ║
    ╚══════════════════════════════════════════════════════════════╝
    """
    print(banner)

def check_python_version():
    """Check if Python version is compatible"""
    if sys.version_info < (3, 8):
        print("❌ Error: Python 3.8 or higher is required")
        print(f"   Current version: {sys.version}")
        return False
    print(f"✅ Python version: {sys.version.split()[0]}")
    return True

def install_system_dependencies():
    """Install system-level dependencies"""
    print("\n📦 Installing system dependencies...")
    
    # Check if we're on a Debian/Ubuntu system
    if shutil.which('apt'):
        dependencies = [
            'python3-pip',
            'python3-dev',
            'build-essential',
            'cmake',
            'git',
            'curl',
            'wget'
        ]
        
        try:
            subprocess.run(['sudo', 'apt', 'update'], check=True)
            subprocess.run(['sudo', 'apt', 'install', '-y'] + dependencies, check=True)
            print("✅ System dependencies installed")
        except subprocess.CalledProcessError:
            print("⚠️  Warning: Could not install system dependencies automatically")
            print("   Please install manually: python3-pip python3-dev build-essential cmake")
    else:
        print("⚠️  Warning: Non-Debian system detected. Please install dependencies manually.")

def create_virtual_environment():
    """Create and activate virtual environment"""
    print("\n🐍 Setting up Python virtual environment...")
    
    venv_path = Path("venv")
    if venv_path.exists():
        print("✅ Virtual environment already exists")
        return True
    
    try:
        subprocess.run([sys.executable, '-m', 'venv', 'venv'], check=True)
        print("✅ Virtual environment created")
        return True
    except subprocess.CalledProcessError:
        print("❌ Failed to create virtual environment")
        return False

def install_python_dependencies():
    """Install Python dependencies"""
    print("\n📚 Installing Python dependencies...")
    
    # Use venv pip if available
    pip_cmd = 'venv/bin/pip' if Path('venv/bin/pip').exists() else 'pip3'
    
    try:
        # Upgrade pip first
        subprocess.run([pip_cmd, 'install', '--upgrade', 'pip'], check=True)
        
        # Install requirements
        subprocess.run([pip_cmd, 'install', '-r', 'requirements.txt'], check=True)
        
        # Install spaCy model
        subprocess.run([pip_cmd, 'install', 'https://github.com/explosion/spacy-models/releases/download/en_core_web_sm-3.6.0/en_core_web_sm-3.6.0-py3-none-any.whl'], check=True)
        
        print("✅ Python dependencies installed")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install Python dependencies: {e}")
        return False

def setup_configuration():
    """Setup configuration files"""
    print("\n⚙️  Setting up configuration...")
    
    # Create .env file if it doesn't exist
    if not Path('.env').exists():
        if Path('.env.example').exists():
            shutil.copy('.env.example', '.env')
            print("✅ Created .env configuration file")
            print("   Please edit .env file to configure your system")
        else:
            print("⚠️  Warning: .env.example not found")
    else:
        print("✅ Configuration file already exists")
    
    # Create logs directory
    logs_dir = Path('logs')
    logs_dir.mkdir(exist_ok=True)
    print("✅ Created logs directory")
    
    # Create ai_agents __init__.py
    ai_agents_dir = Path('ai_agents')
    init_file = ai_agents_dir / '__init__.py'
    if not init_file.exists():
        init_file.write_text('# MADMAX AI Agents Module\n')
        print("✅ Created ai_agents module")

def check_hardware_connections():
    """Check for hardware connections"""
    print("\n🔌 Checking hardware connections...")
    
    # Check for USB serial devices (Pixhawk)
    usb_devices = []
    for device in ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
        if Path(device).exists():
            usb_devices.append(device)
    
    if usb_devices:
        print(f"✅ Found USB serial devices: {', '.join(usb_devices)}")
        print("   These may be your Pixhawk 6C connection")
    else:
        print("⚠️  No USB serial devices found")
        print("   Make sure your Pixhawk 6C is connected via USB")
    
    # Check if we're on Jetson
    jetson_model = None
    if Path('/proc/device-tree/model').exists():
        try:
            with open('/proc/device-tree/model', 'r') as f:
                model = f.read().strip()
                if 'jetson' in model.lower():
                    jetson_model = model
        except:
            pass
    
    if jetson_model:
        print(f"✅ Running on NVIDIA Jetson: {jetson_model}")
    else:
        print("ℹ️  Not running on NVIDIA Jetson (development mode)")

def create_startup_scripts():
    """Create startup scripts"""
    print("\n📝 Creating startup scripts...")
    
    # Create start script
    start_script = Path('start_madmax.sh')
    start_script.write_text("""#!/bin/bash
# MADMAX AI Agentic Aerial Robotics Startup Script

echo "🚁 Starting MADMAX AI Agentic System..."

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
    echo "✅ Virtual environment activated"
fi

# Load environment variables
if [ -f ".env" ]; then
    export $(cat .env | grep -v '^#' | xargs)
    echo "✅ Environment variables loaded"
fi

# Start the system
echo "🚀 Launching MADMAX Agentic Web Server..."
python3 madmax_agentic_webserver.py
""")
    start_script.chmod(0o755)
    print("✅ Created start_madmax.sh")
    
    # Create systemd service file
    service_content = f"""[Unit]
Description=MADMAX AI Agentic Aerial Robotics
After=network.target

[Service]
Type=simple
User={os.getenv('USER', 'root')}
WorkingDirectory={Path.cwd()}
ExecStart={Path.cwd()}/start_madmax.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
"""
    
    service_file = Path('madmax.service')
    service_file.write_text(service_content)
    print("✅ Created madmax.service (systemd service file)")
    print("   To install as system service: sudo cp madmax.service /etc/systemd/system/")

def run_tests():
    """Run basic system tests"""
    print("\n🧪 Running system tests...")
    
    try:
        # Test imports
        sys.path.append('ai_agents')
        
        print("   Testing NLP Intent Extractor...")
        from ai_agents.nlp_intent_extractor import NLPIntentExtractor
        extractor = NLPIntentExtractor()
        result = extractor.extract_intent("take off to 10 meters")
        print(f"   ✅ NLP test passed: {result.intent.value}")
        
        print("   Testing Flight Agent...")
        from ai_agents.flight_agent import FlightAgent
        agent = FlightAgent()
        print("   ✅ Flight Agent initialized")
        
        print("   Testing MAVLink Interface...")
        from ai_agents.mavlink_interface import MAVLinkInterface
        interface = MAVLinkInterface("test://")
        print("   ✅ MAVLink Interface initialized")
        
        print("✅ All system tests passed!")
        return True
        
    except Exception as e:
        print(f"❌ System test failed: {e}")
        return False

def main():
    """Main setup function"""
    print_banner()
    
    # Check prerequisites
    if not check_python_version():
        sys.exit(1)
    
    # Setup steps
    steps = [
        ("Installing system dependencies", install_system_dependencies),
        ("Creating virtual environment", create_virtual_environment),
        ("Installing Python dependencies", install_python_dependencies),
        ("Setting up configuration", setup_configuration),
        ("Checking hardware connections", check_hardware_connections),
        ("Creating startup scripts", create_startup_scripts),
        ("Running system tests", run_tests)
    ]
    
    failed_steps = []
    
    for step_name, step_func in steps:
        try:
            if not step_func():
                failed_steps.append(step_name)
        except Exception as e:
            print(f"❌ Error in {step_name}: {e}")
            failed_steps.append(step_name)
    
    # Summary
    print("\n" + "="*60)
    print("🎯 SETUP SUMMARY")
    print("="*60)
    
    if failed_steps:
        print("❌ Setup completed with errors:")
        for step in failed_steps:
            print(f"   - {step}")
        print("\nPlease resolve the errors and run setup again.")
    else:
        print("✅ Setup completed successfully!")
        print("\n🚀 NEXT STEPS:")
        print("1. Edit .env file to configure your system")
        print("2. Connect your Pixhawk 6C via USB")
        print("3. Run: ./start_madmax.sh")
        print("4. Open web interface: http://localhost:5000")
        print("\n🎉 Welcome to MADMAX AI Agentic Aerial Robotics!")

if __name__ == "__main__":
    main()
