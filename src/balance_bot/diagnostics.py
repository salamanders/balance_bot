import subprocess
import importlib
from pathlib import Path

def check_system_i2c_config():
    print("\nChecking System I2C Config...")
    config_paths = [Path("/boot/firmware/config.txt"), Path("/boot/config.txt")]
    found_overlay = False

    for path in config_paths:
        if path.exists():
            try:
                content = path.read_text()
                active_overlay = False
                for line in content.splitlines():
                    if line.strip().startswith("dtoverlay=i2c-gpio"):
                        active_overlay = True
                        break

                if active_overlay:
                    print(f"SUCCESS: Software I2C overlay found in {path}")
                    found_overlay = True
                else:
                    print(f"INFO: No Software I2C overlay in {path}")
            except Exception as e:
                print(f"WARNING: Could not read {path}: {e}")

    if not found_overlay:
        print("INFO: Software I2C (Bus 3) not configured. This is OK if using standard hardware.")
        print("      If PiconZero blocks the I2C pins, you may need to enable Software I2C.")

def check_i2c_tools():
    print("\nChecking I2C Devices (i2cdetect)...")

    for bus_id in [1, 3]:
        print(f"--- Scanning Bus {bus_id} ---")
        try:
            result = subprocess.run(["i2cdetect", "-y", str(bus_id)], capture_output=True, text=True)
            print(result.stdout)

            # Bus 1 usually has PiconZero (0x22)
            if bus_id == 1:
                if "22" in result.stdout:
                    print("SUCCESS: PiconZero (0x22) detected on Bus 1.")
                else:
                    print("FAILURE: PiconZero (0x22) NOT detected on Bus 1.")

            # MPU6050 (0x68) can be on either
            if "68" in result.stdout:
                print(f"SUCCESS: MPU6050 (0x68) detected on Bus {bus_id}.")
            else:
                print(f"INFO: MPU6050 (0x68) NOT detected on Bus {bus_id}.")

        except FileNotFoundError:
            print("WARNING: 'i2cdetect' command not found. Please install i2c-tools.")
            break
        except Exception as e:
            print(f"ERROR running i2cdetect on Bus {bus_id}: {e}")

def check_smbus():
    print("\nChecking SMBus Access...")
    try:
        import smbus2
        for bus_id in [1, 3]:
            try:
                # Try to open the bus
                bus = smbus2.SMBus(bus_id)
                bus.close()
                print(f"SUCCESS: SMBus({bus_id}) opened successfully.")
            except Exception as e:
                print(f"INFO: Could not open SMBus({bus_id}): {e}")
                if bus_id == 1:
                     print("Hint: Check permissions (e.g. 'sudo usermod -aG i2c $USER') and reboot.")
    except ImportError:
         print("FAILURE: 'smbus2' package not installed.")

def check_imports():
    print("\nChecking Python Imports...")

    modules = ["smbus2", "mpu6050"]

    for mod in modules:
        try:
            importlib.import_module(mod)
            print(f"SUCCESS: Import '{mod}' working.")
        except ImportError as e:
             print(f"FAILURE: Could not import '{mod}': {e}")

    # Check internal piconzero
    try:
        from .piconzero import PiconZero  # noqa: F401
        print("SUCCESS: Internal 'piconzero' module import working.")
    except ImportError as e:
         print(f"FAILURE: Could not import internal 'piconzero': {e}")

def run_diagnostics():
    print("=== Hardware Diagnostics ===")
    check_imports()
    check_system_i2c_config()
    check_smbus()
    check_i2c_tools()
    print("\nDiagnostics Complete.")
    print("========================")
