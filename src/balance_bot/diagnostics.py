import subprocess
import importlib

def check_i2c_tools():
    print("Checking I2C Devices (i2cdetect)...")
    try:
        result = subprocess.run(["i2cdetect", "-y", "1"], capture_output=True, text=True)
        print(result.stdout)

        if "22" in result.stdout:
             print("SUCCESS: PiconZero (0x22) detected.")
        else:
             print("FAILURE: PiconZero (0x22) NOT detected.")

        if "68" in result.stdout:
             print("SUCCESS: MPU6050 (0x68) detected.")
        else:
             print("FAILURE: MPU6050 (0x68) NOT detected.")

    except FileNotFoundError:
        print("WARNING: 'i2cdetect' command not found. Please install i2c-tools.")
    except Exception as e:
        print(f"ERROR running i2cdetect: {e}")

def check_smbus():
    print("\nChecking SMBus Access...")
    try:
        import smbus2
        try:
            # Try to open the bus
            bus = smbus2.SMBus(1)
            bus.close()
            print("SUCCESS: SMBus(1) opened successfully.")
        except Exception as e:
            print(f"FAILURE: Could not open SMBus(1): {e}")
            print("Hint: Check permissions (e.g. 'sudo usermod -aG i2c $USER') and reboot.")
            print("Hint: Ensure I2C is enabled in raspi-config.")
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
    check_smbus()
    check_i2c_tools()
    print("\nDiagnostics Complete.")
    print("========================")
