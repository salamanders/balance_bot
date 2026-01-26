
class MockPiconZero:
    def init(self):
        print("[MockPiconZero] init")

    def stop(self):
        print("[MockPiconZero] stop")

    def setMotor(self, motor, value):
        pass

    def cleanup(self):
        print("[MockPiconZero] cleanup")

class MockMPU6050:
    def __init__(self, address):
        self.address = address
        print(f"[MockMPU6050] init at {address}")

    def get_accel_data(self):
        # Return some wobble to test logic?
        return {'x': 0.0, 'y': 0.0, 'z': 9.8}

    def get_gyro_data(self):
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}
