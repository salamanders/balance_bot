import json
import logging
from pathlib import Path
from unittest.mock import MagicMock, patch
import pytest

from balance_bot.discovery.types import Atom, ExperimentResult
from balance_bot.discovery.knowledge_graph import DiscoveryContext
from balance_bot.discovery.baby_brain import DiscoveryBrain
from balance_bot.discovery.experiments import Experiment, ExpPulse
from balance_bot.utils import Vector3
from balance_bot.enums import Axis

# --- DiscoveryContext Tests ---

def test_context_serialization(tmp_path):
    f = tmp_path / "test_state.json"
    ctx = DiscoveryContext(f)

    # Add some atoms
    ctx.update({
        Atom.HARDWARE_BUS: {"motor": 1, "imu": 1},
        Atom.GRAVITY_VECTOR: Vector3(0.1, 0.2, 9.8),
        Atom.PITCH_AXIS: {"axis": "x", "invert": False}
    })

    # Reload
    ctx2 = DiscoveryContext(f)
    assert ctx2.has_atom(Atom.HARDWARE_BUS)
    assert ctx2.get(Atom.HARDWARE_BUS) == {"motor": 1, "imu": 1}

    grav = ctx2.get(Atom.GRAVITY_VECTOR)
    assert isinstance(grav, Vector3)
    assert grav.z == 9.8

    pitch = ctx2.get(Atom.PITCH_AXIS)
    assert pitch["axis"] == "x"

def test_context_build_config(tmp_path):
    f = tmp_path / "test_state.json"
    ctx = DiscoveryContext(f)

    # Minimal for config
    ctx.update({
        Atom.HARDWARE_BUS: {"motor": 1, "imu": 1},
        Atom.GRAVITY_VECTOR: Vector3(0.0, 0.0, -9.8), # Down is -Z
        Atom.PITCH_AXIS: {"axis": "x", "invert": False},
        Atom.FRICTION_THRESHOLD: 20,
        Atom.CHASSIS_HANDEDNESS: {"left": 0, "right": 1},
        Atom.MOTOR_PHASING: {"invert_right": True},
        Atom.MOTOR_POLARITY: {"invert_both": False}
    })

    c = ctx.build_config()
    assert c.motor_i2c_bus == 1
    assert c.accel_vertical_axis == Axis.Z
    assert c.accel_vertical_invert == True # Because -9.8 < 0
    assert c.gyro_pitch_axis == Axis.X
    assert c.motor_r_invert == True # From Phasing

# --- Brain Tests ---

class MockExp(Experiment):
    def __init__(self, name, reqs, outs):
        self.name = name
        self._reqs = reqs
        self._outs = outs
        self.run_called = False

    @property
    def required_atoms(self): return self._reqs
    @property
    def output_atoms(self): return self._outs

    def run(self, ctx, hw):
        self.run_called = True
        return ExperimentResult(success=True, data={a: True for a in self._outs})

def test_brain_loop(tmp_path):
    f = tmp_path / "brain_state.json"
    ctx = DiscoveryContext(f)
    brain = DiscoveryBrain(ctx)

    # Setup Experiments
    exp1 = MockExp("Exp1", [], [Atom.HARDWARE_BUS])
    exp2 = MockExp("Exp2", [Atom.HARDWARE_BUS], [Atom.GRAVITY_VECTOR])

    brain.experiments = [exp1, exp2]

    # Override think to run once per loop? No, think is infinite.
    # We can invoke logic manually or patch.

    # Loop 1: No atoms. exp1 runs.
    # We simulate the steps of think()

    # 1. Update Hardware (No bus yet)
    brain._update_hardware()
    assert brain.hw is None

    # 2. Select Candidates
    candidates = [e for e in brain.experiments if e.can_run(ctx) and not e.has_result(ctx)]
    assert len(candidates) == 1
    assert candidates[0] == exp1

    # 3. Run
    res = candidates[0].run(ctx, brain.hw)
    ctx.update(res.data)

    # Loop 2: HARDWARE_BUS known. exp2 runs.
    brain._update_hardware()
    # Mock build_config for _update_hardware (it might fail if missing fields)
    # But here we just put True in Atom.HARDWARE_BUS so it will be weird.
    # Let's verify candidates only.

    candidates = [e for e in brain.experiments if e.can_run(ctx) and not e.has_result(ctx)]
    assert len(candidates) == 1
    assert candidates[0] == exp2

    # 4. Run exp2
    res = candidates[0].run(ctx, brain.hw)
    ctx.update(res.data)

    # Loop 3: All done.
    candidates = [e for e in brain.experiments if e.can_run(ctx) and not e.has_result(ctx)]
    assert len(candidates) == 0

# --- Experiment Tests ---

def test_exp_pulse_mock():
    # Mock smbus
    with patch("balance_bot.discovery.experiments.smbus") as mock_smbus:
        ctx = MagicMock()
        hw = None
        exp = ExpPulse()

        # Configure mock bus to find 0x22 (Motor) and 0x68 (IMU)
        mock_bus_inst = MagicMock()
        mock_smbus.SMBus.return_value = mock_bus_inst

        # When check_motor calls read_byte_data(0x22, 0) -> Success
        # When check_imu calls read_byte_data(0x68, 0x75) -> 0x68

        def side_effect(addr, reg):
            if addr == 0x22: return 0
            if addr == 0x68 and reg == 0x75: return 0x68
            raise OSError

        mock_bus_inst.read_byte_data.side_effect = side_effect

        res = exp.run(ctx, hw)
        assert res.success
        assert res.data[Atom.HARDWARE_BUS]["motor"] is not None
        assert res.data[Atom.HARDWARE_BUS]["imu"] is not None
