import pytest
from balance_bot.discovery.steps import DeriveKinematicsStep

def test_kinematics_step_structure():
    step = DeriveKinematicsStep()
    assert step.name == "Derive Kinematics (Single Wiggle)"
