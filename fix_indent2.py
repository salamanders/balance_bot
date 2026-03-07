with open('tests/test_kinematics_logic.py', 'r') as f:
    lines = f.readlines()

new_lines = []
skip = False
for i, line in enumerate(lines):
    if "hw.execute_maneuver.side_effect =" in line:
        new_lines.append(line)
        new_lines.append("            make_result(l_gyro, l_accel_delta),\n")
        new_lines.append("            make_result(r_gyro, r_accel_delta)\n")
        new_lines.append("        ]\n")
        skip = True
        continue

    if skip:
        if "hw.drive_and_measure.side_effect =" in line:
            pass # Keep skipping until we close
        if line.strip() == "]" and i > 75:
            skip = False
        continue

    new_lines.append(line)

with open('tests/test_kinematics_logic.py', 'w') as f:
    f.writelines(new_lines)
