with open('tests/test_kinematics_logic.py', 'r') as f:
    lines = f.readlines()

with open('tests/test_kinematics_logic.py', 'w') as f:
    for i, line in enumerate(lines):
        if i == 74 or i == 75:  # lines are 0-indexed, so 74 is 75 in file
            continue
        f.write(line)
        if i == 73:
            f.write("        ]\n")
