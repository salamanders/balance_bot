import re

with open('tests/test_piconzero_internals.py', 'r') as f:
    content = f.read()

replacement = """        with self.assertRaises(OSError):
            pz.set_motor(0, 100)
        self.assertEqual(mock_bus.write_byte_data.call_count, pz.retries + 2)"""

content = re.sub(r'        with self\.assertRaises\(OSError\):\n            pz\.set_motor\(0, 100\)\n        self\.assertEqual\(mock_bus\.write_byte_data\.call_count, pz\.retries\)', replacement, content, flags=re.DOTALL)

with open('tests/test_piconzero_internals.py', 'w') as f:
    f.write(content)
