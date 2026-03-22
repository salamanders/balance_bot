with open('tests/test_piconzero_internals.py', 'r') as f:
    content = f.read()

# Wait! The mock captures all sleeps, including 0.005. So 0.005 is correct.
# But wait, pz uses time.sleep(0.01) !
# Let's see what pz uses exactly
