import timeit
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from balance_bot.utils import cross_product, Vector3

# Legacy Dictionary Implementation (for baseline comparison)
def cross_product_dict(a: dict, b: dict) -> dict:
    return {
        "x": a["y"] * b["z"] - a["z"] * b["y"],
        "y": a["z"] * b["x"] - a["x"] * b["z"],
        "z": a["x"] * b["y"] - a["y"] * b["x"],
    }

def benchmark():
    print("Benchmarking Cross Product Implementations...")
    number = 100000

    # 1. Legacy Dictionary
    setup_dict = """
from __main__ import cross_product_dict
a = {'x': 1.0, 'y': 2.0, 'z': 3.0}
b = {'x': 4.0, 'y': 5.0, 'z': 6.0}
"""
    t_dict = timeit.timeit("cross_product_dict(a, b)", setup=setup_dict, number=number)
    print(f"Legacy Dict: {t_dict:.4f}s")

    # 2. New Vector3 (Slots)
    setup_new = """
from balance_bot.utils import cross_product, Vector3
a = Vector3(1.0, 2.0, 3.0)
b = Vector3(4.0, 5.0, 6.0)
"""
    t_new = timeit.timeit("cross_product(a, b)", setup=setup_new, number=number)
    print(f"New Vector3: {t_new:.4f}s (Speedup: {t_dict/t_new:.2f}x)")

if __name__ == "__main__":
    benchmark()
