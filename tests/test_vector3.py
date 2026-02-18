import pytest
import math
from balance_bot.utils import Vector3, cross_product, analyze_dominance

def test_initialization():
    v = Vector3(1.0, 2.0, 3.0)
    assert v.x == 1.0
    assert v.y == 2.0
    assert v.z == 3.0

def test_from_dict():
    d = {'x': 10.0, 'y': 20.0, 'z': 30.0}
    v = Vector3.from_dict(d)
    assert v.x == 10.0
    assert v.y == 20.0
    assert v.z == 30.0

def test_dict_access():
    v = Vector3(1, 2, 3)
    assert v['x'] == 1
    assert v['y'] == 2
    assert v['z'] == 3
    assert v[0] == 1
    assert v[1] == 2
    assert v[2] == 3

    with pytest.raises(KeyError):
        _ = v['w']

def test_arithmetic_add():
    v1 = Vector3(1, 2, 3)
    v2 = Vector3(4, 5, 6)
    v3 = v1 + v2
    assert v3 == Vector3(5, 7, 9)
    # Ensure immutability
    assert v1 == Vector3(1, 2, 3)

def test_arithmetic_sub():
    v1 = Vector3(5, 7, 9)
    v2 = Vector3(1, 2, 3)
    v3 = v1 - v2
    assert v3 == Vector3(4, 5, 6)

def test_arithmetic_mul():
    v1 = Vector3(1, 2, 3)
    v2 = v1 * 2.0
    assert v2 == Vector3(2, 4, 6)

    v3 = 2.0 * v1
    assert v3 == Vector3(2, 4, 6)

def test_arithmetic_div():
    v1 = Vector3(2, 4, 6)
    v2 = v1 / 2.0
    assert v2 == Vector3(1, 2, 3)

    with pytest.raises(ZeroDivisionError):
        _ = v1 / 0.0

def test_negation():
    v = Vector3(1, -2, 3)
    v_neg = -v
    assert v_neg == Vector3(-1, 2, -3)

def test_magnitude():
    v = Vector3(3, 4, 0)
    assert v.magnitude == 5.0

    v = Vector3(1, 1, 1)
    assert math.isclose(v.magnitude, math.sqrt(3))

def test_dot_product():
    v1 = Vector3(1, 0, 0)
    v2 = Vector3(0, 1, 0)
    assert v1.dot(v2) == 0.0 # Orthogonal

    v3 = Vector3(1, 2, 3)
    v4 = Vector3(4, 5, 6)
    # 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    assert v3.dot(v4) == 32.0

def test_cross_product_method():
    v1 = Vector3(1, 0, 0)
    v2 = Vector3(0, 1, 0)
    v3 = v1.cross(v2)
    assert v3 == Vector3(0, 0, 1) # Right hand rule

    v4 = v2.cross(v1)
    assert v4 == Vector3(0, 0, -1)

def test_cross_product_function():
    v1 = Vector3(1, 0, 0)
    v2 = Vector3(0, 1, 0)
    v3 = cross_product(v1, v2)
    assert v3 == Vector3(0, 0, 1)

def test_analyze_dominance_with_vector():
    v = Vector3(1.0, 10.0, 0.5)
    winner, ratio, success = analyze_dominance(v, "Test")
    assert winner == 'y'
    assert success is True
    # 10.0 / (1.0 + 1e-9) approx 10.0
    assert ratio > 9.9
