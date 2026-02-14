from balance_bot.utils import cross_product, Vector3

def test_cross_product_basic():
    # X x Y = Z
    a = Vector3(1, 0, 0)
    b = Vector3(0, 1, 0)
    c = cross_product(a, b)
    assert c.x == 0 and c.y == 0 and c.z == 1

    # Y x X = -Z
    c = cross_product(b, a)
    assert c.x == 0 and c.y == 0 and c.z == -1

    # Z x X = Y
    d = Vector3(0, 0, 1)
    c = cross_product(d, a)
    assert c.x == 0 and c.y == 1 and c.z == 0

    # Y x Z = X
    c = cross_product(b, d)
    assert c.x == 1 and c.y == 0 and c.z == 0

def test_cross_product_arbitrary():
    # A = [1, 2, 3]
    # B = [4, 5, 6]
    # A x B = [-3, 6, -3]
    a = Vector3(1, 2, 3)
    b = Vector3(4, 5, 6)
    c = cross_product(a, b)
    assert c.x == -3 and c.y == 6 and c.z == -3

def test_cross_product_parallel():
    # A x A = 0
    a = Vector3(1, 2, 3)
    c = cross_product(a, a)
    assert c.x == 0 and c.y == 0 and c.z == 0
