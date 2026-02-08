from balance_bot.utils import cross_product

def test_cross_product_basic():
    # X x Y = Z
    a = {'x': 1, 'y': 0, 'z': 0}
    b = {'x': 0, 'y': 1, 'z': 0}
    c = cross_product(a, b)
    assert c == {'x': 0, 'y': 0, 'z': 1}

    # Y x X = -Z
    c = cross_product(b, a)
    assert c == {'x': 0, 'y': 0, 'z': -1}

    # Z x X = Y
    d = {'x': 0, 'y': 0, 'z': 1}
    c = cross_product(d, a)
    assert c == {'x': 0, 'y': 1, 'z': 0}

    # Y x Z = X
    c = cross_product(b, d)
    assert c == {'x': 1, 'y': 0, 'z': 0}

def test_cross_product_arbitrary():
    # A = [1, 2, 3]
    # B = [4, 5, 6]
    # A x B = [-3, 6, -3]
    a = {'x': 1, 'y': 2, 'z': 3}
    b = {'x': 4, 'y': 5, 'z': 6}
    c = cross_product(a, b)
    assert c == {'x': -3, 'y': 6, 'z': -3}

def test_cross_product_parallel():
    # A x A = 0
    a = {'x': 1, 'y': 2, 'z': 3}
    c = cross_product(a, a)
    assert c == {'x': 0, 'y': 0, 'z': 0}
