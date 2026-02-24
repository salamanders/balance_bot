import math
import pytest
import glm
from balance_bot.utils import sort_resting_vectors, vector_angle

def test_vector_angle_basic():
    v1 = glm.vec3(1.0, 0.0, 0.0)
    v2 = glm.vec3(0.0, 1.0, 0.0)
    assert math.isclose(vector_angle(v1, v2), 90.0, abs_tol=0.1)

    v3 = glm.vec3(1.0, 0.0, 0.0)
    assert math.isclose(vector_angle(v1, v3), 0.0, abs_tol=0.1)

    v4 = glm.vec3(-1.0, 0.0, 0.0)
    assert math.isclose(vector_angle(v1, v4), 180.0, abs_tol=0.1)

def test_sort_resting_vectors_two_groups():
    # Group A: Around (1, 0, 0)
    # Group B: Around (0, 1, 0)

    # Pivot will be (1, 0, 0)
    v1 = glm.vec3(1.0, 0.0, 0.0)
    v2 = glm.vec3(0.98, 0.02, 0.0) # Very close to v1 (angle < 20)

    v3 = glm.vec3(0.0, 1.0, 0.0) # 90 deg away
    v4 = glm.vec3(0.02, 0.98, 0.0) # Very close to v3

    vectors = [v1, v2, v3, v4]

    avg_a, avg_b = sort_resting_vectors(vectors)

    # Check avg_a (should be around v1)
    assert avg_a.x > 0.9
    assert avg_a.y < 0.1

    # Check avg_b (should be around v3)
    assert avg_b.x < 0.1
    assert avg_b.y > 0.9

    # Check return type
    assert isinstance(avg_a, glm.vec3)
    assert isinstance(avg_b, glm.vec3)

def test_sort_resting_vectors_single_cluster_raises():
    # Only vectors around (1, 0, 0)
    v1 = glm.vec3(1.0, 0.0, 0.0)
    v2 = glm.vec3(0.99, 0.01, 0.0)
    v3 = glm.vec3(1.0, 0.05, 0.0)

    vectors = [v1, v2, v3]

    with pytest.raises(ValueError) as excinfo:
        sort_resting_vectors(vectors)

    assert "clustered near pivot" in str(excinfo.value)

def test_sort_resting_vectors_empty_raises():
    with pytest.raises(ValueError) as excinfo:
        sort_resting_vectors([])

    assert "No vectors collected" in str(excinfo.value)
