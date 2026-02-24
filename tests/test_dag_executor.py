import pytest
from unittest.mock import MagicMock
from balance_bot.dag_executor import DAGExecutor

def test_dag_success_linear():
    """Test A -> B -> C execution order."""
    state = {"A": False, "B": False, "C": False}
    log = []

    def make_action(name):
        def action():
            log.append(f"Run {name}")
            state[name] = True
        return action

    graph = {
        "A": {
            "condition": lambda: state["A"],
            "depends_on": [],
            "action": make_action("A")
        },
        "B": {
            "condition": lambda: state["B"],
            "depends_on": ["A"],
            "action": make_action("B")
        },
        "C": {
            "condition": lambda: state["C"],
            "depends_on": ["B"],
            "action": make_action("C")
        }
    }

    executor = DAGExecutor(graph)
    # Check sort order
    assert executor.sorted_nodes == ["A", "B", "C"]

    executor.run()

    assert state["A"] and state["B"] and state["C"]
    assert log == ["Run A", "Run B", "Run C"]

def test_dag_cycle_detection():
    """Test A -> B -> A throws ValueError."""
    graph = {
        "A": {
            "condition": lambda: True,
            "depends_on": ["B"],
            "action": lambda: None
        },
        "B": {
            "condition": lambda: True,
            "depends_on": ["A"],
            "action": lambda: None
        }
    }

    with pytest.raises(ValueError, match="Dependency Cycle Detected"):
        DAGExecutor(graph)

def test_dag_missing_dependency():
    """Test A -> ? throws ValueError."""
    graph = {
        "A": {
            "condition": lambda: True,
            "depends_on": ["UNKNOWN"],
            "action": lambda: None
        }
    }

    with pytest.raises(ValueError, match="depends on unknown node"):
        DAGExecutor(graph)

def test_dag_cascading_invalidation():
    """
    Test that if a dependency becomes False, the executor goes back to fix it.
    Scenario:
    A -> B.
    1. A is True. B is False. -> Run B.
    2. B action triggers A to become False (simulating e.g. a reset).
    3. Next check: A is False. -> Run A.
    4. Then Run B again (since B condition might still be unsatisfied or became unsatisfied).
    """
    state = {"A": True, "B": False}
    log = []

    def action_A():
        log.append("Run A")
        state["A"] = True

    def action_B():
        log.append("Run B")
        state["B"] = True
        # Simulate side effect: A breaks
        if len(log) == 1: # Only break A the first time B runs
            log.append("Break A")
            state["A"] = False
            state["B"] = False # B also fails because it needs A, or just fails logic

    graph = {
        "A": {
            "condition": lambda: state["A"],
            "depends_on": [],
            "action": action_A
        },
        "B": {
            "condition": lambda: state["B"],
            "depends_on": ["A"],
            "action": action_B
        }
    }

    executor = DAGExecutor(graph)
    executor.run()

    # Expected sequence:
    # 1. Check A (True). Check B (False). Run B.
    # 2. Action B breaks A.
    # 3. Loop restarts. Check A (False). Run A.
    # 4. Loop restarts. Check A (True). Check B (False). Run B.
    # 5. Action B succeeds.
    # 6. Loop restarts. Check A (True). Check B (True). Success.

    assert log == ["Run B", "Break A", "Run A", "Run B"]
    assert state["A"] is True
    assert state["B"] is True

def test_dag_max_iterations():
    """Test that infinite loops are caught."""
    graph = {
        "A": {
            "condition": lambda: False, # Always false
            "depends_on": [],
            "action": lambda: None # Does nothing
        }
    }

    executor = DAGExecutor(graph)

    with pytest.raises(RuntimeError, match="exceeded 10 iterations"):
        executor.run(max_iterations=10)

def test_dag_independent_nodes():
    """Test A, B independent."""
    state = {"A": False, "B": False}
    log = []

    graph = {
        "A": {
            "condition": lambda: state["A"],
            "depends_on": [],
            "action": lambda: (log.append("A"), state.update({"A": True}))
        },
        "B": {
            "condition": lambda: state["B"],
            "depends_on": [],
            "action": lambda: (log.append("B"), state.update({"B": True}))
        }
    }

    executor = DAGExecutor(graph)
    # Sort order is deterministic because we sorted the queue in Kahn's algo
    # A comes before B alphabetically
    assert executor.sorted_nodes == ["A", "B"]

    executor.run()
    assert log == ["A", "B"]
