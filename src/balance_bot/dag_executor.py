import logging
import time
from typing import Callable, Dict, List, Any, Optional

logger = logging.getLogger(__name__)

class DAGExecutor:
    """
    Executes a Directed Acyclic Graph (DAG) of tasks based on dynamic conditions.

    The executor:
    1. Validates the graph structure and ensures no cycles (static check).
    2. Sorts nodes topologically.
    3. Runs a loop where it evaluates conditions in topological order.
    4. Executes the FIRST node whose condition is False but whose dependencies are met.
       (Note: Dependencies must be met by definition if we process in topological order and stop at the first False).
    """

    def __init__(self, graph: Dict[str, Any]):
        """
        Initialize with a dependency graph.

        Graph format:
        {
            "node_id": {
                "description": "Description of the step",
                "condition": lambda: boolean_check(),
                "depends_on": ["dependency_id", ...],
                "action": lambda: do_something()
            },
            ...
        }
        """
        self.graph = graph
        self.sorted_nodes = []
        self._validate_and_sort()

    def _validate_and_sort(self):
        """
        Performs a topological sort using Kahn's Algorithm.
        Raises ValueError if a cycle is detected or dependencies are missing.
        """
        # Calculate in-degrees
        in_degree = {node: 0 for node in self.graph}
        for node, data in self.graph.items():
            for dep in data.get("depends_on", []):
                if dep not in self.graph:
                    raise ValueError(f"Node '{node}' depends on unknown node '{dep}'")
                in_degree[node] += 1

        # Let's rebuild efficiently
        adj_list = {node: [] for node in self.graph}
        in_degree = {node: 0 for node in self.graph}

        for node, data in self.graph.items():
            for dep in data.get("depends_on", []):
                if dep not in self.graph:
                     raise ValueError(f"Node '{node}' depends on unknown node '{dep}'")
                # dep -> node
                adj_list[dep].append(node)
                in_degree[node] += 1

        queue = [node for node in self.graph if in_degree[node] == 0]
        # Sort queue initially to ensure deterministic order for independent nodes?
        # Not strictly required but good for stability.
        queue.sort()

        sorted_list = []
        while queue:
            u = queue.pop(0)
            sorted_list.append(u)

            for v in adj_list[u]:
                in_degree[v] -= 1
                if in_degree[v] == 0:
                    queue.append(v)

            # Keep queue sorted? No, standard BFS is fine, or queue.sort() if we want strict lexical priority
            # For now, insertion order is fine.

        if len(sorted_list) != len(self.graph):
            raise ValueError("Dependency Cycle Detected! The graph is not a DAG.")

        self.sorted_nodes = sorted_list
        logger.info(f"DAG Topological Sort: {self.sorted_nodes}")

    def run(self, max_iterations=100) -> bool:
        """
        Execute the graph.
        Returns True if all conditions are met.
        Raises RuntimeError if max_iterations is exceeded.
        """
        print(f"Starting DAG Execution ({len(self.sorted_nodes)} nodes)...")

        for iteration in range(max_iterations):
            print(f"\n--- Iteration {iteration + 1} ---")

            # 1. Evaluate State
            # We iterate in topological order. The first node that is FALSE is our bottleneck.
            # Because it is first, all its dependencies (which appear earlier) MUST be True
            # (otherwise we would have stopped at them).

            target_node = None
            all_satisfied = True

            for node_id in self.sorted_nodes:
                data = self.graph[node_id]
                condition_met = False
                try:
                    condition_met = data["condition"]()
                except Exception as e:
                    logger.error(f"Error evaluating condition for '{node_id}': {e}")
                    # If a check fails (e.g. hardware read error), we treat it as False?
                    # Or re-raise?
                    # Requirement: "If an IMU read fails, it should raise an exception... not silently flip"
                    # But the requirement said: "IMU read fails... raise exception, not silently flip to False"
                    # So we should probably let it bubble up if it's critical.
                    raise

                if not condition_met:
                    print(f"-> [MISSING] {node_id}: {data.get('description', '')}")
                    target_node = node_id
                    all_satisfied = False
                    break # Stop at the first blocker

            if all_satisfied:
                print("-> [SUCCESS] All graph conditions met.")
                return True

            # 2. Execute Action
            if target_node:
                data = self.graph[target_node]
                print(f"-> [ACTION] Executing '{target_node}'...")
                action_fn = data.get("action")
                if action_fn:
                    try:
                        action_fn()
                    except Exception as e:
                        print(f"-> [ERROR] Action '{target_node}' failed: {e}")
                        raise
                else:
                    print(f"-> [WARNING] Node '{target_node}' has no action!")

                # We do NOT continue the loop linearly. We break and restart the loop
                # to re-evaluate the entire state (Cascading Invalidation).
                # (The loop 'for node_id' is already broken, we continue the outer 'while'/'for iteration')
                time.sleep(0.5) # Brief pause for stability
                continue

        raise RuntimeError(f"DAG Execution exceeded {max_iterations} iterations. Possible oscillation or stuck state.")
