"""
Drone Delivery Planner
--------------------------------
Implements STRIPS-style planning using BFS.
"""

from collections import deque


# STRIPS Helper Functions


def is_applicable(state, action):
    """
    Returns True if action's preconditions are satisfied in state.
    """
    return all(precond in state for precond in action['preconditions'])

def apply_action(state, action):
    """
    Applies a STRIPS action and returns a NEW state.
    """
    new_state = set(state)  # start from current state
    for item in action['add']:
        new_state.add(item)
    for item in action['delete']:
        if item in new_state:
            new_state.remove(item)
    return frozenset(new_state)

def goal_satisfied(state, goal):
    """
    Checks if all goal fluents are true in the state.
    """
    
    return all(goal in state for goal in goal)

# Planner (BFS)


def forward_search(initial_state, goal_state, actions):
    """
    Breadth-First Search planner.

    Returns:
        plan (list of action names)
        states_explored (int)
    """
    
    frontier = deque([(initial_state, [])])
    explored = set()
    while frontier:
        state, plan = frontier.popleft()
        if state == goal_state:
            return plan, len(explored)
        explored.add(state)
        for action in actions:
            if is_applicable(state, action):
                new_state = apply_action(state, action)
                if new_state not in explored:
                    frontier.append((new_state, plan + [action['name']]))
    return None, len(explored)

# Domain Definition

def create_actions():
    """
    Define ALL ground actions here.
    Returns a list of action dictionaries.
    """
    
    actions = []

    pass


def create_initial_state():
    """
    Returns the initial state as a frozenset.
    """
    pass


def create_goal_state():
    """
    Returns the goal state as a frozenset.
    """
    pass

# Main Runner

def main():

    actions = create_actions()
    initial_state = create_initial_state()
    goal_state = create_goal_state()

    plan, explored = forward_search(initial_state, goal_state, actions)

    print("\n===== RESULT =====")

    if plan:
        print("Plan found!\n")

        for step, action in enumerate(plan, 1):
            print(f"{step}. {action}")

        print("\nPlan length:", len(plan))

    else:
        print("No plan found.")

    print("States explored:", explored)


if __name__ == "__main__":
    main()
