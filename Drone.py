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
    return all(fluent in state for fluent in goal)

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
        if goal_satisfied(state, goal_state):
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

    # Movement actions (consume battery)
    actions.append({
        'name': 'fly_base_to_pickup',
        'preconditions': ['at_base', 'battery_full'],
        'add': ['at_pickup', 'battery_low'],
        'delete': ['at_base', 'battery_full']
    })
    actions.append({
        'name': 'fly_pickup_to_customer',
        'preconditions': ['at_pickup', 'battery_full', 'has_package'],
        'add': ['at_customer', 'battery_low'],
        'delete': ['at_pickup', 'battery_full']
    })

    # Pickup and delivery
    actions.append({
        'name': 'pickup_package',
        'preconditions': ['at_pickup'],
        'add': ['has_package'],
        'delete': []
    })
    actions.append({
        'name': 'deliver_package',
        'preconditions': ['at_customer', 'has_package'],
        'add': ['delivered'],
        'delete': ['has_package']
    })

    # Recharge action (restore battery)
    actions.append({
        'name': 'recharge_at_pickup',
        'preconditions': ['at_pickup', 'battery_low'],
        'add': ['battery_full'],
        'delete': ['battery_low']
    })
    return actions

def create_initial_state():
    """
    Returns the initial state as a frozenset.
    """
    return frozenset(['at_base', 'battery_full'])

def create_goal_state():
    """
    Returns the goal state as a frozenset.
    """
    return frozenset(['delivered'])

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
