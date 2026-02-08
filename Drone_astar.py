"""
Drone Delivery Planner
STRIPS-style planning using A*.
"""

import heapq
from collections import deque

def create_initial_state():
    """
    Returns the initial state as a frozenset.
    """
    return frozenset([
        'at_base',
        'battery_10',
        'carrying_none',
        'package_c1_at_pickup',
        'package_c2_at_pickup',
        'package_c3_at_pickup',
        'package_c4_at_pickup',
        'package_c5_at_pickup'
    ])

def create_goal_state():
    """
    Returns the goal state as a frozenset.
    """
    return frozenset([
        'at_base',
        'delivered_c1',
        'delivered_c2',
        'delivered_c3',
        'delivered_c4',
        'delivered_c5'
    ])



# Planner (A*)

def forward_search(initial_state, goal_state, actions):
    """
    A* Search planner.

    Returns:
        plan (list of action names)
        states_explored (int)
    """
    customer_costs = {
        'c1': 2,
        'c2': 2,
        'c3': 3,
        'c4': 4,
        'c5': 3,
    }
    base_pickup_cost = 2
    def battery_level(state):
        for item in state:
            if item.startswith('battery_'):
                return int(item.split('_')[1])
        return 0

    def heuristic(state):

        # Lower-bound estimate of remaining battery cost.
        remaining_cost = 0
        for customer, cost in customer_costs.items():
            if f'delivered_{customer}' not in state:
                remaining_cost += cost
                # Add minimum cost to return to base.
        if 'at_base' in goal_state and 'at_base' not in state:
            if 'at_pickup' in state:
                remaining_cost += base_pickup_cost
            else:
                for customer, cost in customer_costs.items():
                    if f'at_{customer}' in state:
                        remaining_cost += cost
                        break
        return remaining_cost

    pq = []#priority queue of states to explore
    heapq.heappush(pq, (heuristic(initial_state), 0, initial_state, []))
    best_g = {initial_state: 0}#best cost found so far for each state
    explored = set()

    while pq:
        f_cost, g_cost, state, plan = heapq.heappop(pq)
        if state in explored:#prevent loops
            continue
        if goal_satisfied(state, goal_state):
            return plan, len(explored)
        explored.add(state)
        for action in actions:
            if is_applicable(state, action):
                new_state = apply_action(state, action)
                new_g = g_cost + action.get('cost', 0)
                if new_state not in best_g or new_g < best_g[new_state]:
                    best_g[new_state] = new_g
                    f_cost = new_g + heuristic(new_state)
                    heapq.heappush(
                        pq, (f_cost, new_g, new_state, plan + [action['name']])
                    )
    return None, len(explored)

# Domain Definition

def create_actions():
    """
    Define ALL ground actions.
    Returns a list of action dictionaries.
    """
    
    actions = []
    customers = ['c1', 'c2', 'c3', 'c4', 'c5']
    customer_costs = {
        'c1': 2,
        'c2': 2,
        'c3': 3,
        'c4': 4,
        'c5': 3,
    }

    # Movement actions (each trip costs 2 battery units)
    for level in range(2, 11):
        actions.append({
            'name': f'fly_base_to_pickup_b{level}',
            'preconditions': ['at_base', f'battery_{level}'],
            'add': ['at_pickup', f'battery_{level - 2}'],
            'delete': ['at_base', f'battery_{level}'],
            'cost': 2
        })
        actions.append({
            'name': f'fly_pickup_to_base_b{level}',
            'preconditions': ['at_pickup', f'battery_{level}'],
            'add': ['at_base', f'battery_{level - 2}'],
            'delete': ['at_pickup', f'battery_{level}'],
            'cost': 2
        })

    # Movement actions with varied customer distances (costs 2-4 units)
    for customer in customers:
        cost = customer_costs[customer]
        for level in range(cost, 11):
            actions.append({
                'name': f'fly_pickup_to_{customer}_b{level}',
                'preconditions': ['at_pickup', f'battery_{level}'],
                'add': [f'at_{customer}', f'battery_{level - cost}'],
                'delete': ['at_pickup', f'battery_{level}'],
                'cost': cost
            })
            actions.append({
                'name': f'fly_{customer}_to_pickup_b{level}',
                'preconditions': [f'at_{customer}', f'battery_{level}'],
                'add': ['at_pickup', f'battery_{level - cost}'],
                'delete': [f'at_{customer}', f'battery_{level}'],
                'cost': cost
            })
            actions.append({
                'name': f'fly_{customer}_to_base_b{level}',
                'preconditions': [f'at_{customer}', f'battery_{level}'],
                'add': ['at_base', f'battery_{level - cost}'],
                'delete': [f'at_{customer}', f'battery_{level}'],
                'cost': cost
            })

    # Pickup and delivery (one package at a time)
    actions.append({
        'name': 'pickup_package_c1',
        'preconditions': ['at_pickup', 'carrying_none', 'package_c1_at_pickup'],
        'add': ['carrying_c1'],
        'delete': ['carrying_none', 'package_c1_at_pickup'],
        'cost': 0
    })
    actions.append({
        'name': 'pickup_package_c2',
        'preconditions': ['at_pickup', 'carrying_none', 'package_c2_at_pickup'],
        'add': ['carrying_c2'],
        'delete': ['carrying_none', 'package_c2_at_pickup'],
        'cost': 0
    })

    actions.append({
        'name': 'pickup_package_c3',
        'preconditions': ['at_pickup', 'carrying_none', 'package_c3_at_pickup'],
        'add': ['carrying_c3'],
        'delete': ['carrying_none', 'package_c3_at_pickup'],
        'cost': 0
    })
    actions.append({
        'name': 'pickup_package_c4',
        'preconditions': ['at_pickup', 'carrying_none', 'package_c4_at_pickup'],
        'add': ['carrying_c4'],
        'delete': ['carrying_none', 'package_c4_at_pickup'],
        'cost': 0
    })
    actions.append({
        'name': 'pickup_package_c5',
        'preconditions': ['at_pickup', 'carrying_none', 'package_c5_at_pickup'],
        'add': ['carrying_c5'],
        'delete': ['carrying_none', 'package_c5_at_pickup'],
        'cost': 0
    })
    actions.append({
        'name': 'deliver_package_c1',
        'preconditions': ['at_c1', 'carrying_c1'],
        'add': ['delivered_c1', 'carrying_none'],
        'delete': ['carrying_c1'],
        'cost': 0
    })
    actions.append({
        'name': 'deliver_package_c2',
        'preconditions': ['at_c2', 'carrying_c2'],
        'add': ['delivered_c2', 'carrying_none'],
        'delete': ['carrying_c2'],
        'cost': 0
    })
    actions.append({
        'name': 'deliver_package_c3',
        'preconditions': ['at_c3', 'carrying_c3'],
        'add': ['delivered_c3', 'carrying_none'],
        'delete': ['carrying_c3'],
        'cost': 0
    })
    actions.append({
        'name': 'deliver_package_c4',
        'preconditions': ['at_c4', 'carrying_c4'],
        'add': ['delivered_c4', 'carrying_none'],
        'delete': ['carrying_c4'],
        'cost': 0
    })
    actions.append({
        'name': 'deliver_package_c5',
        'preconditions': ['at_c5', 'carrying_c5'],
        'add': ['delivered_c5', 'carrying_none'],
        'delete': ['carrying_c5'],
        'cost': 0
    })

    # Recharge action (restore battery to full at base)
    for level in range(0, 10):
        actions.append({
            'name': f'recharge_at_base_b{level}',
            'preconditions': ['at_base', f'battery_{level}'],
            'add': ['battery_10'],
            'delete': [f'battery_{level}'],
            'cost': 0
        })
    return actions

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

