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
    pass 
def apply_action(state, action):
    """
    Applies a STRIPS action and returns a NEW state.
    """
    pass

def goal_satisfied(state, goal):
    """
    Checks if all goal fluents are true in the state.
    """
    pass
    

# Planner (BFS)


def forward_search(initial_state, goal_state, actions):
    """
    Breadth-First Search planner.

    Returns:
        plan (list of action names)
        states_explored (int)
    """
    pass
    

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
