# Drone Delivery STRIPS Planner

## Overview
This project implements a classical planning system using STRIPS-style representations to model a real-world drone delivery scenario. A Breadth-First Search (BFS) planner is used to generate a sequence of actions that allows a drone to successfully deliver packages while managing battery constraints.

![Drone](Drone.jpeg)

# Part 1: Domain Selection & Description

## Real-World Scenario
The domain models an autonomous delivery drone responsible for transporting packages from a central base to multiple customers. The drone must manage limited battery resources, travel between locations, pick up packages, deliver them, and recharge when necessary. For example, once a food delivery is ordered online, a drone is obligated to get the food at the pick-up restaurant and deliver it to the customer at home while ensuring it can get back to the base with battery constraints. 

This scenario reflects real-world logistics systems used in modern drone delivery services, where intelligent planning is required to optimize routes and ensure successful mission completion.



## Entities / Objects in the Domain

The domain contains the following objects:

- **Drone** – the autonomous agent performing deliveries.
- **Base** – the central hub where the drone starts and can recharge.
- **Pickup Location** – where packages are stored before delivery.
- **Customers (c1–c5)** – delivery destinations.
- **Packages** – one package per customer.
- **Battery Levels** – discretized energy states ranging from `battery_0` to `battery_10`.


## Agent Objective
The drone's objective is to:

1. Deliver all packages to their respective customers  
2. Manage battery usage during travel  
3. Recharge when necessary  
4. Return to base after completing all deliveries  



## Why Planning is Needed

A simple reflex agent is insufficient for this domain because actions have **long-term consequences**, that is, without proper planning, the drone may run out of batteries before completing the tasks.

Therefore, the agent must reason about future states and choose an action sequence that satisfies all constraints, making this a classical planning problem.

---

# Part 2: STRIPS Formalization

## Predicates / Fluents

These predicates describe the world state:

### Location Predicates
- `at_base`
- `at_pickup`
- `at_c1` ... `at_c5`

### Battery Predicates
- `battery_0` through `battery_10`

Only one battery level can be true at a time.

### Carrying Predicates
- `carrying_none`
- `carrying_c1` ... `carrying_c5`

### Package Status
- `package_cX_at_pickup`
- `delivered_cX`

---

## Action Schemas

### 1. Fly Action

**Example:** `fly_pickup_to_c1_b10`

**Preconditions:**
- Drone is at the departure location  
- Drone has sufficient battery  

**Add Effects:**
- Drone is at the destination  
- Battery is reduced  

**Delete Effects:**
- Previous location  
- Previous battery level  



### 2. Pick Up Package

**Example:** `pickup_package_c1`

**Preconditions:**
- Drone is at pickup location  
- Drone is not carrying a package  
- Package is available  

**Add Effects:**
- Drone is carrying the package  

**Delete Effects:**
- Package is no longer at pickup  
- Drone is no longer empty  



### 3. Deliver Package

**Example:** `deliver_package_c1`

**Preconditions:**
- Drone is at the customer location  
- Drone is carrying the correct package  

**Add Effects:**
- Package is delivered  
- Drone becomes empty  

**Delete Effects:**
- Drone is no longer carrying the package  



### 4. Recharge Battery

**Example:** `recharge_at_base_b4`

**Preconditions:**
- Drone is at base  
- Battery is below full  

**Add Effects:**
- Battery is restored to full (`battery_10`)  

**Delete Effects:**
- Previous battery level  

---

## Example Problem Instance

### Initial State
- Drone is at base
- Battery is full (`battery_10`)
- Drone is not carrying any package
- All packages are at the pickup location

### Goal State
- All packages have been delivered
- Drone has returned to base

---

# Part 3: Python Implementation

The planner is implemented in Python using STRIPS principles.



## Domain Definition
All ground actions are programmatically generated in `create_actions()`, allowing the planner to scale to larger problem sizes.

Battery consumption varies depending on customer distance, creating a non-trivial state space that requires intelligent planning.



## Planner Functions

### `is_applicable(state, action)`
Checks whether an action’s preconditions are satisfied in the current state.



### `apply_action(state, action)`
Applies STRIPS transitions:

```
new_state = (state - delete_list) ∪ add_list
```

Returns a new immutable state to ensure safe tracking during search.



### `goal_satisfied(state, goal)`
Verifies that all goal fluents are true.



### `forward_search(initial_state, goal_state, actions)`
Implements **A\*** search, which:

- Uses a priority queue ordered by `f = g + h`  
- Optimizes for **battery cost** (each action has a cost)  
- Uses a heuristic to estimate remaining battery cost  
- Avoids revisiting states using an explored set and best-cost tracking 

The heuristic function is admissible because it estimate the lower bound of the battery cost by summing the costs of trips from remaining customers to the pickup. Since it ignores extra required moves like needing to go to pickup before each delivery and recharging at base, it usually underestimate but will never overestimate the cost. 

## Why A* Instead of BFS

We used A\* instead of BFS because this domain is effectively a weighted planning problem: different flights consume different amounts of battery (some customers are farther than others). BFS treats every action as the same cost and only minimizes the number of actions, which can lead to battery-inefficient routes. A\* lets us optimize total battery cost, while still using a heuristic to guide the search and reduce unnecessary exploration.

## Main Execution Flow

The `main()` function:

1. Defines actions  
2. Creates the initial and goal states  
3. Runs the planner  
4. Prints:

- The resulting plan  
- Plan length  
- Number of states explored  

If no solution exists, the program reports `"No plan found"`.



# How to Run

```bash
python Drone_astar.py
```

---

# Summary

This project demonstrates how classical planning techniques can be applied to a realistic logistics problem. By modeling battery constraints, delivery requirements, and spatial movement, the planner must reason about future consequences rather than relying on simple reactive behavior.

The result is a structured, scalable planning system capable of solving complex delivery tasks.

