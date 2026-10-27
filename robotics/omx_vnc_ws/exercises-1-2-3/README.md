[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/hoZw_su5)
<img width="1276" height="779" alt="image" src="https://github.com/user-attachments/assets/ca457a10-d58f-4757-91ae-d9b7e1fd9157" />

# Exercise 1: Simultaneous Control of OpenMANIPULATOR-X

Create a ROS 2 package named *openmanipulator_motion_control* that moves:

- the base joint (joint1) left and right, and
- the elbow joint (joint3) up (top) and down (bottom),

in an Simultaneous pattern using a timer callback.

## Node should:

- Publish to two joint command topics
- Simultaneously every few seconds
- Print clear logs describing motion states

## By completing this exercise, you will:

- Create and build a custom ROS 2 package in Python.
- Publish joint-specific position commands to the OpenMANIPULATOR-X arm.
- Move the robot Left–Right (base joint) and Top–Bottom (elbow joint).
- Observe the corresponding motions in Gazebo or RViz.

## Concepts Covered

- ROS 2 package creation
- Node & Publisher creation
- Joint-specific topic control
- Timer callbacks
- Understanding robot joint hierarchy (base, shoulder, elbow, wrist)

---

# Exercise 2: Sequential Control of OpenMANIPULATOR-X

Make the arm move left–right first, then top–bottom, repeating this sequence continuously.

---

# Exercise 3: Interactive Direction Control (Left, Right, Top, Bottom, Reset)

Allow the user to control the OpenMANIPULATOR-X arm interactively from the terminal by typing commands:

- `left` → rotate base joint (joint1) counter-clockwise
- `right` → rotate base joint clockwise
- `top` → raise arm (elbow joint3 upward)
- `bottom` → lower arm (elbow joint3 downward)
- `reset` → return to default (home) position

## Expected Behavior

- It prompts user input.
- When you type left, right, top, bottom, or reset, the robot moves to the specified direction
