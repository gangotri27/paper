[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/7k8TyJk4)
# Exercise 6: Service-Based Pick-and-Place Task Orchestration (arm only)

Build a ROS 2 service `/perform_task` that executes a full pick-and-place sequence on the OpenMANIPULATOR-X.

The task should automatically move through a predefined series of poses:

home → ready → pick → place → home

The client triggers the task once; the server executes all steps in order and replies when the sequence is complete.

## Concepts Covered

- Multi-step motion orchestration within a service callback
- Using delays (time.sleep()) to allow motion completion
- Publishing sequential joint commands
- Designing higher-level "task services" built on pose primitives

## Expected Behavior

**The arm should:**

- Move to home position
- Transition to ready
- Move downward to pick pose
- Extend forward to place pose
- Return to home

**The service then responds:**
```
Pick-and-Place task completed successfully!
```


# 6A: Adding and Positioning Objects in Gazebo for Pick-and-Place (setup)

Introducing object creation, placement, and physics interaction in Gazebo.

- Spawn a movable cube at the correct pick position.
- Align it with the OpenMANIPULATOR-X gripper in Gazebo.
- Observe realistic gravity and collisions.

## Concepts Covered

- Gazebo world file structure
- Spawning models via XML/SDF
- Object placement (pose) and scaling
- Static vs. dynamic objects
- Physics and friction parameters

## Expected Behavior

- A small movable cube resting on the table in front of the arm
- When you run Exercise 6 (Pick-and-Place Service), the gripper aligns above it at the pick position



# 6B: Using Gazebo Link Attacher Plugin for Realistic Grasping

Integrate the `gazebo_ros_link_attacher` plugin so that when the gripper closes, the cube attaches to the gripper links; and when it opens, the cube detaches.

## Concepts Covered

- Gazebo → ROS plugin integration
- Attaching and detaching links during runtime
- Calling Gazebo services from Python (`/attach` & `/detach`)
- Synchronizing gripper motion with simulation physics

## Expected Behavior

| Step | Action | Physics Result |
|------|--------|----------------|
| Pick | Gripper closes → attach service called | Cube links to gripper; moves rigidly |
| Lift | Arm moves upward | Cube moves with arm |
| Place | Gripper opens → detach service called | Cube falls by gravity onto table |

---

# Exercise 7: Pick-and-Place with Gripper Control (full simulation)

Extend the previous `/perform_task` service so that, during the pick-and-place sequence, the gripper closes to grab an object at the pick pose and opens to release it at the place pose.

## Concepts Covered

- Publishing to the gripper joint topic
- Synchronizing gripper and arm motions
- Combining sequential actions in one service callback
- Building realistic end-to-end manipulation logic

## Expected Behavior

| Step | Action | Description |
|------|--------|-------------|
| 1 | Home | Arm resets to base position |
| 2 | Ready | Moves into pre-action pose |
| 3 | Pick | Arm descends |
| 4 | Gripper Close | Object grasped |
| 5 | Ready | Lifts object |
| 6 | Place | Moves to release area |
| 7 | Gripper Open | Object released |
| 8 | Home | Returns to start |
