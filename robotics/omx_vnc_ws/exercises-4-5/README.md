[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/lT2ytd7l)
# Exercise 4: Service-Based Control of OpenMANIPULATOR-X

Create a ROS 2 Service called `/move_arm` that accepts a direction command (left, right, top, bottom, reset) and moves the OpenMANIPULATOR-X arm accordingly. The client sends a single request each time; the server performs one corresponding motion and replies with a success message.

## Concepts Covered

- Defining and using ROS 2 services
- Understanding request–response vs. publisher–subscriber
- Creating both Service Server and Client nodes
- Maintaining internal joint state on the server side

## Expected Behavior

- The server node remains running, waiting for requests.
- The client prompts user input.
- When you type left, right, top, bottom, or reset, the robot moves and the server responds with a feedback string.

---

# Exercise 5: Service-Based Control for Predefined Arm Poses

Create a ROS 2 service `/set_pose` that moves the OpenMANIPULATOR-X arm to predefined poses such as:

- home (neutral position)
- pick (reaching down)
- place (placing an object forward)
- ready (intermediate ready pose)

The client will request one of these poses, and the server will command all relevant joints accordingly.

## Concepts Covered

- Using services for multi-joint control
- Mapping high-level pose names → sets of joint positions
- Sending multiple topic commands simultaneously
- Writing cleaner, modular service callbacks

## Expected Behavior

| Command | Description |
|---------|-------------|
| `home` | All joints to 0 rad (neutral) |
| `ready` | Slightly lifted ready stance |
| `pick` | Arm reaches downward (as if picking) |
| `place` | Arm extends forward (as if placing) |
| `quit` | Exit client |
