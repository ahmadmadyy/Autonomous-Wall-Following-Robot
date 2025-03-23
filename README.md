# Wall-Following Robot Behavior with ROS2

## PART I: Topics

In this part, you will practice using **topics** to control a robot.  
Your goal is to create a **ROS2 program** that makes the robot exhibit a **wall-following behavior**.

---

### 🧭 Wall-Following Behavior

The robot must follow the wall on its **right-hand side**, maintaining a distance of **30 cm (0.3 m)**.

#### ✅ To implement this:

1. **Subscribe to the laser topic**
   - Read the ray at **90º to the right** (relative to the front of the robot).
   - Use it to measure distance to the wall.

2. **Control logic:**
   - If distance > **0.3 m** → **Turn slightly right**
   - If distance < **0.2 m** → **Turn slightly left**
   - If distance is between **0.2 m and 0.3 m** → **Move forward**

---

#### ⚠️ Handling Wall Transitions

- Use the **front laser ray**.
- If the front ray reads < **0.5 m**:
  - **Turn left** while moving forward to follow the new wall.

The result of this behavior must be that the robot moves along the whole environment.

---

### 🧪 Testing Your Wall-Following Program

1. Launch the simulation environment.
2. In another terminal, run:

   ```bash
   source ros2_ws/install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. Use the keyboard to move the robot to a **convenient position** near a wall, facing along it.

4. > ⚠️ **IMPORTANT:**  
   > Close the teleop terminal after positioning the robot.  
   > If you don’t, it will interfere with your program.

5. Now launch your ROS program and observe the behavior.
6. If the robot doesn’t behave correctly, debug and repeat the process.

> 🧠 **Reminder:** Always verify your implementation in simulation before trying it on a real robot.

---

## PART II: Services

### 🎯 Goal: Create a ROS Service to Find the Wall

You must:

1. Create a **ROS2 service server** named `find_wall` using a custom message `FindWall.srv`.
2. Modify the **PART I program** to call this service before starting the control loop.
3. Create a **launch file** to launch both nodes.

---

### 📄 `FindWall.srv` Definition

```srv
---
bool wallfound
```

### 🧠 Service Behavior:

1. Identify the **shortest laser ray** → assume it's the wall.
2. **Rotate the robot** to face the wall (ray 0 becomes shortest).
3. **Move forward** until ray 0 is < **0.3 m**.
4. Rotate so **ray 270** is facing the wall.
5. Return `wallfound = true`.

---

### 🧪 Testing the Service

- Launch the service server node.
- Call the service using:

   ```bash
   ros2 service call /find_wall project_interfaces/srv/FindWall
   ```

---

### 🛠 Modify PART I Program

- Add a **service client** that calls `find_wall` before the wall-following loop.

---

### 🚀 Launch File

- Create `main.launch.py` that launches:
  - The service server node.
  - The wall-following node.

> 🧠 **Reminder:** Test everything in simulation before using a real robot.

---

## PART III: Actions

### 🎯 Goal: Record Odometry with an Action Server

You must:

1. Create an **action server** named `record_odom` using `OdomRecord.action`.
2. Modify the **wall-following node** to call the action server before control starts.
3. Add the action server to the `main.launch.py`.

---

### 📄 `OdomRecord.action` Definition

```action
---
geometry_msgs/msg/Point32[] list_of_odoms
---
float32 current_total
```

### 🧠 Action Server Behavior

- Record **(x, y, θ)** as `Point32` every **1 second**.
- **Feedback:** Total meters traveled.
- **Finish condition:** When robot completes a lap.
- **Result:** Return all recorded odometries.

---

### 🧪 Test the Action Server

- Launch the action server node.
- Test using:

   ```bash
   ros2 run action_tutorials axclient
   ```

- Send a goal to `/record_odom`.

---

### 🛠 Modify Wall-Follower Node

- Add an **action client** that starts odometry recording before wall-following.

---

### 🚀 Update Launch File

- Add the action server node to `main.launch.py`.

> 🧠 **Reminder:** Once working in simulation, book a session in the Real Robot Lab for hardware testing.
