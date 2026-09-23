# MIT command for every arm joint, in /rebotarm/joint_states order.
# Published on /rebotarm/arm/stream/mit; the control loop resends the latest command every cycle.
std_msgs/Header header
float64[] pos
float64[] vel
float64[] kp
float64[] kd
float64[] tau
