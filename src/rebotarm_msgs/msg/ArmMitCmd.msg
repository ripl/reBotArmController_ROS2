# MIT command for every arm joint, in /rebotarm/joint_states order.
# Published on /rebotarm/arm/stream/mit. header.stamp is when the arm should reach pos (system clock): the
# control loop sends pos linear in time from the last position it sent, then pos.
std_msgs/Header header
float64[] pos
float64[] vel
float64[] kp
float64[] kd
float64[] tau
