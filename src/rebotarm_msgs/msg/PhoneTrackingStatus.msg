uint8 DISCONNECTED=0
uint8 UNCALIBRATED=1
uint8 COLLECTING=2
uint8 CALIBRATED=3
uint8 ERROR=4

std_msgs/Header header
uint8 state
string session_id
bool stream_valid
bool calibration_valid
uint32 calibration_samples
uint32 calibration_target_samples
string message
