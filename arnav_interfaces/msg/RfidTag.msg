# One RFID tag read by the reader on the underside of the robot.
# tag_id is what the reader reports (a short id like "01", or an EPC that arnav_sim/config/layout.yaml maps to one).
std_msgs/Header header
string tag_id
float32 rssi        # signal strength, any unit (0 if the reader does not give one)
