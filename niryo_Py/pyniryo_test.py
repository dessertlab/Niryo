from pyniryo import NiryoRobot, JointsPosition

robot_ip_address = "192.168.100.205"

# Connect to robot & calibrate
robot = NiryoRobot(robot_ip_address)
robot.calibrate_auto()
# Move joints
robot.move(JointsPosition(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
# Turn learning mode ON
robot.set_learning_mode(True)
# Stop TCP connection
robot.close_connection()