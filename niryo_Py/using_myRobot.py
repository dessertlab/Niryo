import os
from myrobot_class import MyRobot

# - - - - -
# IP
# - - - - -
ROBOT_25_WIFI_IP = "192.168.100.205"
ROBOT_C5_WIFI_IP = "192.168.100.188"

robot_name = "C25"
if __name__=="__main__":
    try:
        ip = ROBOT_25_WIFI_IP if robot_name=="C25" else ROBOT_C5_WIFI_IP
        robot = MyRobot(ip, robot_name, use_degrees=True)
        robot.load_from_json(os.path.join(os.path.dirname(__file__), "sequences.json"))
        coreography = robot.get_saved_sequence("say-hi")

        if True:
            robot.run_sequence(coreography)

    except Exception as e:
        print("ERROR: ", e)
        raise
    
    finally:
        # Chiudi tutto
        robot.disconnect()

        print("The end!")