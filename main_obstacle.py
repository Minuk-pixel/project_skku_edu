if __name__ == "__main__":
    from utils.vision_utils import detect_obstacle, detect_traffic_light, detect_stop_line
    from utils.control_utils import send_command
    from Function_Library import *

    while True:
        if detect_obstacle():
            handle_obstacle()
        elif detect_stop_line():
            if detect_traffic_light() == "RED":
                send_command("S")
            else:
                continue_driving()
        else:
            continue_driving()