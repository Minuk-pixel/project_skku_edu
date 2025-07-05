if __name__ == "__main__":
    from utils.control_utils import send_command
    from Function_Library import *

    go_to_target_slot(target_slot=2)
    align_for_parking()
    execute_parking()