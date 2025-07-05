import keyboard
from utils.control_utils import CarController


def main():
    car = CarController(port="COM4")
    angle = 0  # 초기 조향 각도

    print("🚗 키보드 제어 시작 (ESC로 종료)")
    print("[W] 전진, [S] 후진, [X] 정지, [A] 좌회전, [D] 우회전, [C] 중앙, [←][→] 조향각도 조절")

    try:
        while True:
            if keyboard.is_pressed("w"):
                car.forward()
            elif keyboard.is_pressed("s"):
                car.backward()
            elif keyboard.is_pressed("x"):
                car.stop()
            elif keyboard.is_pressed("a"):
                car.steer_left()
            elif keyboard.is_pressed("d"):
                car.steer_right()
            elif keyboard.is_pressed("c"):
                car.steer_center()
            elif keyboard.is_pressed("left"):
                angle -= 5
                angle = max(-20, angle)
                car.steer_angle(angle)
            elif keyboard.is_pressed("right"):
                angle += 5
                angle = min(20, angle)
                car.steer_angle(angle)
            elif keyboard.is_pressed("esc"):
                print("⛔ 종료합니다.")
                car.stop()
                car.steer_center()
                break

    except KeyboardInterrupt:
        print("⛔ 강제 종료됨.")
    finally:
        car.close()


if __name__ == "__main__":
    main()
