import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from inner_hull_status_interfaces.msg import InnerHullStatus
import pygame
import time
import os


class SoundController:
    def __init__(self, sound_file: str):
        pygame.mixer.init()
        pygame.mixer.music.load(sound_file)

    def play(self):
        pygame.mixer.music.play()


class StatusSub(Node):
    def __init__(self):
        super().__init__("status_sub")
        history_count = 10
        self.subscription = self.create_subscription(
            InnerHullStatus,
            "rov/inner_hull_status",
            self.listener_callback,
            history_count,
        )
        self.subscription  # prevent unused variable warning

        sound_file = "meka_ge_keihou10.mp3"
        sound_file_path = os.path.join(os.path.dirname(__file__), sound_file)
        self.sc = SoundController(sound_file_path)

    def listener_callback(self, msg):
        if msg.water_leak_1:
            self.sc.play()
            print("\n\n\nError!!! Water leakage happens.\n\n\n")
        pmsg = "\n\nwater_leak_1 [], water_leak_1_volt [V], temp [C.deg], pressure [hPa], humidity [%]\n"
        pmsg += f"{msg.water_leak_1}, {msg.water_leak_1_volt:3.1f}, {msg.temperature_1:5.1f}, {msg.barometric_pressure:6.1f}, {msg.humidity:2.0f}"
        print(pmsg)


def main(args=None):
    rclpy.init(args=args)

    subscriber = StatusSub()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
