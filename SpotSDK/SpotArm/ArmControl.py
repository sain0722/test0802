import threading
import time


class ArmControl:

    def __init__(self, arm):
        self.arm = arm

    def gripper_open_and_close(self):
        self.arm.gripper_open()
        time.sleep(1)
        self.arm.gripper_close()

    def test_move(self):
        self.arm.joint_move('sh0_right')
        time.sleep(0.5)

        self.arm.joint_move('sh0_right')
        time.sleep(0.5)

        self.arm.joint_move('sh0_right')
        time.sleep(0.5)

        self.arm.joint_move('el0_up')
        time.sleep(0.5)

        self.arm.joint_move('el0_up')
        time.sleep(0.5)

        self.arm.joint_move('wr0_up')
        time.sleep(0.5)

        self.arm.joint_move('wr1_right')
        time.sleep(0.5)

        self.arm.joint_move('wr0_up')
        time.sleep(0.5)

        self.arm.joint_move('wr1_right')
        time.sleep(0.5)
