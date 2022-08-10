from bosdyn.client.robot_command import RobotCommandBuilder


class Arm:

    def __init__(self, robot_command_executor, get_current_joint_state):
        # current joint state
        # sh0, sh1, el0, el1, wr0, wr1
        self.function = get_current_joint_state
        self.joint_params = get_current_joint_state()
        self.robot_command_executor = robot_command_executor

        self.JOINT_MOVE_RATE = 0.1  # arm joint move rate
        self.JOINT_TIME_SEC  = 0.5  # arm control speed

    def stow(self):
        return self.robot_command_executor.start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def unstow(self):
        return self.robot_command_executor.start_robot_command('unstow', RobotCommandBuilder.arm_ready_command())

    def gripper_open(self):
        return self.robot_command_executor.start_robot_command('gripper_open', RobotCommandBuilder.claw_gripper_open_command(),
                                                               end_time_secs=1)

    def gripper_close(self):
        return self.robot_command_executor.start_robot_command('gripper_close', RobotCommandBuilder.claw_gripper_close_command())

    def joint_move(self, target):
        self.joint_params = self.function()
        if target == "sh0_right":
            self.joint_params['sh0'] = self.joint_params['sh0'] - self.JOINT_MOVE_RATE

        elif target == "sh0_left":
            self.joint_params['sh0'] = self.joint_params['sh0'] + self.JOINT_MOVE_RATE

        elif target == "sh1_up":
            self.joint_params['sh1'] = self.joint_params['sh1'] - self.JOINT_MOVE_RATE

        elif target == "sh1_down":
            self.joint_params['sh1'] = self.joint_params['sh1'] + self.JOINT_MOVE_RATE

        elif target == "el0_up":
            self.joint_params['el0'] = self.joint_params['el0'] - self.JOINT_MOVE_RATE

        elif target == "el0_down":
            self.joint_params['el0'] = self.joint_params['el0'] + self.JOINT_MOVE_RATE

        elif target == "el1_right":
            self.joint_params['el1'] = self.joint_params['el1'] + self.JOINT_MOVE_RATE

        elif target == "el1_left":
            self.joint_params['el1'] = self.joint_params['el1'] - self.JOINT_MOVE_RATE

        elif target == "wr0_up":
            self.joint_params['wr0'] = self.joint_params['wr0'] - self.JOINT_MOVE_RATE

        elif target == "wr0_down":
            self.joint_params['wr0'] = self.joint_params['wr0'] + self.JOINT_MOVE_RATE

        elif target == "wr1_right":
            self.joint_params['wr1'] = self.joint_params['wr1'] - self.JOINT_MOVE_RATE

        elif target == "wr1_left":
            self.joint_params['wr1'] = self.joint_params['wr1'] + self.JOINT_MOVE_RATE

        return self.robot_command_executor.joint_move_cmd_helper(desc=target,
                                                                 params=self.joint_params.values(),
                                                                 time_secs=self.JOINT_TIME_SEC)

    def joint_move_sh0_right(self):
        self.joint_params['sh0'] = self.joint_params['sh0'] - self.JOINT_MOVE_RATE
