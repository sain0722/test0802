from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder

class Body:

    def __init__(self, robot_command_executor):
        self.robot_command_executor = robot_command_executor
        self.VELOCITY_BASE_SPEED   = 0.5  # m/s
        self.VELOCITY_BASE_ANGULAR = 0.8  # rad/sec

    def sit(self):
        return self.robot_command_executor.start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def stand(self):
        return self.robot_command_executor.start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def move_forward(self):
        return self.robot_command_executor.velocity_cmd_helper('move_forward', v_x=self.VELOCITY_BASE_SPEED)

    def move_backward(self):
        return self.robot_command_executor.velocity_cmd_helper('move_backward', v_x=-self.VELOCITY_BASE_SPEED)

    def strafe_left(self):
        return self.robot_command_executor.velocity_cmd_helper('strafe_left', v_y=self.VELOCITY_BASE_SPEED)

    def strafe_right(self):
        return self.robot_command_executor.velocity_cmd_helper('strafe_right', v_y=-self.VELOCITY_BASE_SPEED)

    def turn_left(self):
        return self.robot_command_executor.velocity_cmd_helper('turn_left', v_rot=self.VELOCITY_BASE_ANGULAR)

    def turn_right(self):
        return self.robot_command_executor.velocity_cmd_helper('turn_right', v_rot=-self.VELOCITY_BASE_ANGULAR)
