import bosdyn.api.power_pb2 as PowerServiceProto
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient

from SpotSDK.SpotPermission.RobotCommand import RobotCommandExecutor


class Power:
    """
        parameter:
            - power_client
        function:
            - power_on
            - power_off
    """
    def __init__(self, client, robot_command_executor):
        self.power_client = client
        self.robot_command_executor = robot_command_executor

    def request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self.power_client.power_command_async(request)

    def safe_power_off(self):
        command = RobotCommandBuilder.safe_power_off_command()
        return self.robot_command_executor.start_robot_command('safe_power_off', command)
