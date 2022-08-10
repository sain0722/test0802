import threading
import time

import bosdyn.client.util
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.docking import blocking_dock_robot
from bosdyn.client.estop import EstopClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.world_object import WorldObjectClient

from SpotSDK.SpotGlobal.GlobalDefine import AsyncRobotState
from SpotSDK.SpotPermission.Estop import Estop
from SpotSDK.SpotPermission.Lease import Lease
from SpotSDK.SpotPermission.Power import Power
from SpotSDK.SpotPermission.RobotCommand import RobotCommandExecutor
from SpotSDK.SpotBody.Body import Body


def create_robot(hostname):
    sdk = bosdyn.client.create_standard_sdk('SpotClient')
    robot = sdk.create_robot(hostname)
    bosdyn.client.util.authenticate(robot)

    return robot

class Robot:
    n_dock_id = 520

    def __init__(self, hostname):
        # self.robot = "robot"
        # self.robot_command_client = 'robot_command_client'
        self.robot = create_robot(hostname)

        # Power.__init__(self, self.robot)
        # Lease.__init__(self, self.robot)
        # Estop.__init__(self, self.robot)

        self.robot_id             = self.robot.ensure_client(RobotIdClient.default_service_name).get_id(timeout=0.4)
        self.power_client         = self.robot.ensure_client(PowerClient.default_service_name)
        self.estop_client         = self.robot.ensure_client(EstopClient.default_service_name)
        self.lease_client         = self.robot.ensure_client(LeaseClient.default_service_name)

        self.robot_state_client   = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.image_client         = self.robot.ensure_client(ImageClient.default_service_name)
        self.world_object_client  = self.robot.ensure_client(WorldObjectClient.default_service_name)

        self.RobotCommandExecutor = RobotCommandExecutor(self.robot_command_client)

        self._robot_state_task = AsyncRobotState(self.robot_state_client)
        self._async_tasks = AsyncTasks([self._robot_state_task])

        self.stop_state_event = threading.Event()
        # self.stop_state_event.clear()
        self.get_state_thread = None

        # async get state
        self.start_getting_state()
        print("Spot connect succeeded.")

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def start_getting_state(self):

        # @TODO: robot state async callback_done function
        callback_is_done = False

        self.get_state_thread = threading.Thread(target=self.thread_state)
        self.get_state_thread.daemon = True
        self.get_state_thread.start()

    def thread_state(self):
        while not self.stop_state_event.isSet():
            self._async_tasks.update()

    def get_current_joint_state(self):
        state = self.robot_state
        if not state:
            return None
        joint_states = state.kinematic_state.joint_states
        joint_names = ['arm0.sh0', 'arm0.sh1', 'arm0.el0', 'arm0.el1', 'arm0.wr0', 'arm0.wr1']
        joint_pos_list = [
            state.position.value
            for state in joint_states if state.name in joint_names
        ]
        joint_pos_dict = {
            name.split(".")[1]: round(value, 4)
            for name, value in zip(joint_names, joint_pos_list)
        }

        return joint_pos_dict

    def power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def dock(self):
        # make sure standing
        blocking_stand(self.robot_command_client)

        # Dock the robot
        # docking하는 도중에 프로그램이 멈춰서 도킹을 쓰레드로 작성
        dock_thread = threading.Thread(target=blocking_dock_robot, args=[self.robot, self.n_dock_id], daemon=True)
        dock_thread.start()

    def docking(self):
        # make sure standing
        blocking_stand(self.robot_command_client)

        # Dock the robot
        blocking_dock_robot(self.robot, self.n_dock_id)

        return 'docking done'


if __name__ == "__main__":
    # from Estop import Estop
    # from Lease import Lease
    # from Power import Power
    Robot("192.168.200.66")
