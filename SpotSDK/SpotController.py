from SpotSDK.SpotArm.Arm import Arm
from SpotSDK.SpotArm.ArmControl import ArmControl
from SpotSDK.SpotBody.Body import Body
from SpotSDK.SpotCamera.Camera import Camera
from SpotSDK.SpotGlobal.CarInspectionFiducial import CarInspection
from SpotSDK.SpotGlobal.Fiducial import Fiducial
from SpotSDK.SpotGlobal.GraphNav import GraphNav
from SpotSDK.SpotPermission.Robot import Robot
from SpotSDK.SpotPermission.Power import Power
from SpotSDK.SpotPermission.Estop import Estop
from SpotSDK.SpotPermission.Lease import Lease


class SpotController(Robot, Power, Estop, Lease, Body, Arm, Camera):

    def __init__(self, s_hostname):
        # s_hostname = "192.168.200.66"
        # Robot.__init__(self, s_hostname)
        # Power.__init__(self, self.power_client, self.RobotCommandExecutor)
        # Estop.__init__(self, self.estop_client)
        # Lease.__init__(self, self.lease_client)
        # Body.__init__(self, self.RobotCommandExecutor)
        # Arm.__init__(self, self.RobotCommandExecutor, self.get_current_joint_state)
        # Camera.__init__(self, self.robot)
        self.ROBOT    = Robot(s_hostname)
        self.POWER    = Power(self.ROBOT.power_client, self.ROBOT.RobotCommandExecutor)
        self.ESTOP    = Estop(self.ROBOT.estop_client)
        self.LEASE    = Lease(self.ROBOT.lease_client)
        self.BODY     = Body(self.ROBOT.RobotCommandExecutor)
        self.ARM      = Arm(self.ROBOT.RobotCommandExecutor, self.ROBOT.get_current_joint_state)
        self.CAMERA   = Camera(self.ROBOT.robot)
        self.FIDUCIAL = Fiducial(self.ROBOT)
        self.GRAPHNAV = None

        self.CarInspection = CarInspection(self.FIDUCIAL)
        # # Wait for the first responses.
        # if self._robot_state_task is not None:
        #     self.init_class()
        # self.arm = Arm(self.ROBOT.RobotCommandExecutor, self.ROBOT.get_current_joint_state)
        # self.arm_control = ArmControl(self.arm)
        # self.Fiducial = Fiducial(self.ROBOT)

    def shutdown(self):
        if self.ESTOP.estop_keepalive:
            self.ESTOP.return_estop()

        if self.ROBOT.robot.is_powered_on():
            self.POWER.safe_power_off()

        if self.LEASE.lease_keepalive:
            self.LEASE.return_lease()

    def set_graph_nav_object(self, upload_path):
        self.GRAPHNAV = GraphNav(self.ROBOT, self.LEASE, upload_path)
