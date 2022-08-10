import time

from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.lease import Error as LeaseBaseError

# from SpotSDK.SpotPermission.Robot import Robot
from bosdyn.api import arm_command_pb2, synchronized_command_pb2, robot_command_pb2


def try_grpc(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError, LeaseBaseError) as err:
        # self.add_message("Failed {}: {}".format(desc, err))
        print("Failed {}: {}".format(desc, err))
        return None
        # return "Failed {}: {}".format(desc, err)

def try_grpc_async(desc, thunk):
    def on_future_done(fut):
        try:
            fut.result()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            # self.add_message("Failed {}: {}".format(desc, err))
            print("Failed {}: {}".format(desc, err))
            return None
            # return "Failed {}: {}".format(desc, err)

    future = thunk()
    future.add_done_callback(on_future_done)

def make_robot_command(arm_joint_traj):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)


class RobotCommandExecutor:
    def __init__(self, client):
        self.robot_command_client  = client
        self.VELOCITY_CMD_DURATION = 0.6  # seconds

    def start_robot_command(self, desc, command_proto, end_time_secs=None):
        def _start_command():
            return self.robot_command_client.robot_command(lease=None, command=command_proto,
                                                           end_time_secs=end_time_secs)

        return try_grpc(desc, _start_command)

    def velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        return self.start_robot_command(
                desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
                end_time_secs=time.time() + self.VELOCITY_CMD_DURATION)

    def joint_move_cmd_helper(self, params, desc='', time_secs=1.0):

        sh0, sh1, el0, el1, wr0, wr1 = params
        # time_secs = JOINT_TIME_SEC
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1, time_since_reference_secs=time_secs)

        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])
        command = make_robot_command(arm_joint_traj)

        return self.start_robot_command(desc=desc, command_proto=command)

