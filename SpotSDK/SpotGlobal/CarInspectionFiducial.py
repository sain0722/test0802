import math
import threading
import time

from bosdyn.api import world_object_pb2, basic_command_pb2
from bosdyn.client import frame_helpers, math_helpers
from bosdyn.client.robot_command import RobotCommandBuilder

from SpotSDK.SpotGlobal.Fiducial import Fiducial

def block_for_trajectory_cmd(command_client, cmd_id, timeout_sec):
    """Helper that blocks until a trajectory command reaches STATUS_AT_GOAL or a timeout is
        exceeded."""
    end_time = time.time() + timeout_sec

    while (timeout_sec is None or time.time() < end_time) and threading.current_thread().is_alive():
        feedback_resp = command_client.robot_command_feedback(cmd_id)

        current_state = feedback_resp.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
        movement_state = feedback_resp.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.body_movement_status
        if current_state == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL:
            print("Successfully reached the goal!")
            return True

        elif (current_state == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_GOING_TO_GOAL and
            movement_state == basic_command_pb2.SE2TrajectoryCommand.Feedback.BODY_STATUS_SETTLED):
            print("Body finished moving even though goal not fully achieved.")
            return True

        time.sleep(0.1)

    print("Trajectory command timeout exceeded (or cancelled).")
    return False


def block_for_stance_cmd(command_client, cmd_id, timeout_sec):
    """Helper that blocks until a stance command reaches STATUS_AT_GOAL or a timeout is
        exceeded."""
    end_time = time.time() + timeout_sec

    while (timeout_sec is None or time.time() < end_time) and threading.current_thread().is_alive():
        feedback_resp = command_client.robot_command_feedback(cmd_id)
        current_state = feedback_resp.feedback.synchronized_feedback.mobility_command_feedback.stance_feedback.status
        if current_state == basic_command_pb2.StanceCommand.Feedback.STATUS_STANCED:
            print("Robot has achieved the desired stance.")
            return True

        time.sleep(0.1)

    return False


class CarInspection:

    def __init__(self, fiducial):
        self.Fiducial = fiducial

    def get_nearest_fiducial(self, fiducials):
        """Determine the fiducial closest to the body position."""
        assert len(fiducials) >= 1
        odom_tform_body = frame_helpers.get_odom_tform_body(fiducials[0].transforms_snapshot)

        closest = None
        distance_to_closest = None
        for fid in fiducials:
            if fid.apriltag_properties.tag_id >= 500:
                # Skip docking fiducials.
                continue
            body_tform_fiducial = frame_helpers.get_a_tform_b(fid.transforms_snapshot,
                                                              frame_helpers.BODY_FRAME_NAME,
                                                              fid.apriltag_properties.frame_name_fiducial_filtered)
            dist_2d = math.sqrt(
                body_tform_fiducial.x * body_tform_fiducial.x + body_tform_fiducial.y * body_tform_fiducial.y)
            if distance_to_closest is None:
                closest = fid
                distance_to_closest = dist_2d
            elif dist_2d < distance_to_closest:
                closest = fid
                distance_to_closest = dist_2d
        return closest

    def position_body_over_fiducial(self, world_object_client, robot_command_client):
        """Center the body over the fiducial in a stance."""
        fiducials = world_object_client.list_world_objects(
            object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]).world_objects

        assert len(fiducials) >= 1
        goal_fiducial = self.get_nearest_fiducial(fiducials)
        if not goal_fiducial:
            print("없음")
            return None, None, None

        odom_tform_fiducial = frame_helpers.get_se2_a_tform_b(
            goal_fiducial.transforms_snapshot, frame_helpers.ODOM_FRAME_NAME,
            goal_fiducial.apriltag_properties.frame_name_fiducial)

        stow_cmd = RobotCommandBuilder.arm_stow_command()
        # First walk the robot to the fiducial. Center the body on the fiducial.
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=odom_tform_fiducial.x, goal_y=odom_tform_fiducial.y,
            goal_heading=odom_tform_fiducial.angle, frame_name=frame_helpers.ODOM_FRAME_NAME,
            build_on_command=stow_cmd)

        end_time = 10.0
        cmd_id = robot_command_client.robot_command(command=robot_cmd,
                                                    end_time_secs=time.time() + end_time)

        should_continue = block_for_trajectory_cmd(robot_command_client, cmd_id, end_time)
        if not should_continue and not threading.current_thread().is_alive():
            # Only stop if the thread was explicitly cancelled. Otherwise, we probably just
            # timed out on the trajectory command.
            print("Stopping after the blocking trajectory command because it was cancelled.")
            return should_continue, None, None

        # This example ues the position of the fiducial and specifies the stance offsets relative to
        # the center of the fiducial.
        # Stance offsets from fiducial position.
        x_offset = 0.25
        y_offset = 0.25

        pos_fl_rt_odom = odom_tform_fiducial * math_helpers.SE2Pose(x_offset, y_offset, 0)
        pos_fr_rt_odom = odom_tform_fiducial * math_helpers.SE2Pose(x_offset, -y_offset, 0)
        pos_hl_rt_odom = odom_tform_fiducial * math_helpers.SE2Pose(-x_offset, y_offset, 0)
        pos_hr_rt_odom = odom_tform_fiducial * math_helpers.SE2Pose(-x_offset, -y_offset, 0)

        stance_cmd = RobotCommandBuilder.stance_command(
            frame_helpers.ODOM_FRAME_NAME, pos_fl_rt_odom.position, pos_fr_rt_odom.position,
            pos_hl_rt_odom.position, pos_hr_rt_odom.position)
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        stance_robot_command = RobotCommandBuilder.build_synchro_command(stow_cmd, stance_cmd)

        end_time = 7.0  # seconds
        print("Issuing the stance command.")
        cmd_id_stance = robot_command_client.robot_command(command=stance_robot_command,
                                                           end_time_secs=time.time() + end_time)

        # should_continue = block_for_stance_cmd(robot_command_client, cmd_id_stance, end_time)
        return should_continue, goal_fiducial, stance_cmd

    def centering_on_fiducial(self, fid_id, dist_margin):
        """ Run centering on fiducial service:
            Taking the fid_id and dist_margin
            1. Finds for the waypoint associated with te fiducial
            2. Then, moves the robot to the waypoint
            3. Detects and centers the robot on the closet fiducial in a stable stance.
        """
        # Take the first argument as the destination fiducial number.
        # if len(args) < 1:
        #     print("Wrong input. [fid-id] [dist-margin]")
        #     return

        should_continue, goal_fiducial, _ = self.position_body_over_fiducial(self.Fiducial.world_object_client, self.Fiducial.robot_command_client)
