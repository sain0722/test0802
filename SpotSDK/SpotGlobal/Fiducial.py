import math
import time

from bosdyn.client import frame_helpers, math_helpers
from pupil_apriltags import Detector

import cv2
import numpy as np
from PIL import Image
from bosdyn import geometry
from bosdyn.api import world_object_pb2, geometry_pb2, trajectory_pb2, image_pb2
from bosdyn.api.geometry_pb2 import SE2VelocityLimit, SE2Velocity, Vec2, Vec3
from bosdyn.client.frame_helpers import get_a_tform_b, VISION_FRAME_NAME, get_vision_tform_body, BODY_FRAME_NAME
from bosdyn.client.image import build_image_request
from bosdyn.client.math_helpers import Quat
from bosdyn.client.robot_command import RobotCommandBuilder
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from bosdyn.client.world_object import WorldObjectClient
class Fiducial:

    def __init__(self, robot):
        self.robot = robot
        self.world_object_client = robot.world_object_client
        self.robot_command_client = robot.robot_command_client
        self._image_client = robot.image_client
        self.robot_state = robot.robot_state
        self._standup = True
        self._movement_on = True
        self._powered_on = False

        self._tag_offset = float(0.5) + 1.1 / 2.0  # meters
        self._limit_speed = True  # Limit the robot's walking speed.
        self._avoid_obstacles = True  # Disable obstacle avoidance.

        # Epsilon distance between robot and desired go-to point.
        self._x_eps = .05
        self._y_eps = .05
        self._angle_eps = .075

        # Maximum speeds.
        self._max_x_vel = 0.5
        self._max_y_vel = 0.5
        self._max_ang_vel = 1.0

        # Latest detected fiducial's position in the world.
        self._current_tag_world_pose = np.array([])

        # Heading angle based on the camera source which detected the fiducial.
        self._angle_desired = None
        # List of all possible camera sources.
        self._source_names = [
            src.name for src in self._image_client.list_image_sources() if
            (src.image_type == image_pb2.ImageSource.IMAGE_TYPE_VISUAL and "depth" not in src.name)
        ]
        # print(self._source_names)

        # Transform from the robot's camera frame to the baselink frame.
        # It is a math_helpers.SE3Pose.
        self._camera_tform_body = None

        # Transform from the robot's baselink to the world frame.
        # It is a math_helpers.SE3Pose.
        self._body_tform_world = None

        # Camera intrinsics for the current camera source being analyzed.
        self._intrinsics = None

        # Camera source which a bounding box was last detected in.
        self._previous_source = None

        # Dictionary mapping camera source to it's latest image taken.
        self._image = dict()

    @property
    def image(self):
        """Return the current image associated with each source name."""
        return self._image

    def get_fiducial_objects(self):
        """Get all fiducials that Spot detects with its perception system."""
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self.world_object_client.list_world_objects(
            object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            for fid in fiducial_objects:
                if fid.apriltag_properties.tag_id == 520:
                    continue
                # Return the first detected fiducial.
                return fid
        # Return none if no fiducials are found.
        return None

    @staticmethod
    def get_nearest_fiducial(fiducials):
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

    def position_body_over_fiducial(self):
        fiducials = self.world_object_client.list_world_objects(
            object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
        ).world_objects

        if len(fiducials) == 0:
            print("No fiducials nearby.")
            return None

        fiducial = self.get_nearest_fiducial(fiducials)
        goal_fiducial = self.walk_to_fiducial(fiducial, dist_margin=1.0)
        return goal_fiducial

    def walk_to_fiducial(self, fiducial, dist_margin=1.0):
        if not fiducial:
            print('[walk_to_fiducial] No fiducial nearby.')
            return None

        # SE3 tform
        odom_tform_fiducial = get_a_tform_b(
            fiducial.transforms_snapshot, frame_helpers.ODOM_FRAME_NAME,
            fiducial.apriltag_properties.frame_name_fiducial_filtered
        )

        if self.is_fiducial_horizontal(odom_tform_fiducial):
            print("fiducial is horizontal")
            return False, False
        else:
            goto, heading = self.offset_tag_pose_with_rotation(odom_tform_fiducial, dist_margin)
            odom_T_body = math_helpers.SE3Pose(x=goto.x, y=goto.y, z=goto.z, rot=heading)
            fiducial_T_body = odom_tform_fiducial.inverse() * odom_T_body

            attmept_number = 0
            num_retries = 4
            while attmept_number < num_retries:
                attmept_number += 1
                self.trajectory_cmd(goto, heading)

    def offset_tag_pose_with_rotation(self, odom_tform_fiducial, dist_margin):
        zhat_ewrt_fiducial = Vec3(x=0, y=0, z=1)
        zhat_ewrt_odom = odom_tform_fiducial.rot.transform_vec3(zhat_ewrt_fiducial)
        zhat_ewrt_odom_np = np.array(
            [zhat_ewrt_odom.x, zhat_ewrt_odom.y, zhat_ewrt_odom.z]
        )

        xhat_ewrt_odom_np = np.array([0.0, 0.0, 1.0])

        # Check to see if the fiducial Z-axis is aligned with gravity
        if self.is_fiducial_horizontal(odom_tform_fiducial):
            xhat_ewrt_fiducial = Vec3(x=1, y=0, z=0)
            xhat_ewrt_odom = odom_tform_fiducial.rot.transform_vec3(xhat_ewrt_fiducial)
            xhat_ewrt_odom_np = np.array([xhat_ewrt_odom.x, xhat_ewrt_odom.y, xhat_ewrt_odom.z])
            zhat_ewrt_odom_np = np.array([0.0, 0.0, 1.0])

            xhat_ewrt_odom_np = self.make_orthogonal(zhat_ewrt_odom_np, xhat_ewrt_odom_np)
            goal_position_rt_fiducial = Vec3(x=-dist_margin, y=0, z=0)
        else:
            zhat_ewrt_odom_np = self.make_orthogonal(xhat_ewrt_odom_np, zhat_ewrt_odom_np)
            goal_position_rt_fiducial = Vec3(x=0, y=0, z=dist_margin)

        goal_position_rt_odom = odom_tform_fiducial.transform_vec3(goal_position_rt_fiducial)

        yhat = np.cross(zhat_ewrt_odom_np, xhat_ewrt_odom_np)
        mat = np.array([xhat_ewrt_odom_np, yhat, zhat_ewrt_odom_np]).transpose()
        heading = Quat.from_matrix(mat)

        goal_position_rt_odom_np = np.array([
            goal_position_rt_odom.x, goal_position_rt_odom.y, goal_position_rt_odom.z
        ])

        # return goal_position_rt_odom_np, heading
        return goal_position_rt_odom, heading

    @staticmethod
    def make_orthogonal(primary, secondary):
        # Gram schmidt
        p = primary / np.linalg.norm(primary, ord=2, axis=0, keepdims=True)
        s = secondary / np.linalg.norm(secondary, ord=2, axis=0, keepdims=True)

        u = np.subtract(s, np.multiply(np.dot(p, s) / np.dot(s, s), p))

        normalized_u = u / np.linalg.norm(u, ord=2, axis=0, keepdims=True)
        return normalized_u

    @staticmethod
    def is_fiducial_horizontal(odom_tform_fiducial):
        zhat_ewrt_fiducial = Vec3(x=0, y=0, z=1)
        zhat_ewrt_odom = odom_tform_fiducial.rot.transform_vec3(zhat_ewrt_fiducial)
        zhat_ewrt_odom_np = np.array([
            zhat_ewrt_odom.x, zhat_ewrt_odom.y, zhat_ewrt_odom.z
        ])

        xhat_ewrt_odom_np = np.array([0.0, 0.0, 1.0])
        # Check to see if the fiducial Z-axis is aligned with gravity (0, 0, 1)
        if abs(np.dot(xhat_ewrt_odom_np, zhat_ewrt_odom_np)) > 0.95:
            # The fiducial is probably on the floor
            return True
        return False

    def trajectory_cmd(self, goto, heading, body_height=0):
        rotation = geometry.EulerZXY().to_quaternion()
        NOMINAL_HEIGHT = 0.515
        position = Vec3(x=0.0, y=0.0, z=0.0)
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        heading_yaw = Quat(w=heading.w, x=heading.x, y=heading.y, z=heading.z).to_yaw()
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

        # set mobility params
        mobility_params = self.set_mobility_params(body_control=body_control)

        # command
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goto.x,
            goal_y=goto.y,
            goal_heading=heading_yaw,
            frame_name=frame_helpers.ODOM_FRAME_NAME,
            params=mobility_params
        )

        # Execute
        end_time = 3
        cmd_id = self.robot_command_client.robot_command(command=robot_cmd,
                                                         end_time_secs=time.time() + end_time)

        return cmd_id

    def set_mobility_params(self, body_control=None):
        """Set robot mobility params to disable obstacle avoidance."""
        obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=True,
                                                    disable_vision_foot_obstacle_avoidance=True,
                                                    disable_vision_foot_constraint_avoidance=True,
                                                    obstacle_avoidance_padding=.001)
        if not body_control:
            body_control = self.set_default_body_control()
        if self._limit_speed:
            speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(
                linear=Vec2(x=self._max_x_vel, y=self._max_y_vel), angular=self._max_ang_vel))
            if not self._avoid_obstacles:
                mobility_params = spot_command_pb2.MobilityParams(
                    obstacle_params=obstacles, vel_limit=speed_limit, body_control=body_control,
                    locomotion_hint=spot_command_pb2.HINT_AUTO)
            else:
                mobility_params = spot_command_pb2.MobilityParams(
                    vel_limit=speed_limit, body_control=body_control,
                    locomotion_hint=spot_command_pb2.HINT_AUTO)
        elif not self._avoid_obstacles:
            mobility_params = spot_command_pb2.MobilityParams(
                obstacle_params=obstacles, body_control=body_control,
                locomotion_hint=spot_command_pb2.HINT_AUTO)
        else:
            # When set to none, RobotCommandBuilder populates with good default values
            mobility_params = None
        return mobility_params

    @staticmethod
    def set_default_body_control():
        """Set default body control params to current body position"""
        footprint_R_body = geometry.EulerZXY()
        position = geometry_pb2.Vec3(x=0.0, y=0.0, z=0.0)
        rotation = footprint_R_body.to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

    def compute_fiducial_in_world_frame(self, tvec):
        """Transform the tag position from camera coordinates to world coordinates."""
        fiducial_rt_camera_frame = np.array(
            [float(tvec[0][0]) / 1000.0,
             float(tvec[1][0]) / 1000.0,
             float(tvec[2][0]) / 1000.0])
        body_tform_fiducial = (self._camera_tform_body.inverse()).transform_point(
            fiducial_rt_camera_frame[0], fiducial_rt_camera_frame[1], fiducial_rt_camera_frame[2])
        fiducial_rt_world = self._body_tform_world.inverse().transform_point(
            body_tform_fiducial[0], body_tform_fiducial[1], body_tform_fiducial[2])
        return fiducial_rt_world

    # def start(self):
    #     fiducial = self.get_fiducial_objects()
    #     detected_fiducial = None
    #     fiducial_rt_world = None
    #     use_world_object_service = False
    #     if use_world_object_service:
    #         if fiducial is not None:
    #             vision_tform_fiducial = get_a_tform_b(
    #                 fiducial.transforms_snapshot, VISION_FRAME_NAME,
    #                 fiducial.apriltag_properties.frame_name_fiducial).to_proto()
    #             if vision_tform_fiducial is not None:
    #                 detected_fiducial = True
    #                 fiducial_rt_world = vision_tform_fiducial.position
    #     else:
    #         # Detect the april tag in the images from Spot using the apriltag library.
    #         bboxes, source_name = self.image_to_bounding_box()
    #         if bboxes:
    #             self._previous_source = source_name
    #             (tvec, _, source_name) = self.pixel_coords_to_camera_coords(
    #                 bboxes, self._intrinsics, source_name)
    #             vision_tform_fiducial_position = self.compute_fiducial_in_world_frame(tvec)
    #             fiducial_rt_world = geometry_pb2.Vec3(x=vision_tform_fiducial_position[0],
    #                                                   y=vision_tform_fiducial_position[1],
    #                                                   z=vision_tform_fiducial_position[2])
    #             detected_fiducial = True
    #
    #     if detected_fiducial:
    #         # Go to the tag and stop within a certain distance
    #         # self.go_to_tag(fiducial_rt_world)
    #         pass
    #     else:
    #         print("No fiducials found")
    #
    # def image_to_bounding_box(self):
    #     """Determine which camera source has a fiducial.
    #        Return the bounding box of the first detected fiducial."""
    #     #Iterate through all five camera sources to check for a fiducial
    #     for i in range(len(self._source_names) + 1):
    #         # Get the image from the source camera.
    #         if i == 0:
    #             if self._previous_source != None:
    #                 # Prioritize the camera the fiducial was last detected in.
    #                 source_name = self._previous_source
    #             else:
    #                 continue
    #         elif self._source_names[i - 1] == self._previous_source:
    #             continue
    #         else:
    #             source_name = self._source_names[i - 1]
    #
    #         if source_name == "hand_color_image":
    #             print('pass the hand_color_image')
    #             continue
    #
    #         img_req = build_image_request(source_name, quality_percent=100,
    #                                       image_format=image_pb2.Image.FORMAT_RAW)
    #         image_response = self._image_client.get_image([img_req])
    #         self._camera_tform_body = get_a_tform_b(image_response[0].shot.transforms_snapshot,
    #                                                 image_response[0].shot.frame_name_image_sensor,
    #                                                 BODY_FRAME_NAME)
    #         self._body_tform_world = get_a_tform_b(image_response[0].shot.transforms_snapshot,
    #                                                BODY_FRAME_NAME, VISION_FRAME_NAME)
    #
    #         # Camera intrinsics for the given source camera.
    #         self._intrinsics = image_response[0].source.pinhole.intrinsics
    #         width = image_response[0].shot.image.cols
    #         height = image_response[0].shot.image.rows
    #
    #         # detect given fiducial in image and return the bounding box of it
    #         bboxes = self.detect_fiducial_in_image(image_response[0].shot.image, (width, height),
    #                                                source_name)
    #         if bboxes:
    #             print("Found bounding box for " + str(source_name))
    #             return bboxes, source_name
    #         else:
    #             self._tag_not_located = True
    #             print("Failed to find bounding box for " + str(source_name))
    #     return [], None
    #
    # def detect_fiducial_in_image(self, image, dim, source_name):
    #     """Detect the fiducial within a single image and return its bounding box."""
    #     image_grey = np.array(
    #         Image.frombytes('P', (int(dim[0]), int(dim[1])), data=image.data, decoder_name='raw'))
    #
    #     # Rotate each image such that it is upright
    #     image_grey = self.rotate_image(image_grey, source_name)
    #
    #     # Make the image greyscale to use bounding box detections
    #     # detector = apriltag(family="tag36h11")
    #     # detections = detector.detect(image_grey)
    #
    #     DETECTOR = Detector(families='tag36h11')
    #     detections = DETECTOR.detect(image_grey, estimate_tag_pose=False, camera_params=None, tag_size=None)
    #
    #     image_grey = cv2.cvtColor(image_grey, cv2.COLOR_GRAY2RGB)
    #     bboxes = []
    #     for r in detections:
    #         # Draw the bounding box detection in the image.
    #         # extract the bounding box (x, y)-coordinates for the AprilTag
    #         # and convert each of the (x, y)-coordinate pairs to integers
    #         (ptA, ptB, ptC, ptD) = r.corners
    #         ptB = (int(ptB[0]), int(ptB[1]))
    #         ptC = (int(ptC[0]), int(ptC[1]))
    #         ptD = (int(ptD[0]), int(ptD[1]))
    #         ptA = (int(ptA[0]), int(ptA[1]))
    #         # draw the bounding box of the AprilTag detection
    #         cv2.line(image_grey, ptA, ptB, (0, 255, 0), 1)
    #         cv2.line(image_grey, ptB, ptC, (0, 255, 0), 1)
    #         cv2.line(image_grey, ptC, ptD, (0, 255, 0), 1)
    #         cv2.line(image_grey, ptD, ptA, (0, 255, 0), 1)
    #         # draw the center (x, y)-coordinates of the AprilTag
    #         (cX, cY) = (int(r.center[0]), int(r.center[1]))
    #         cv2.circle(image_grey, (cX, cY), 5, (0, 0, 255), -1)
    #         bboxes.append(r.corners)
    #
    #     self._image[source_name] = image_grey
    #     return bboxes
    #
    # @staticmethod
    # def rotate_image(image, source_name):
    #     """Rotate the image so that it is always displayed upright."""
    #     if source_name == "frontleft_fisheye_image":
    #         image = cv2.rotate(image, rotateCode=0)
    #     elif source_name == "right_fisheye_image":
    #         image = cv2.rotate(image, rotateCode=1)
    #     elif source_name == "frontright_fisheye_image":
    #         image = cv2.rotate(image, rotateCode=0)
    #     return image
    #
    # @staticmethod
    # def bbox_to_image_object_pts(bbox):
    #     """Determine the object points and image points for the bounding box.
    #        The origin in object coordinates = top left corner of the fiducial.
    #        Order both points sets following: (TL,TR, BL, BR)"""
    #     fiducial_height_and_width = 146  # mm
    #     obj_pts = np.array([[0, 0], [fiducial_height_and_width, 0], [0, fiducial_height_and_width],
    #                         [fiducial_height_and_width, fiducial_height_and_width]],
    #                        dtype=np.float32)
    #     # insert a 0 as the third coordinate (xyz)
    #     obj_points = np.insert(obj_pts, 2, 0, axis=1)
    #
    #     # ['lb-rb-rt-lt']
    #     img_pts = np.array([[bbox[3][0], bbox[3][1]], [bbox[2][0], bbox[2][1]],
    #                         [bbox[0][0], bbox[0][1]], [bbox[1][0], bbox[1][1]]], dtype=np.float32)
    #     return obj_points, img_pts
    #
    # @staticmethod
    # def make_camera_matrix(ints):
    #     """Transform the ImageResponse proto intrinsics into a camera matrix."""
    #     camera_matrix = np.array([[ints.focal_length.x, ints.skew.x, ints.principal_point.x],
    #                               [ints.skew.y, ints.focal_length.y, ints.principal_point.y],
    #                               [0, 0, 1]])
    #     return camera_matrix
    #
    # def pixel_coords_to_camera_coords(self, bbox, intrinsics, source_name):
    #     """Compute transformation of 2d pixel coordinates to 3d camera coordinates."""
    #     camera = self.make_camera_matrix(intrinsics)
    #     # Track a triplet of (translation vector, rotation vector, camera source name)
    #     best_bbox = (None, None, source_name)
    #     # The best bounding box is considered the closest to the robot body.
    #     closest_dist = float('inf')
    #     for i in range(len(bbox)):
    #         obj_points, img_points = self.bbox_to_image_object_pts(bbox[i])
    #         if self._camera_to_extrinsics_guess[source_name][0]:
    #             # initialize the position estimate with the previous extrinsics solution
    #             # then iteratively solve for new position
    #             old_rvec, old_tvec = self._camera_to_extrinsics_guess[source_name][1]
    #             _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera, np.zeros((5, 1)),
    #                                          old_rvec, old_tvec, True, cv2.SOLVEPNP_ITERATIVE)
    #         else:
    #             # Determine current extrinsic solution for the tag.
    #             _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera, np.zeros((5, 1)))
    #
    #         # Save extrinsics results to help speed up next attempts to locate bounding box in
    #         # the same camera source.
    #         self._camera_to_extrinsics_guess[source_name] = (True, (rvec, tvec))
    #
    #         dist = math.sqrt(float(tvec[0][0])**2 + float(tvec[1][0])**2 +
    #                          float(tvec[2][0])**2) / 1000.0
    #         if dist < closest_dist:
    #             closest_dist = dist
    #             best_bbox = (tvec, rvec, source_name)
    #
    #     # Flag indicating if the best april tag been found/located
    #     self._tag_not_located = best_bbox[0] is None and best_bbox[1] is None
    #     return best_bbox
    #
