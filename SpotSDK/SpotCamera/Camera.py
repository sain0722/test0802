import cv2
import numpy as np
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

from SpotSDK.SpotCamera.GripperCameraParameter import GripperCameraParameter

def image_to_opencv(image, auto_rotate=True):
    """Convert an image proto message to an openCV image."""
    num_channels = 1  # Assume a default of 1 byte encodings.
    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        dtype = np.uint16
        extension = ".png"
    else:
        dtype = np.uint8
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            num_channels = 3
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            num_channels = 4
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            num_channels = 1
        elif image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
            num_channels = 1
            dtype = np.uint16
        extension = ".jpg"

    img = np.frombuffer(image.shot.image.data, dtype=dtype)
    if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
        cv_depth = img.reshape(image.shot.image.rows,
                               image.shot.image.cols)

        # Visual is a JPEG
        cv_visual = cv2.imdecode(np.frombuffer(image.shot.image.data, dtype=np.uint8), -1)

        # Convert the visual image from a single channel to RGB so we can add color
        # visual_rgb = cv_visual if len(cv_visual.shape) == 3 else cv2.cvtColor(cv_visual, cv2.COLOR_GRAY2RGB)

        # cv2.applyColorMap() only supports 8-bit; convert from 16-bit to 8-bit and do scaling
        min_val = np.min(cv_depth)
        max_val = np.max(cv_depth)
        depth_range = max_val - min_val
        try:
            depth8 = (255.0 / depth_range * (cv_depth - min_val)).astype('uint8')
        except RuntimeWarning:
            print("image 없음")
            # os._exit(1)
        depth8_rgb = cv2.cvtColor(depth8, cv2.COLOR_GRAY2RGB)
        depth_color = cv2.applyColorMap(depth8_rgb, cv2.COLORMAP_JET)
        # Add the two images together.
        # out = cv2.addWeighted(visual_rgb, 0.5, depth_color, 0.5, 0)

        if auto_rotate:
            # out = ndimage.rotate(depth_color, ROTATION_ANGLE[image.source.name])
            if image.source.name[0:5] == "front":
                depth_color = cv2.rotate(depth_color, cv2.ROTATE_90_CLOCKWISE)

            elif image.source.name[0:5] == "right":
                depth_color = cv2.rotate(depth_color, cv2.ROTATE_180)
        # pixel_format = image.shot.image.pixel_format

        return depth_color, extension

    if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
        try:
            # Attempt to reshape array into a RGB rows X cols shape.
            img = img.reshape((image.shot.image.rows, image.shot.image.cols, num_channels))
        except ValueError:
            # Unable to reshape the image data, trying a regular decode.
            img = cv2.imdecode(img, -1)
    else:
        img = cv2.imdecode(img, -1)

    if auto_rotate:
        # img = ndimage.rotate(img, ROTATION_ANGLE[image.source.name])
        if image.source.name[0:5] == "front":
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        elif image.source.name[0:5] == "right":
            img = cv2.rotate(img, cv2.ROTATE_180)
    # pixel_format = image.shot.image.pixel_format
    if len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    else:
        print("exception")
        print(img.shape)
    return img, extension

class Camera:

    def __init__(self, robot):
        self.image_client = robot.ensure_client(ImageClient.default_service_name)
        self.ParameterManager = GripperCameraParameter(robot)
        self.video_mode = False

    def take_image(self):
        source_name = 'hand_color_image'
        image = self.image_client.get_image_from_sources([source_name])
        image, _ = image_to_opencv(image[0], auto_rotate=True)
        return image

    def take_image_from_source(self, camera_name):
        image_client = self.image_client
        source_name = camera_name
        image_sources = image_client.list_image_sources()
        source = [source for source in image_sources if source.name == source_name]
        pixel_format = source[0].pixel_formats[0]
        image_request = [
            build_image_request(source_name, pixel_format=pixel_format)
            # for source in image_sources if source.name == source_name
        ]
        image_responses = image_client.get_image(image_request)
        image, _ = image_to_opencv(image_responses[0], auto_rotate=True)
        return image
