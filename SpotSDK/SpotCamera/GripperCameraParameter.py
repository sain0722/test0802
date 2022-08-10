from bosdyn.api import gripper_camera_param_pb2
from bosdyn.client.gripper_camera_param import GripperCameraParamClient
from google.protobuf import wrappers_pb2


class GripperCameraParameter:

    def __init__(self, robot):
        self.gripper_camera_param_client = robot.ensure_client(GripperCameraParamClient.default_service_name)
        self.request = gripper_camera_param_pb2.GripperCameraGetParamRequest()

    def get_gripper_params(self):
        response = self.gripper_camera_param_client.get_camera_params(self.request)
        return response.params

    def set_gripper_params(self, s_resolution, f_brightness, f_contrast, f_saturation, f_gain,
                           b_exposure_auto, f_exposure_absolute, b_focus_auto, f_focus_absolute, s_hdr):

        gn_camera_mode = self.set_resolution(s_resolution, b_flag=False)
        gf_brightness  = self.set_brightness(f_brightness, b_flag=False)
        gf_contrast    = self.set_contrast  (f_contrast,   b_flag=False)
        gf_saturation  = self.set_saturation(f_saturation, b_flag=False)
        gf_gain        = self.set_gain      (f_gain,       b_flag=False)
        gn_hdr         = self.set_hdr       (s_hdr,        b_flag=False)

        gb_exposure_auto, gf_exposure_absolute = self.set_exposure(b_exposure_auto, f_exposure_absolute, b_flag=False)
        gb_focus_auto, gf_focus_absolute       = self.set_focus   (b_focus_auto, f_focus_absolute,       b_flag=False)

        """
            @TODO: Parameter Setting
            - focus, exposure
            - absolute/auto preprocessing
        """
        params = gripper_camera_param_pb2.GripperCameraParams(
            camera_mode=gn_camera_mode,
            brightness=gf_brightness,
            contrast=gf_contrast,
            gain=gf_saturation,
            saturation=gf_gain,
            focus_absolute=gb_exposure_auto,
            focus_auto=gf_exposure_absolute,
            exposure_absolute=gb_focus_auto,
            exposure_auto=gf_focus_absolute,
            hdr=gn_hdr,)
        #     led_mode=led_mode,
        #     led_torch_brightness=led_torch_brightness

        request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
        response = self.gripper_camera_param_client.set_camera_params(request)
        return response

    def get_resolution(self):
        params = self.get_gripper_params()
        return params.camera_mode

    def set_resolution(self, s_resolution, b_flag=True):
        """
        :param s_resolution:
        "640x480", "1280x720", "1920x1080", "3840x2160", "4096x2160", "4208x3120"

        :param b_flag:
        Apply Immediately

        :return:
        // 1280x720 pixels at 60 frames per second in UYVY format
        MODE_1280_720_60FPS_UYVY = 1;

        // 640x480 pixels at 120 frames per second in UYVY format
        // Warning: this frame rate may not be achievable with long exposure times.
        MODE_640_480_120FPS_UYVY = 11;

        // 1920x1080 pixels at 60 frames per second in Motion JPG format
        MODE_1920_1080_60FPS_MJPG = 14;

        // 3840x2160 pixels at 30 frames per second in Motion JPG format
        MODE_3840_2160_30FPS_MJPG = 15;

        // 4208x3120 pixels at 20 frames per second in Motion JPG format
        MODE_4208_3120_20FPS_MJPG = 16;

        // 4096x2160 pixels at 30 frames per second in Motion JPG format
        MODE_4096_2160_30FPS_MJPG = 17;
        """
        gn_camera_mode = None
        if s_resolution == '640x480':
            gn_camera_mode = gripper_camera_param_pb2.GripperCameraParams.MODE_640_480_120FPS_UYVY
        elif s_resolution == '1280x720':
            gn_camera_mode = gripper_camera_param_pb2.GripperCameraParams.MODE_1280_720_60FPS_UYVY
        elif s_resolution == '1920x1080':
            gn_camera_mode = gripper_camera_param_pb2.GripperCameraParams.MODE_1920_1080_60FPS_MJPG
        elif s_resolution == '3840x2160':
            gn_camera_mode = gripper_camera_param_pb2.GripperCameraParams.MODE_3840_2160_30FPS_MJPG
        elif s_resolution == '4096x2160':
            gn_camera_mode = gripper_camera_param_pb2.GripperCameraParams.MODE_4096_2160_30FPS_MJPG
        elif s_resolution == '4208x3120':
            gn_camera_mode = gripper_camera_param_pb2.GripperCameraParams.MODE_4208_3120_20FPS_MJPG

        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(camera_mode=gn_camera_mode)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)

            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gn_camera_mode

    def get_brightness(self):
        params = self.get_gripper_params()
        return params.brightness

    def set_brightness(self, f_brightness, b_flag=True):
        gf_brightness = wrappers_pb2.FloatValue(value=f_brightness)
        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(brightness=gf_brightness)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gf_brightness

    def get_contrast(self):
        params = self.get_gripper_params()
        return params.contrast

    def set_contrast(self, f_contrast, b_flag=True):
        gf_contrast = wrappers_pb2.FloatValue(value=f_contrast)
        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(contrast=gf_contrast)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gf_contrast

    def get_saturation(self):
        params = self.get_gripper_params()
        return params.saturation

    def set_saturation(self, f_saturation, b_flag=True):
        gf_saturation = wrappers_pb2.FloatValue(value=f_saturation)
        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(saturation=gf_saturation)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gf_saturation

    def get_gain(self):
        params = self.get_gripper_params()
        return params.gain

    def set_gain(self, f_gain, b_flag=True):
        gf_gain = wrappers_pb2.FloatValue(value=f_gain)
        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(gain=gf_gain)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gf_gain

    def get_focus(self):
        params = self.get_gripper_params()
        f_focus_absolute = None
        if params.HasField('focus_absolute'):
            b_focus_auto = params.focus_auto
            f_focus_absolute = params.focus_absolute
        else:
            b_focus_auto = params.focus_auto

        return b_focus_auto, f_focus_absolute

    def set_focus(self, b_focus_auto, f_focus_absolute, b_flag=True):
        gb_focus_auto     = wrappers_pb2.BoolValue(value=b_focus_auto)
        gf_focus_absolute = wrappers_pb2.FloatValue(value=f_focus_absolute)
        if b_flag:
            if b_focus_auto:
                params = gripper_camera_param_pb2.GripperCameraParams(focus_auto=gb_focus_auto)
            else:
                params = gripper_camera_param_pb2.GripperCameraParams(focus_auto=gb_focus_auto,
                                                                      focus_absolute=gf_focus_absolute)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            if b_focus_auto:
                return gb_focus_auto
            else:
                return gb_focus_auto, gf_focus_absolute

    def get_exposure(self):
        params = self.get_gripper_params()
        # b_exposure_auto = None
        f_exposure_absolute = None
        if params.HasField('exposure_absolute'):
            b_exposure_auto = params.exposure_auto
            f_exposure_absolute = params.exposure_absolute
        else:
            b_exposure_auto = params.focus_auto

        return b_exposure_auto, f_exposure_absolute

    def set_exposure(self, b_exposure_auto, f_exposure_absolute, b_flag=True):
        gb_exposure_auto     = wrappers_pb2.BoolValue(value=b_exposure_auto)
        gf_exposure_absolute = wrappers_pb2.FloatValue(value=f_exposure_absolute)
        if b_flag:
            if b_exposure_auto:
                params = gripper_camera_param_pb2.GripperCameraParams(exposure_auto=gb_exposure_auto)
            else:
                params = gripper_camera_param_pb2.GripperCameraParams(exposure_auto=gb_exposure_auto,
                                                                      exposure_absolute=gf_exposure_absolute)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)
            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            if b_exposure_auto:
                return gb_exposure_auto
            else:
                return gb_exposure_auto, gf_exposure_absolute

    def get_hdr(self):
        params = self.get_gripper_params()
        return params.hdr

    def set_hdr(self, s_hdr_mode, b_flag):
        """
        :@param
            s_hdr_mode:
                'OFF', 'Auto', 'Low', 'Med', 'High', 'Max'
            b_flag:
                True : Accept Directly
                False: return value
        :return:
            HDR_UNKNOWN  = 0;  // (or not set): will not change HDR settings.
            HDR_OFF      = 1;  // HDR disabled
            HDR_AUTO     = 2;  // Camera's on-board processor determines how much HDR is needed
            HDR_MANUAL_1 = 3;  // Manual HDR enabled (minimum)
            HDR_MANUAL_2 = 4;  //
            HDR_MANUAL_3 = 5;  //
            HDR_MANUAL_4 = 6;  // Manual HDR enabled (maximum)

        """
        gn_hdr = None
        if s_hdr_mode == 'OFF':
            gn_hdr = gripper_camera_param_pb2.HDR_OFF
        elif s_hdr_mode == 'Auto':
            gn_hdr = gripper_camera_param_pb2.HDR_AUTO
        elif s_hdr_mode == 'Low':
            gn_hdr = gripper_camera_param_pb2.HDR_MANUAL_1
        elif s_hdr_mode == 'Med':
            gn_hdr = gripper_camera_param_pb2.HDR_MANUAL_2
        elif s_hdr_mode == 'High':
            gn_hdr = gripper_camera_param_pb2.HDR_MANUAL_3
        elif s_hdr_mode == 'Max':
            gn_hdr = gripper_camera_param_pb2.HDR_MANUAL_4

        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(hdr=gn_hdr)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)

            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gn_hdr

    def get_led_mode(self):
        params = self.get_gripper_params()
        return params.led_mode

    def set_led_mode(self, s_led_mode, b_flag):
        gn_led_mode = None
        if s_led_mode == 'OFF':
            gn_led_mode = gripper_camera_param_pb2.GripperCameraParams.LED_MODE_OFF
        elif s_led_mode == 'TORCH':
            gn_led_mode = gripper_camera_param_pb2.GripperCameraParams.LED_MODE_TORCH

        if b_flag:
            params = gripper_camera_param_pb2.GripperCameraParams(led_mode=gn_led_mode)
            request = gripper_camera_param_pb2.GripperCameraParamRequest(params=params)

            response = self.gripper_camera_param_client.set_camera_params(request)
            return response
        else:
            return gn_led_mode

    def get_led_torch_brightness(self):
        params = self.get_gripper_params()
        return params.led_mode
