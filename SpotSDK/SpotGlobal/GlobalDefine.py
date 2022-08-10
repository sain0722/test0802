import logging

from bosdyn.api import gripper_camera_param_pb2
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncPeriodicQuery
from bosdyn.client.lease import Error as LeaseBaseError
from google.protobuf import wrappers_pb2

LOGGER = logging.getLogger()

class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


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

def gripper_open_and_close(func):
    """
        @Temporary decorator
    """
    def wrapper(self):
        self.spot_control.ARM.gripper_open()
        func(self)
        self.spot_control.ARM.gripper_close()
    return wrapper

