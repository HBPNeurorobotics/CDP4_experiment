from sensor_msgs.msg import CameraInfo
@nrp.MapVariable("camera_info_right", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("camera_info", Topic("/hollie/camera/right/camera_info", CameraInfo))
@nrp.Robot2Neuron(triggers="camera_info")
def camera_info_right_callback(t, camera_info, camera_info_right):
    camera_info_right.value = camera_info.value
