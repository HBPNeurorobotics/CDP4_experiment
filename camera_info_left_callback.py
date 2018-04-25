from sensor_msgs.msg import CameraInfo
@nrp.MapVariable("camera_info_left", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("camera_info", Topic("/hollie/camera/left/camera_info", CameraInfo))
@nrp.Robot2Neuron(triggers="camera_info")
def camera_info_left_callback(t, camera_info, camera_info_left):
    camera_info_left.value = camera_info.value
