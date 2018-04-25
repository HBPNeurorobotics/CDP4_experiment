from stereo_msgs.msg import DisparityImage
@nrp.MapVariable("disparity_image", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("disparity_image_new", Topic("/hollie/camera/disparity", DisparityImage))
@nrp.Robot2Neuron(triggers="disparity_image_new")
def disparity_image_callback(t, disparity_image_new, disparity_image):
    disparity_image.value = disparity_image_new.value
