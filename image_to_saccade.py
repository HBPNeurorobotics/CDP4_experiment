import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from attention import Saliency, Saccade

tensorflow_path = rospy.get_param("~tensorflow_path", "/opt/tensorflow_venv/lib/python2.7/site-packages")
model_file = rospy.get_param('~saliency_file', '/tmp/model.ckpt')
network_input_height = float(rospy.get_param('~network_input_height', '192'))
network_input_width = float(rospy.get_param('~network_input_width', '256'))
shift_activity = bool(rospy.get_param('~shift_activity', 'True'))

@nrp.MapVariable("saliency", initial_value = Saliency(tensorflow_path, model_file, network_input_height, network_input_width))
@nrp.MapVariable("saccade", initial_value = Saccade(shift_activity))

@nrp.MapVariable("potential_target_pub", initial_value = rospy.Publisher("/saccade_potential_target", Point, queue_size=1))
@nrp.MapVariable("saliency_image_pub", initial_value = rospy.Publisher("/saliency_map_image", Image, queue_size=1))

@nrp.MapVariable("bridge", initial_value=CvBridge())

@nrp.MapVariable("last_time", initial_value=None)

@nrp.MapRobotSubscriber("image", Topic("/hollie/camera/left/image_raw", Image))
@nrp.Neuron2Robot(Topic('/saccade_target', Point))
def image_to_saccade(t, saliency, saccade, potential_target_pub, saliency_image_pub, bridge, last_time, image):
    import rospy

    if image.value is None:
        return

    if last_time.value is None:                                                                                                                                                                                           
        last_time.value = rospy.get_time()
    current_time = rospy.get_time()
    dt = current_time - last_time.value
    last_time.value = current_time

    image = bridge.value.imgmsg_to_cv2(image.value, "bgr8")

    saliency_map = saliency.value.compute_saliency_map(image)
    (target, is_actual_target, V, M) = saccade.value.compute_saccade_target(saliency_map, dt)

    target = Point(target[0], target[1], target[2])

    potential_target_pub.value.publish(target)

    saliency_map_image = bridge.value.cv2_to_imgmsg(np.uint8(saliency_map * 255.), "mono8")

    saliency_image_pub.value.publish(saliency_map_image)

    if is_actual_target:
        return target
