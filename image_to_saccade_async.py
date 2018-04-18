import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from attention import Saliency
from std_msgs.msg import Float32MultiArray

tensorflow_path = rospy.get_param("~tensorflow_path", "/opt/tensorflow_venv/lib/python2.7/site-packages")
model_file = rospy.get_param('~saliency_file', '/tmp/model.ckpt')
network_input_height = float(rospy.get_param('~network_input_height', '192'))
network_input_width = float(rospy.get_param('~network_input_width', '256'))

@nrp.MapVariable("saliency", initial_value = Saliency(tensorflow_path, model_file, network_input_height, network_input_width))
@nrp.MapVariable("saliency_pub", initial_value = rospy.Publisher("/saliency_map", Float32MultiArray, queue_size=1))
@nrp.MapVariable("saliency_image_pub", initial_value = rospy.Publisher("/saliency_map_image", Image, queue_size=1))
@nrp.MapVariable("bridge", initial_value=CvBridge())
@nrp.MapRobotSubscriber("image", Topic("/hollie/camera/left/image_raw", Image))
def image_to_saliency(t, saliency, saliency_pub, saliency_image_pub, bridge, image):
    if image.value is None:
        return

    image = bridge.value.imgmsg_to_cv2(image.value, "bgr8")
    saliency_map = saliency.value.compute_saliency_map(image)
    saliency_map_image = bridge.value.cv2_to_imgmsg(np.uint8(saliency_map * 255.), "mono8")
    saliency_image_pub.value.publish(saliency_map_image)

    from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
    height = MultiArrayDimension(size=len(saliency_map))
    width = MultiArrayDimension(size=len(saliency_map[0]))
    lo = MultiArrayLayout([height, width], 0)
    saliency_pub.value.publish(Float32MultiArray(layout=lo, data=saliency_map.flatten()))

import rospy
from attention import Saccade
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

@nrp.MapVariable("saccade", initial_value = Saccade(), scope=nrp.GLOBAL)
@nrp.MapVariable("target_pub", initial_value = rospy.Publisher("/saccade_target", Point, queue_size=1))
@nrp.MapVariable("potential_target_pub", initial_value = rospy.Publisher("/saccade_potential_target", Point, queue_size=1))
@nrp.MapVariable("bridge", initial_value=CvBridge())
@nrp.MapVariable("visual_neurons_pub", initial_value = rospy.Publisher("/visual_neurons", Image, queue_size=1))
@nrp.MapVariable("motor_neurons_pub", initial_value = rospy.Publisher("/motor_neurons", Image, queue_size=1))
@nrp.MapRobotSubscriber("saliency_map", Topic("/saliency_map", Float32MultiArray))
@nrp.MapRobotSubscriber("clock", Topic("/clock", Clock))
@nrp.Neuron2Robot(triggers = "clock", throttling_rate = 1000)
def saliency_to_saccade(t, saccade, target_pub, potential_target_pub, saliency_map, bridge, visual_neurons_pub, motor_neurons_pub, clock):
    if saliency_map.value is None:
        return

    lo = saliency_map.value.layout
    saliency_map_extracted = np.asarray(saliency_map.value.data[lo.data_offset:]).reshape(lo.dim[0].size, lo.dim[1].size)
    (target, is_actual_target, V, M) = saccade.value.compute_saccade_target(saliency_map_extracted, 1)
    target = Point(target[0], target[1], target[2])
    potential_target_pub.value.publish(target)
    if is_actual_target:
        target_pub.value.publish(target)

    V = (V - V.min()) / (V.max() - V.min())                                                                                                                                                                          
    M = (M - M.min()) / (M.max() - M.min())
    
    visual_neurons_image = bridge.value.cv2_to_imgmsg(np.uint8(V * 255.), "mono8")
    motor_neurons_image = bridge.value.cv2_to_imgmsg(np.uint8(M * 255.), "mono8")
    visual_neurons_pub.value.publish(visual_neurons_image)
    motor_neurons_pub.value.publish(motor_neurons_image)

from attention import Saccade
from std_msgs.msg import Empty

@nrp.MapVariable("saccade", initial_value=Saccade(), scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("shift", Topic("/shift", Empty))
@nrp.Neuron2Robot(triggers = "shift")
def shift_callback(t, saccade, shift):
    saccade.value.shift()

from attention import Saccade
from std_msgs.msg import Empty

@nrp.MapVariable("saccade", initial_value=Saccade(), scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("reset", Topic("/reset", Empty))
@nrp.Neuron2Robot(triggers = "reset")
def reset_callback(t, saccade, reset):
    from attention import Saccade
    saccade.value = Saccade()
