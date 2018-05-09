import rospy
from attention import Saccade
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from embodied_attention.srv import Target

@nrp.MapVariable("saccade", initial_value = Saccade(), scope=nrp.GLOBAL)
@nrp.MapVariable("target_pub", initial_value = rospy.Publisher("/saccade_target", Point, queue_size=1))
@nrp.MapVariable("potential_target_pub", initial_value = rospy.Publisher("/saccade_potential_target", Point, queue_size=1))
@nrp.MapVariable("bridge", initial_value=CvBridge())
@nrp.MapVariable("visual_neurons_pub", initial_value = rospy.Publisher("/visual_neurons", Image, queue_size=1))
@nrp.MapVariable("motor_neurons_pub", initial_value = rospy.Publisher("/motor_neurons", Image, queue_size=1))
@nrp.MapVariable("last_time", initial_value = None)
@nrp.MapVariable("hm_proxy", initial_value = rospy.ServiceProxy('/saccade', Target))
@nrp.MapRobotSubscriber("saliency_map", Topic("/saliency_map", Float32MultiArray))
def saliency_to_saccade(t, saccade, target_pub, potential_target_pub, saliency_map, bridge, visual_neurons_pub, motor_neurons_pub, last_time, hm_proxy):
    if saliency_map.value is None:
        return

    if last_time.value is None:
        last_time.value = t
    current_time = t
    dt = current_time - last_time.value
    last_time.value = current_time

    lo = saliency_map.value.layout
    saliency_map_extracted = np.asarray(saliency_map.value.data[lo.data_offset:]).reshape(lo.dim[0].size, lo.dim[1].size)
    (target, is_actual_target, visual_neurons, motor_neurons) = saccade.value.compute_saccade_target(saliency_map_extracted, dt * 1000)
    target = Point(target[0], target[1], target[2])
    potential_target_pub.value.publish(target)
    if is_actual_target:
        target_pub.value.publish(target)
        hm_proxy.value(target)

    visual_neurons_min = visual_neurons.min()
    visual_neurons_max = visual_neurons.max()
    motor_neurons_min = motor_neurons.min()
    motor_neurons_max = motor_neurons.max()

    if visual_neurons_max - visual_neurons_min is not 0 and motor_neurons_max - motor_neurons_min is not 0:
        visual_neurons = (visual_neurons - visual_neurons_min) / (visual_neurons_max - visual_neurons_min)
        motor_neurons = (motor_neurons - motor_neurons_min) / (motor_neurons_max - motor_neurons_min)

        visual_neurons_image = bridge.value.cv2_to_imgmsg(np.uint8(visual_neurons * 255.), "mono8")
        motor_neurons_image = bridge.value.cv2_to_imgmsg(np.uint8(motor_neurons * 255.), "mono8")
        visual_neurons_pub.value.publish(visual_neurons_image)
        motor_neurons_pub.value.publish(motor_neurons_image)

    return
