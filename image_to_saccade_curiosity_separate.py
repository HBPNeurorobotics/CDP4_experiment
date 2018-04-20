import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from attention import Saliency
from std_msgs.msg import Float32MultiArray
from image_geometry import StereoCameraModel
from embodied_attention.srv import Transform

tensorflow_path = rospy.get_param("tensorflow_path", "/opt/tensorflow_venv/lib/python2.7/site-packages")
model_file = rospy.get_param('~saliency_file', '/tmp/model.ckpt')
network_input_height = float(rospy.get_param('~network_input_height', '192'))
network_input_width = float(rospy.get_param('~network_input_width', '256'))

@nrp.MapVariable("saliency", initial_value = Saliency(tensorflow_path, model_file, network_input_height, network_input_width))
@nrp.MapVariable("saliency_pub", initial_value = rospy.Publisher("/saliency_map", Float32MultiArray, queue_size=1))
@nrp.MapVariable("saliency_image_pub", initial_value = rospy.Publisher("/saliency_map_image", Image, queue_size=1))
@nrp.MapVariable("bridge", initial_value=CvBridge())
@nrp.MapVariable("points", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapVariable("camera_model", initial_value=StereoCameraModel())
@nrp.MapVariable("camera_info_left", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("camera_info_right", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("disparity_image", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("transform_proxy", initial_value=rospy.ServiceProxy("/transform", Transform))
@nrp.MapVariable("last_time", initial_value = None)
@nrp.MapVariable("elapsed", initial_value = 0)
@nrp.MapRobotSubscriber("image", Topic("/hollie/camera/left/image_raw", Image))
def image_to_saliency(t, image, bridge, saliency, saliency_pub, saliency_image_pub, points, camera_model, camera_info_left, camera_info_right, disparity_image, transform_proxy, last_time, elapsed):
    if t < 1.0:
        return

    if image.value is None or camera_info_left.value is None or camera_info_right.value is None or disparity_image.value is None:
        return

    if last_time.value is None:
        last_time.value = t
    current_time = t
    dt = current_time - last_time.value
    last_time.value = current_time

    elapsed.value = elapsed.value + dt
    if elapsed.value < 0.01:
        return
    else:
        elapsed.value = 0.

    image = bridge.value.imgmsg_to_cv2(image.value, "bgr8")
    saliency_map = saliency.value.compute_saliency_map(image)

    # apply curiosity
    import rospy
    import geometry_msgs
    import tf2_geometry_msgs
    camera_model.value.fromCameraInfo(camera_info_left.value, camera_info_right.value)
    disparity_image = bridge.value.imgmsg_to_cv2(disparity_image.value.image)
    for point in points.value:
        # call service
        point_new = geometry_msgs.msg.PointStamped()
        point_new.header = point.header
        point_new.point = point.point
        transformed = transform_proxy.value(point_new).res
        transformed_new = tf2_geometry_msgs.PointStamped()
        transformed_new.header = transformed.header
        transformed_new.point = transformed.point
        transformed = transformed_new
        point_torso = (-transformed.point.y, -transformed.point.z, transformed.point.x)
        pixel = camera_model.value.project3dToPixel(point_torso)
        x = int(pixel[0][0] * (len(saliency_map[0])/float(camera_info_left.value.width)))
        y = int(pixel[0][1] * (len(saliency_map)/float(camera_info_left.value.height)))
        disparity = camera_model.value.getDisparity(point_torso[2])
        x = x + disparity
        if x >= 0 and x < len(saliency_map[0]) and y >=0 and y < len(saliency_map):
            from skimage.draw import circle
            rr, cc = circle(y, x, 25, (len(saliency_map), len(saliency_map[0])))
            saliency_map[rr, cc] = 0.

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
    if t < 1.0:
        return

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

    visual_neurons = (visual_neurons - visual_neurons.min()) / (visual_neurons.max() - visual_neurons.min())
    motor_neurons = (motor_neurons - motor_neurons.min()) / (motor_neurons.max() - motor_neurons.min())
    
    visual_neurons_image = bridge.value.cv2_to_imgmsg(np.uint8(visual_neurons * 255.), "mono8")
    motor_neurons_image = bridge.value.cv2_to_imgmsg(np.uint8(motor_neurons * 255.), "mono8")
    visual_neurons_pub.value.publish(visual_neurons_image)
    motor_neurons_pub.value.publish(motor_neurons_image)

from tf2_geometry_msgs import PointStamped 
@nrp.MapVariable("points", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("point", Topic("/saccade_point", PointStamped))
@nrp.Robot2Neuron(triggers="point")
def point_callback(t, point, points):
    points.value.append(point.value)

from sensor_msgs.msg import CameraInfo
@nrp.MapVariable("camera_info_left", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("camera_info", Topic("/hollie/camera/left/camera_info", CameraInfo))
@nrp.Robot2Neuron(triggers="camera_info")
def camera_info_left_callback(t, camera_info, camera_info_left):
    camera_info_left.value = camera_info.value

from sensor_msgs.msg import CameraInfo
@nrp.MapVariable("camera_info_right", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("camera_info", Topic("/hollie/camera/right/camera_info", CameraInfo))
@nrp.Robot2Neuron(triggers="camera_info")
def camera_info_right_callback(t, camera_info, camera_info_right):
    camera_info_right.value = camera_info.value

from stereo_msgs.msg import DisparityImage
@nrp.MapVariable("disparity_image", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("disparity_image_new", Topic("/hollie/camera/disparity", DisparityImage))
@nrp.Robot2Neuron(triggers="disparity_image_new")
def disparity_image_callback(t, disparity_image_new, disparity_image):
    disparity_image.value = disparity_image_new.value

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
