import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from image_geometry import StereoCameraModel

from attention import Saliency, Saccade

tensorflow_path = rospy.get_param("~tensorflow_path", "/opt/tensorflow_venv/lib/python2.7/site-packages")
model_file = rospy.get_param('~saliency_file', '/tmp/model.ckpt')
network_input_height = float(rospy.get_param('~network_input_height', '192'))
network_input_width = float(rospy.get_param('~network_input_width', '256'))
shift_activity = bool(rospy.get_param('~shift_activity', 'True'))

@nrp.MapVariable("saliency", initial_value = Saliency(tensorflow_path, model_file, network_input_height, network_input_width))
@nrp.MapVariable("saccade", initial_value = Saccade(shift_activity))

@nrp.MapVariable("target_pub", initial_value = rospy.Publisher("/saccade_target", Point, queue_size=1))
@nrp.MapVariable("potential_target_pub", initial_value = rospy.Publisher("/saccade_potential_target", Point, queue_size=1))
@nrp.MapVariable("saliency_image_pub", initial_value = rospy.Publisher("/saliency_map_image", Image, queue_size=1))

@nrp.MapVariable("bridge", initial_value=CvBridge())

@nrp.MapVariable("last_time", initial_value=None)

@nrp.MapVariable("points", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapVariable("camera_model", initial_value=StereoCameraModel())
@nrp.MapVariable("camera_info_left", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("camera_info_right", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("disparity_image", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("tfBuffer", initial_value=None)
@nrp.MapVariable("listener", initial_value=None)

@nrp.MapRobotSubscriber("image", Topic("/hollie/camera/left/image_raw", Image))
@nrp.Robot2Neuron(triggers="image", throttling_rate=10.0)
def image_to_saccade(t, saliency, saccade, target_pub, potential_target_pub, saliency_image_pub, bridge, last_time, points, camera_model, camera_info_left, camera_info_right, disparity_image, tfBuffer, listener, image):
    if image.value is None or camera_info_left.value is None or camera_info_right.value is None or disparity_image.value is None:
        return

    if tfBuffer.value is None or listener.value is None:
        import rospy
        import tf2_ros
        tfBuffer.value = tf2_ros.Buffer(rospy.Duration(30))
        listener.value = tf2_ros.TransformListener(tfBuffer.value)

    if last_time.value is None:                                                                                                                                                                                           
        last_time.value = t
    current_time = t
    dt = current_time - last_time.value
    last_time.value = current_time

    image = bridge.value.imgmsg_to_cv2(image.value, "bgr8")

    saliency_map = saliency.value.compute_saliency_map(image)

    # apply curiosity
    import rospy
    camera_model.value.fromCameraInfo(camera_info_left.value, camera_info_right.value)
    disparity_image = bridge.value.imgmsg_to_cv2(disparity_image.value.image)
    for point in points.value:
        rospy.loginfo("drawing point")
        point.header.stamp = rospy.Time.from_sec(t)
        transformed = tfBuffer.value.transform(point, camera_model.value.tfFrame(), timeout=rospy.Duration(0.1))
        point_torso = (-transformed.point.y, -transformed.point.z, transformed.point.x)
        pixel = camera_model.value.project3dToPixel(point_torso)
        x = int(pixel[0][0] * (len(saliency_map[0])/float(camera_info_left.value.width)))
        y = int(pixel[0][1] * (len(saliency_map)/float(camera_info_left.value.height)))
        disparity = camera_model.value.getDisparity(point_torso[2])
        x = x + disparity
        rospy.loginfo("point at %d, %d" % (x, y))
        if x >= 0 and x < len(saliency_map[0]) and y >=0 and y < len(saliency_map):
            from skimage.draw import circle
            rr, cc = circle(y, x, 15)
            rr = filter(lambda x: x >= 0 and x < len(saliency_map), rr)
            cc = filter(lambda x: x >= 0 and x < len(saliency_map[0]), cc)
            saliency_map[rr, cc] = 0.

    (target, is_actual_target, V, M) = saccade.value.compute_saccade_target(saliency_map, dt)

    target = Point(target[0], target[1], target[2])

    potential_target_pub.value.publish(target)

    saliency_map_image = bridge.value.cv2_to_imgmsg(np.uint8(saliency_map * 255.), "mono8")

    saliency_image_pub.value.publish(saliency_map_image)

    if is_actual_target:
        target_pub.value.publish(target)


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
