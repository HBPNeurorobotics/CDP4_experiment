import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from attention import Saliency
from std_msgs.msg import Float32MultiArray
from image_geometry import StereoCameraModel
from embodied_attention.srv import Transform

tensorflow_path = rospy.get_param("tensorflow_path", "/opt/tensorflow_venv/lib/python2.7/site-packages")
model_file = rospy.get_param('saliency_file', '/tmp/model.ckpt')
network_input_height = float(rospy.get_param('network_input_height', '192'))
network_input_width = float(rospy.get_param('network_input_width', '256'))

@nrp.MapVariable("saliency", initial_value = Saliency(tensorflow_path, model_file, network_input_height, network_input_width, False))
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
    # when using tf, wait for the transformations to be spread
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
        x = x + 6 # correction, bug in opencv?
        y = int(pixel[0][1] * (len(saliency_map)/float(camera_info_left.value.height)))
        disparity = camera_model.value.getDisparity(point_torso[2])
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

    return
