import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from attention import Saliency
from std_msgs.msg import Float32MultiArray
from image_geometry import PinholeCameraModel
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
@nrp.MapVariable("camera_model", initial_value=PinholeCameraModel())
@nrp.MapVariable("camera_info_left", initial_value=None, scope=nrp.GLOBAL)
@nrp.MapVariable("last_time", initial_value = None)
@nrp.MapVariable("elapsed", initial_value = 0)
@nrp.MapVariable("pan", initial_value = 0, scope=nrp.GLOBAL)
@nrp.MapVariable("tilt", initial_value = 0, scope=nrp.GLOBAL)
@nrp.MapVariable("saliency_map", initial_value = None)
@nrp.MapRobotSubscriber("image", Topic("/hollie/camera/left/image_raw", Image))
def image_to_saliency(t, image, bridge, saliency, saliency_pub, saliency_image_pub, points, camera_model, camera_info_left, last_time, elapsed, pan, tilt, saliency_map):
    if image.value is None or camera_info_left.value is None:
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

    if saliency_map.value is None:
        image = bridge.value.imgmsg_to_cv2(image.value, "bgr8")
        image = np.zeros(image.shape)
        saliency_map.value = saliency.value.compute_saliency_map(image)

    saliency_map_current = saliency_map.value.copy()

    # apply curiosity
    camera_model.value.fromCameraInfo(camera_info_left.value)
    for point in points.value:
        # call service
        pixel = camera_model.value.project3dToPixel((point.point.x - pan.value, point.point.y - tilt.value, point.point.z))
        x = int(pixel[0] * (len(saliency_map_current[0])/float(camera_info_left.value.width)))
        x = x + 6 # correction, bug in opencv?
        y = int(pixel[1] * (len(saliency_map_current)/float(camera_info_left.value.height)))
        if x >= 0 and x < len(saliency_map_current[0]) and y >=0 and y < len(saliency_map_current):
            from skimage.draw import circle
            rr, cc = circle(y, x, 25, (len(saliency_map_current), len(saliency_map_current[0])))
            saliency_map[rr, cc] = saliency_map[rr, cc] * min(1, (t - point.header.stamp.to_sec()))

    saliency_map_image = bridge.value.cv2_to_imgmsg(np.uint8(saliency_map_current * 255.), "mono8")
    saliency_image_pub.value.publish(saliency_map_image)

    from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
    height = MultiArrayDimension(size=len(saliency_map_current))
    width = MultiArrayDimension(size=len(saliency_map_current[0]))
    lo = MultiArrayLayout([height, width], 0)
    saliency_pub.value.publish(Float32MultiArray(layout=lo, data=saliency_map_current.flatten()))

    return
