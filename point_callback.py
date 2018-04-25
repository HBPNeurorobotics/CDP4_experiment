from tf2_geometry_msgs import PointStamped 
@nrp.MapVariable("points", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("point", Topic("/saccade_point", PointStamped))
@nrp.Robot2Neuron(triggers="point")
def point_callback(t, point, points):
    points.value.append(point.value)
