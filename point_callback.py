from geometry_msgs.msg import Point
@nrp.MapVariable("points", initial_value=[], scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("point", Topic("/saccade_point", Point))
@nrp.Robot2Neuron(triggers="point")
def point_callback(t, point, points):
    points.value.append(point.value)
