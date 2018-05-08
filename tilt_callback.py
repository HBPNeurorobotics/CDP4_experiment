from std_msgs.msg import Float64 
@nrp.MapVariable("tilt", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("tilt_new", Topic("/tilt", Float64))
@nrp.Robot2Neuron(triggers="tilt_new")
def tilt_callback(t, tilt, tilt_new):
    tilt.value = tilt_new.value
