from std_msgs.msg import Float64 
@nrp.MapVariable("pan", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("pan_new", Topic("/pan", Float64))
@nrp.Robot2Neuron(triggers="pan_new")
def pan_callback(t, pan, pan_new):
    pan.value = pan_new.value
