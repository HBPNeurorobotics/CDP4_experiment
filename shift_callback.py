from attention import Saccade
from std_msgs.msg import Empty

@nrp.MapVariable("saccade", initial_value=Saccade(), scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("shift", Topic("/shift", Empty))
@nrp.Neuron2Robot(triggers = "shift")
def shift_callback(t, saccade, shift):
    saccade.value.shift()
