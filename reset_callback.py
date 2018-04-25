from attention import Saccade
from std_msgs.msg import Empty

@nrp.MapVariable("saccade", initial_value=Saccade(), scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("reset", Topic("/reset", Empty))
@nrp.Neuron2Robot(triggers = "reset")
def reset_callback(t, saccade, reset):
    from attention import Saccade
    saccade.value = Saccade()
