from sensor_msgs.msg import JointState
@nrp.MapVariable("pan", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapVariable("tilt", initial_value=0, scope=nrp.GLOBAL)
@nrp.MapRobotSubscriber("joint_state", Topic("/joint_states", JointState))
@nrp.Robot2Neuron(triggers="joint_state")
def joint_states_callback(t, pan, tilt, joint_state):
    pan.value = joint_state.value.position[joint_state.value.name.index("hollie_left_eye_pan_joint")] - joint_state.value.position[joint_state.value.name.index("hollie_neck_yaw_joint")]
    tilt.value = joint_state.value.position[joint_state.value.name.index("hollie_eyes_tilt_joint")] + joint_state.value.position[joint_state.value.name.index("hollie_neck_pitch_joint")]
