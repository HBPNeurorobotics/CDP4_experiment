from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach
import rospy
import time
import os

vc = VirtualCoach(environment='local')

for i in range(0,9):
    print "### Running experiment %d" % i
    print "Launching"
    sim = vc.launch_experiment('cdp4')
    print "Starting"
    sim.start()
    while rospy.get_time() < 11.1:
        time.sleep(2)
    print "Stopping"
    sim.stop()
    time.sleep(20)
    print "Renaming"
    os.rename("/home/nrpuser/.ros/cdp4/experiment.bag", "/home/nrpuser/.ros/cdp4/experiment_" + str(i) + ".bag")
