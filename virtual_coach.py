from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach
import rospy
import time
import os

vc = VirtualCoach(environment='local')
fname_base = "/home/nrpuser/.ros/cdp4/experiment"

for i in range(0, 10):
    print "### Running experiment %d" % i
    print "Launching"
    sim = vc.launch_experiment('cdp4')
    print "Starting"
    sim.start()
    while rospy.get_time() < 11.1:
        time.sleep(2)
    print "Pausing"
    sim.pause()
    print "Stopping"
    sim.stop()
    while os.path.isfile(fname_base + ".bag.active"):
        time.sleep(2)
    print "Renaming"
    os.rename(fname_base + ".bag", fname_base + "_" + str(i) + ".bag")
