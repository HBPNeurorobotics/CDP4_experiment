from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach
import rospy
import time
import os
import argparse
import timeit

parser = argparse.ArgumentParser(description='Run a number of trial and store data.')
parser.add_argument('--n', type=int, default=10,
                    help='number of trials')
parser.add_argument('--duration', type=float, default=1.,
                    help='duration of a trial (in second)')

args = parser.parse_args()

vc = VirtualCoach(environment='local', storage_username='nrpuser')
fname_base = "/home/nrpuser/.ros/cdp4/experiment"

def setup(sim):
    pass

start = timeit.default_timer()
for i in range(args.n):
    print "### Running experiment %d" % i
    print "Launching"
    sim = vc.launch_experiment('CDP4_experiment_0')
    setup(sim)
    print "Starting"
    sim.start()
    while rospy.get_time() <= args.duration:
        time.sleep(2)
    print "Pausing"
    sim.pause()
    print "Stopping"
    sim.stop()
    while os.path.isfile(fname_base + ".bag.active"):
        time.sleep(2)
    print "Renaming"
    os.rename(fname_base + ".bag", fname_base + "_" + str(i) + ".bag")

stop = timeit.default_timer()
print("### Execution took {}s".format(stop - start))
