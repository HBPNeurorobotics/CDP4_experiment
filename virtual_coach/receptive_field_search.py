# disable global logging from the virtual coach
import logging
from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach
import rospy
import time
import os
import argparse
import numpy as np
import timeit
import itertools
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
# logging.disable(logging.INFO)
logging.getLogger('rospy').propagate = False
logging.getLogger('rosout').propagate = False
np.set_printoptions(precision=4, suppress=True)


parser = argparse.ArgumentParser(description='Run a number of trial and store data.')
parser.add_argument('--duration', type=float, default=1.,
                    help='duration of a trial')
parser.add_argument('--n-amp', type=int, default=10,
                    help='number of amplitudes to try')
parser.add_argument('--n-std', type=int, default=10,
                    help='number of standard deviations to try')

parser.add_argument('--min-amp', type=float, default=0.002,
                    help='minimum amplitude of the gaussian receptive fields')
parser.add_argument('--max-amp', type=float, default=0.2,
                    help='maximum amplitude of the gaussian receptive fields')
parser.add_argument('--min-std', type=float, default=0.025,
                    help='minimum standard deviation of the gaussian receptive fields')
parser.add_argument('--max-std', type=float, default=2.5,
                    help='maximum standard deviation of the gaussian receptive fields')


args = parser.parse_args()
rf_stds = np.linspace(args.min_std, args.max_std, args.n_std)
rf_amps = np.linspace(args.min_amp, args.max_amp, args.n_amp)

vc = VirtualCoach(environment='local', storage_username='nrpuser')
fname_base = "/home/nrpuser/.ros/cdp4/experiment"

saccade_tf_base=open("../saliency_to_saccade.py", "r").read()
def get_saccade_tf(amp, std):
    return saccade_tf_base.replace("Saccade()",
                                   "Saccade(amp_rf={}, sig_rf={})".format(amp, std),
                                   1)

def setup(sim, amp, std):
    sim.edit_transfer_function('saliency_to_saccade',
                               get_saccade_tf(amp, std))

start = timeit.default_timer()
last_tick = start
np.save(fname_base + "_config.npy", {
    'rf_stds': rf_stds,
    'rf_amps': rf_amps,
    'duration': args.duration,
})

for i, (amp, std) in enumerate(itertools.product(rf_amps, rf_stds)):
    logger.info("Running experiment {} with amp: {} std: {}".format(i, amp, std))
    sim = vc.launch_experiment('CDP4_experiment_0')
    sim._Simulation__logger.setLevel(logging.WARNING)
    setup(sim, amp, std)
    logger.info("Starting simulation")
    np.random.seed(10)
    sim.start()
    while rospy.get_time() <= args.duration:
        time.sleep(2)
    logger.info("Stoping simulation")
    sim.pause()
    sim.stop()
    while os.path.isfile(fname_base + ".bag.active"):
        time.sleep(2)
    new_bag_name = fname_base + "_" + str(i) + ".bag"
    logger.info("Renaming and saving to {}".format(new_bag_name))
    os.rename(fname_base + ".bag", new_bag_name)
    np.save(fname_base + "_" + str(i)+ ".npy", {
        'amp': amp,
        'std': std
    })
    now = timeit.default_timer()
    logger.info("Trial took {}s".format(now - last_tick))
    last_tick = now
    time.sleep(2)

stop = timeit.default_timer()
logger.info("### Execution took {}s".format(stop - start))
