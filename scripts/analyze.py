import sys, getopt
import rosbag
import matplotlib.pyplot as plt
from scipy.misc import imread
import math
import numpy as np
import os

def split_bag(bag):
    print '### Splitting ###'
    status_msgs = [msg for msg in bag.read_messages('/status') if msg.message.data == 'reset']
    timestamps = [t.timestamp for t in status_msgs]
    if len(timestamps) == 0:
        print 'No status message found'
        return
    print 'Going to split at: ' + str(timestamps)

    bags = [rosbag.Bag(bag.filename.split('.')[0] + ':0.bag', 'w')]
    i = 0
    for topic, msg, t in bag.read_messages():
        if i < len(timestamps) and t >= timestamps[i]:
            i = i + 1
            bags.append(rosbag.Bag(bag.filename.split('.')[0] + ':' + str(i) + '.bag', 'w'))
        bags[i].write(topic, msg, t)

    for b in bags:
        b.close()
    print

def rates(bag, plot):
    print '### Rates ###'
    target_msgs = [msg for msg in bag.read_messages('/saccade_target')]
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    if len(timestamps) == 0:
        print 'No target message found'
        return

    print "Time to first saccade execution: %f" % timestamps[0]
    print "Number of fixations: %d" % len(timestamps)
    
    start = timestamps[0]
    stop = timestamps[len(timestamps) - 1]
    normalized_timestamps = [(t - start) for t in timestamps]
    del normalized_timestamps[0]
    
    rates = [(i+1)/x for i, x in enumerate(normalized_timestamps)]
    print "Rates: " + str(rates)
    
    fig = plt.figure(figsize=(12.8, 12.8))
    ax = fig.add_subplot(111)
    plt.title('Saccade rates')
    plt.xlabel('seconds')
    plt.ylabel('rate (saccades/second)')
    plt.grid(True)
    plt.plot(normalized_timestamps, rates)
    plt.savefig(bag.filename.split(".")[0] + "_rates.png", dpi=150)
    if plot:
        plt.show()
    print

def durations(bag, plot):
    print '### Fixation durations ###'
    target_msgs = [msg for msg in bag.read_messages('/saccade_target')]
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    if len(timestamps) == 0:
        print 'No target message found'
        return

    start = timestamps[0]
    stop = timestamps[len(timestamps) - 1]
    normalized_timestamps = [(t - start) for t in timestamps]
    durations = [j-i for i, j in zip(normalized_timestamps[:-1], normalized_timestamps[1:])]
    print "Fixation durations by ordinal fixation number: " + str(durations)

    duration_avgs = map(lambda (i, x): sum(durations[0:i+1])/(i+1), enumerate(durations))

    print "Average fixation duration: " + str(duration_avgs[len(duration_avgs) - 1])

    fig = plt.figure(figsize=(12.8, 12.8))
    ax = fig.add_subplot(111)
    plt.title('(Average) Fixation durations by ordinal fixation number')
    plt.xlabel('fixation #')
    plt.ylabel('duration (seconds)')
    plt.grid(True)
    plt.plot(durations, label='duration')
    plt.plot(duration_avgs, label='average duration')
    plt.legend()
    plt.savefig(bag.filename.split(".")[0] + "_durations.png", dpi=150)
    if plot:
        plt.show()

    fig2 = plt.figure(figsize=(12.8, 12.8))
    ax2 = fig2.add_subplot(111)
    plt.title('Fixation duration distribution')
    plt.xlabel('duration (seconds)')
    plt.ylabel('number of fixations')
    plt.grid(True)
    plt.hist(durations, bins='doane')
    plt.savefig(bag.filename.split(".")[0] + "_duration_distribution.png", dpi=150)
    if plot:
        plt.show()
    print

def targets(bag, plot):
    print '### Targets ###'
    pan_values = [msg.message.data for msg in bag.read_messages('/pan')]
    tilt_values = [msg.message.data for msg in bag.read_messages('/tilt')]
    if len(pan_values) == 0 or len(tilt_values) == 0:
        print 'No target message found'
        return
    print "Pan values: " + str(pan_values)
    print "Tilt values: " + str(tilt_values)

    fov = [msg.message for msg in bag.read_messages('/fov')]
    x_limits = [i.x for i in fov]
    y_limits = [i.y for i in fov]

    fig = plt.figure(figsize=(12.8, 12.8))
    ax = fig.add_subplot(111)
    img = imread(os.path.expanduser('~/.ros/test/panorama.png'))
    plt.imshow(img)
    plt.title('Saccade targets')
    plt.xlabel('pan (rad)')
    plt.ylabel('tilt (rad)')

    plt.xticks([0, 640, 1280], [-math.pi/2, 0, math.pi/2])
    plt.yticks([0, 640, 1280], [-math.pi/2, 0, math.pi/2])

    x_limits = map(lambda x: (x + math.pi/2) / math.pi * len(img[0]), x_limits)
    y_limits = map(lambda x: (x + math.pi/2) / math.pi * len(img), y_limits)
    plt.plot(x_limits, y_limits, 'r.')
    pan_values = map(lambda x: (x + math.pi/2) / math.pi * len(img[0]), pan_values)
    tilt_values = map(lambda x: (x + math.pi/2) / math.pi * len(img), tilt_values)
    plt.plot(pan_values, tilt_values)
    for i, xy in enumerate(zip(pan_values, tilt_values)):
        ax.annotate(i, xy=xy, textcoords='data')
    plt.savefig(bag.filename.split(".")[0] + "_targets.png", dpi=fig.dpi)
    if plot:
        plt.show()
    print

def amplitudes(bag, plot):
    print '### Saccade amplitudes ###'
    pan_values = [msg.message.data for msg in bag.read_messages('/pan')]
    tilt_values = [msg.message.data for msg in bag.read_messages('/tilt')]
    if len(pan_values) == 0 or len(tilt_values) == 0:
        print 'No target message found'
        return

    pan_amplitudes = [j-i for i, j in zip(pan_values[:-1], pan_values[1:])]
    tilt_amplitudes = [j-i for i, j in zip(tilt_values[:-1], tilt_values[1:])]

    amplitudes = map(lambda (x,y): math.sqrt(x*x + y*y), zip(pan_amplitudes, tilt_amplitudes))
    print "Saccade amplitudes: " + str(amplitudes)

    total_amplitude = sum(amplitudes)
    print "Total saccade amplitude: %f" % total_amplitude

    average_amplitude = total_amplitude/len(amplitudes)
    print "Average saccade amplitude: %f" % average_amplitude

    fig = plt.figure(figsize=(12.8, 12.8))
    ax = fig.add_subplot(111)
    plt.title('Saccade amplitudes by ordinal fixation number')
    plt.xlabel('fixation #')
    plt.ylabel('amplitude (rad)')
    plt.grid(True)
    plt.plot(amplitudes)
    plt.savefig(bag.filename.split(".")[0] + "_amplitudes.png", dpi=fig.dpi)
    if plot:
        plt.show()

    fig2 = plt.figure(figsize=(12.8, 12.8))
    ax2 = fig2.add_subplot(111)
    plt.title('Saccade amplitude distribution')
    plt.xlabel('amplitude (rad)')
    plt.ylabel('number of saccades')
    plt.grid(True)
    plt.hist(amplitudes, bins='doane')
    plt.savefig(bag.filename.split(".")[0] + "_amplitude_distribution.png", dpi=150)
    if plot:
        plt.show()
    print

def list_labels(bag):
    print '### Labels ###'
    labels = [msg.message.data for msg in bag.read_messages('/label')]
    print labels
    print

def main(argv):
    cmd = ''
    plot = False
    try:
        opts, args = getopt.getopt(argv,"hpb:c:",["bag=", "cmd="])
    except getopt.GetoptError:
        print 'test.py -b <bagfile> -c <cmd>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -b <bagfile> -c <cmd>'
            sys.exit()
        elif opt == '-p':
            plot = True
        elif opt in ("-b", "--bag"):
            bag_file = arg
        elif opt in ("-c", "--cmd"):
            cmd = arg

    bag = rosbag.Bag(bag_file)

    if cmd == 'split':
        split_bag(bag)
    elif cmd == 'rates':
        rates(bag, plot)
    elif cmd == 'durations':
        durations(bag, plot)
    elif cmd == 'targets':
        targets(bag, plot)
    elif cmd == 'amplitudes':
        amplitudes(bag, plot)
    elif cmd == 'labels':
        list_labels(bag)
    elif cmd == 'all':
        rates(bag, plot)
        durations(bag, plot)
        targets(bag, plot)
        amplitudes(bag, plot)
        list_labels(bag)
    else:
        print 'Command not found'
        print 'Commands: rates, durations, targets, amplitudes, labels'

if __name__ == "__main__":
   main(sys.argv[1:])
