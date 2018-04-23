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

def general(bag, plot):
    print '### General ###'
    target_msgs = [msg for msg in bag.read_messages('/status')]
    print "Droppped %d saccades:" % len(target_msgs)
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    normalized_timestamps = [(t - bag.get_start_time()) for t in timestamps]
    for t in normalized_timestamps:
        print "\tat " + str(t)

    target_msgs = [msg for msg in bag.read_messages('/saccade_target')]
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    normalized_timestamps = [(t - bag.get_start_time()) for t in timestamps]
    print "Time to first saccade execution: %f" % normalized_timestamps[0]
    print "Number of fixations: %d" % len(normalized_timestamps)

    labels = [msg.message.data for msg in bag.read_messages('/label')]
    print "Labels: %s" % str(labels)
    print

def rates(bag, plot):
    print '### Rates ###'
    target_msgs = [msg for msg in bag.read_messages('/saccade_target')]
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    if len(timestamps) == 0:
        print 'No target message found'
        return

    normalized_timestamps = [(t - bag.get_start_time()) for t in timestamps]
    
    rates = [(i+1)/x for i, x in enumerate(normalized_timestamps)]
    print "Saccade rates: " + str(rates)
    
    fig = plt.figure()
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

    normalized_timestamps = [(t - bag.get_start_time()) for t in timestamps]

    durations = [j-i for i, j in zip(normalized_timestamps[:-1], normalized_timestamps[1:])]
    print "Fixation durations by ordinal fixation number: " + str(durations)

    duration_avgs = map(lambda (i, x): sum(durations[0:i+1])/(i+1), enumerate(durations))
    print "Average fixation duration: " + str(duration_avgs[len(duration_avgs) - 1])

    fig = plt.figure()
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

    fig2 = plt.figure()
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
    pan_values.insert(0, 0)
    tilt_values.insert(0, 0)
    print "Pan values: " + str(pan_values)
    print "Tilt values: " + str(tilt_values)

    x_limits = []
    y_limits = []
    camera_infos = [msg.message for msg in bag.read_messages('/hollie/camera/left/camera_info')]
    camera_info = camera_infos[0]
    from image_geometry import PinholeCameraModel
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    top = map(lambda x: (x, 0), range(0, camera_info.width + 1, 16))
    right = map(lambda y: (camera_info.width, y), range(0, camera_info.height + 1, 16))
    bottom = map(lambda x: (x, camera_info.height), range(camera_info.width + 1, 0, -16))
    left = map(lambda y: (0, y), range(camera_info.height + 1, 0, -16))

    for p in top+right+bottom+left:
         point_eye = camera_model.projectPixelTo3dRay(p)
         point_eye = (1., point_eye[0], -point_eye[1])

         pan_eye = math.atan2(point_eye[1], point_eye[0])
         tilt_eye = math.atan2(-point_eye[2], math.sqrt(math.pow(point_eye[0], 2) + math.pow(point_eye[1], 2)))
         x_limits.append(pan_eye)
         y_limits.append(tilt_eye)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    img = imread(os.path.expanduser('~/.ros/cdp4/panorama.png'))
    plt.imshow(img)
    plt.title('Saccade targets')
    plt.xlabel('pan (rad)')
    plt.ylabel('tilt (rad)')

    plt.xticks([0, 640, 1280], [round(-math.pi/2, 4), 0, round(math.pi/2, 4)])
    plt.yticks([0, 640, 1280], [round(-math.pi/2, 4), 0, round(math.pi/2, 4)])

    x_limits = map(lambda x: (x + math.pi/2) / math.pi * len(img[0]), x_limits)
    y_limits = map(lambda x: (x + math.pi/2) / math.pi * len(img), y_limits)
    plt.plot(x_limits, y_limits, 'r-')

    scalar = 0.8

    pan_values = map(lambda x: (x * scalar + math.pi/2) / math.pi * len(img[0]), pan_values)
    tilt_values = map(lambda x: (x * scalar + math.pi/2) / math.pi * len(img), tilt_values)
    plt.plot(pan_values, tilt_values)

    for i, xy in enumerate(zip(pan_values, tilt_values)):
        ax.annotate(i, xy=xy, textcoords='data')
    plt.savefig(bag.filename.split(".")[0] + "_targets.png", dpi=fig.dpi)
    if plot:
        plt.show()
    print

def rois(bag, plot):
    print '### ROIs ###'
    pan_values = [msg.message.data for msg in bag.read_messages('/pan')]
    tilt_values = [msg.message.data for msg in bag.read_messages('/tilt')]
    rois = [msg.message for msg in bag.read_messages('/roi')]
    from cv_bridge import CvBridge, CvBridgeError
    cv_bridge = CvBridge()
    rois = map(lambda x: cv_bridge.imgmsg_to_cv2(x, "rgb8"), rois)
    background = np.uint8(np.zeros([800,800,3]))
    pan_values = map(lambda x: int((x + math.pi/2) / math.pi * len(background[0])), pan_values)
    tilt_values = map(lambda x: int((x + math.pi/2) / math.pi * len(background)), tilt_values)
    for (i, roi) in enumerate(rois):
        background[tilt_values[i]-25:tilt_values[i]+25, pan_values[i]-25:pan_values[i]+25, :] = roi

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.xticks([0, len(background[0])/2, len(background[0])], [round(-math.pi/2, 4), 0, round(math.pi/2, 4)])
    plt.yticks([0, len(background)/2, len(background)], [round(-math.pi/2, 4), 0, round(math.pi/2, 4)])
    plt.imshow(background)
    plt.savefig(bag.filename.split(".")[0] + "_rois.png", dpi=fig.dpi)
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

    amplitudes = map(lambda (x,y): math.sqrt(x*x + y*y) * (360/(2*math.pi)), zip(pan_amplitudes, tilt_amplitudes))
    print "Saccade amplitudes: " + str(amplitudes)

    total_amplitude = sum(amplitudes)
    print "Total saccade amplitude: %f" % total_amplitude

    average_amplitude = total_amplitude/len(amplitudes)
    print "Average saccade amplitude: %f" % average_amplitude

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.title('Saccade amplitudes by ordinal fixation number')
    plt.xlabel('fixation #')
    plt.ylabel('amplitude (deg)')
    plt.grid(True)
    plt.plot(amplitudes)
    plt.savefig(bag.filename.split(".")[0] + "_amplitudes.png", dpi=fig.dpi)
    if plot:
        plt.show()

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    plt.title('Saccade amplitude distribution')
    plt.xlabel('amplitude (deg)')
    plt.ylabel('number of saccades')
    plt.grid(True)
    plt.hist(amplitudes, bins='doane')
    plt.savefig(bag.filename.split(".")[0] + "_amplitude_distribution.png", dpi=150)
    if plot:
        plt.show()
    print

def main(argv):
    cmd = ''
    plot = False
    try:
        opts, args = getopt.getopt(argv,"hpb:c:",["bag=", "cmd="])
    except getopt.GetoptError:
        print 'analyze.py -b <bagfile> -c <cmd>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'analyze.py -b <bagfile> -c <cmd>'
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
    elif cmd == 'general':
        general(bag, plot)
    elif cmd == 'rates':
        rates(bag, plot)
    elif cmd == 'durations':
        durations(bag, plot)
    elif cmd == 'amplitudes':
        amplitudes(bag, plot)
    elif cmd == 'targets':
        targets(bag, plot)
    elif cmd == 'rois':
        rois(bag, plot)
    elif cmd == 'all':
        general(bag, plot)
        rates(bag, plot)
        durations(bag, plot)
        amplitudes(bag, plot)
        targets(bag, plot)
        rois(bag, plot)
    else:
        print 'Command not found'
        print 'Commands: split, general, rates, durations, amplitudes, targets, rois'

if __name__ == "__main__":
   main(sys.argv[1:])
