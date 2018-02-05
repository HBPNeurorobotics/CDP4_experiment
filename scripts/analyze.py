import sys, getopt
import rosbag
import matplotlib.pyplot as plt

def plot_rates(bag_file):
    bag = rosbag.Bag(bag_file)
    target_msgs = [msg for msg in bag.read_messages('/saccade_target')]
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    print "Timestamps: " + str(timestamps)
    
    start = timestamps[0]
    stop = timestamps[len(timestamps) - 1]
    normalized_timestamps = [(t - start) for t in timestamps]
    del normalized_timestamps[0]
    print "Start: %f" % start
    print "Stop: %f" % stop
    print "Normalized timestamps: " + str(normalized_timestamps)
    
    rates = [(i+1)/x for i, x in enumerate(normalized_timestamps)]
    print "Rates: " + str(rates)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.title('Saccade rate')
    plt.xlabel('seconds')
    plt.ylabel('#saccades/seconds')
    plt.grid(True)
    plt.plot(normalized_timestamps, rates)
    plt.savefig(bag_file.split(".")[0] + "_rates.png", dpi=150)
    plt.show()

def plot_targets(bag_file):
    bag = rosbag.Bag(bag_file)
    pan_values = [msg.message.data for msg in bag.read_messages('/robot/left_eye_pan/pos')]
    tilt_values = [msg.message.data for msg in bag.read_messages('/robot/eye_tilt/pos')]
    print "Pan values: " + str(pan_values)
    print "Tilt values: " + str(tilt_values)

    if 0. not in pan_values or 0. not in tilt_values:
        print "Error: Couldn't find start"
        return
    start = pan_values.index(0.)
    if start is not tilt_values.index(0.):
        print "Error: Starts do not match"
        return
    print "Start: %i" % start

    pan_values = pan_values[start:]
    tilt_values = tilt_values[start:]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.title('Saccade targets')
    plt.xlabel('pan')
    plt.ylabel('tilt')
    plt.grid(True)
    plt.plot(pan_values, tilt_values)
    for i, xy in enumerate(zip(pan_values, tilt_values)):
        ax.annotate(i, xy=xy, textcoords='data')
    plt.savefig(bag_file.split(".")[0] + "_targets.png", dpi=fig.dpi)
    plt.show()

def list_labels(bag_file):
    bag = rosbag.Bag(bag_file)
    labels = [msg.message.data for msg in bag.read_messages('/label')]
    print labels

def main(argv):
    bag = ''
    cmd = ''
    try:
        opts, args = getopt.getopt(argv,"hb:c:",["bag=", "cmd="])
    except getopt.GetoptError:
        print 'test.py -b <bagfile> -c <cmd>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -b <bagfile> -c <cmd>'
            sys.exit()
        elif opt in ("-b", "--bag"):
            bag = arg
        elif opt in ("-c", "--cmd"):
            cmd = arg

    if cmd == 'rates':
        plot_rates(bag)
    elif cmd == 'targets':
        plot_targets(bag)
    elif cmd == 'labels':
        list_labels(bag)

if __name__ == "__main__":
   main(sys.argv[1:])
