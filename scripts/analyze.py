import sys, getopt
import rosbag
import matplotlib.pyplot as plt

def split_bag(bag):
    print 'Splitting'
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

def plot_rates(bag, plot):
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
    plt.savefig(bag.filename.split(".")[0] + "_rates.png", dpi=150)
    if plot:
        plt.show()

def plot_durations(bag, plot):
    target_msgs = [msg for msg in bag.read_messages('/saccade_target')]
    timestamps = [t.timestamp.to_sec() for t in target_msgs]
    print "Timestamps: " + str(timestamps)

    start = timestamps[0]
    stop = timestamps[len(timestamps) - 1]
    normalized_timestamps = [(t - start) for t in timestamps]
    durations = [j-i for i, j in zip(normalized_timestamps[:-1], normalized_timestamps[1:])]
    print "Fixation durations: " + str(durations)

    print str(enumerate(durations))
    duration_avgs = map(lambda (i, x): sum(durations[0:i+1])/(i+1), enumerate(durations))
    print "Average fixation durations: " + str(duration_avgs)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.title('(Average) Fixation durations')
    plt.xlabel('saccade #')
    plt.ylabel('seconds')
    plt.grid(True)
    plt.plot(durations, label='duration')
    plt.plot(duration_avgs, label='average duration')
    plt.legend()
    plt.savefig(bag.filename.split(".")[0] + "_durations.png", dpi=150)
    if plot:
        plt.show()

def plot_targets(bag, plot):
    pan_values = [msg.message.data for msg in bag.read_messages('/robot/left_eye_pan/pos')]
    tilt_values = [msg.message.data for msg in bag.read_messages('/robot/eye_tilt/pos')]
    print "Pan values: " + str(pan_values)
    print "Tilt values: " + str(tilt_values)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.title('Saccade targets')
    plt.xlabel('pan')
    plt.ylabel('tilt')
    plt.grid(True)
    plt.plot(pan_values, tilt_values)
    for i, xy in enumerate(zip(pan_values, tilt_values)):
        ax.annotate(i, xy=xy, textcoords='data')
    plt.savefig(bag.filename.split(".")[0] + "_targets.png", dpi=fig.dpi)
    if plot:
        plt.show()

def list_labels(bag):
    labels = [msg.message.data for msg in bag.read_messages('/label')]
    print labels

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
        plot_rates(bag, plot)
    elif cmd == 'durations':
        plot_durations(bag, plot)
    elif cmd == 'targets':
        plot_targets(bag, plot)
    elif cmd == 'labels':
        list_labels(bag)
    elif cmd == 'all':
        plot_rates(bag, plot)
        plot_durations(bag, plot)
        plot_targets(bag, plot)
        list_labels(bag)
    else:
        print 'Command not found'
        print 'Commands: rates, targets, durations, labels'

if __name__ == "__main__":
   main(sys.argv[1:])
