import rosbag
import matplotlib.pyplot as plt

BAG_FILE = 'basic_corner.bag'
bag = rosbag.Bag(BAG_FILE)
all_topics = ['pf/pose/odom']

fake_topics = bag.get_type_and_topic_info()[1].keys()
for topic, msg, t in bag.read_messages(topics=all_topics):
    print(msg)

bag.close()
