import rosbag
import matplotlib.pyplot as plt
import math

BAG_FILE = 'final
.bag'
bag = rosbag.Bag(BAG_FILE)
all_topics = ['error']

dist = []
theta = []
times = []
new_times = []

for topic, msg, time in bag.read_messages(topics=all_topics):
    print(msg)
    if len(msg.data) == 3:
        dist.append(math.sqrt(msg.data[0]**2 + msg.data[1]**2))
        theta.append(msg.data[2])
        times.append(time.to_sec())

for i in range(len(times)):
    new_times.append(times[i] - times[0])

plt.plot(new_times, dist, 'b', label = "euclidean distance (meters)")
plt.plot(new_times, theta, 'r', label = "theta error (radians)")
plt.axis([0,5,0,0.5])
plt.title("Error of Particle Filter in Simulation Using EMA")
plt.xlabel("Time")
plt.ylabel("Error Magnitude")
plt.legend()
plt.show()

bag.close()
