import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import rosbag
from matplotlib.pyplot import MultipleLocator

# print(matplotlib.matplotlib_fname())
# print(matplotlib.get_cachedir())

plt.rcParams['font.sans-serif'] = 'SimHei'
plt.rcParams['font.size']=20
plt.grid(False)
plt.xlabel('时间(s)')
plt.ylabel('角速度(rad/s)')

angulars1 = []
angulars2 = []

bag_file1 = "/home/ros/fantasyplus/xtcar2/src/common/draw_plot_from_rosbag/rosbag/chapter4_teb/maze_staright_before.bag"
bag_data1 = rosbag.Bag(bag_file1, "r")
cmd_vel1 = bag_data1.read_messages('/ackermann_steering_controller/cmd_vel')
for topic, msg, t in cmd_vel1:
    angulars1.append(msg.angular.z)
print('teb len:',len(angulars1))

bag_file2 = "/home/ros/fantasyplus/xtcar2/src/common/draw_plot_from_rosbag/rosbag/chapter4_teb/maze_staright_after.bag"
bag_data2 = rosbag.Bag(bag_file2, "r")
cmd_vel2 = bag_data2.read_messages('/ackermann_steering_controller/cmd_vel')
for topic, msg, t in cmd_vel2:
    angulars2.append(msg.angular.z)

print('mpc len:',len(angulars2))

#专门为maze_Sturn_before准备的
# for i in range(20):
#     angulars2.pop(-1)
# print('mpc len:',len(angulars2))

#以5hz的时间分配x轴
x1 = list(np.arange(0, len(angulars1)*0.2,0.2))
x2 = list(np.arange(0, len(angulars2)*0.2,0.2))

plt.plot(x1, angulars1, color="red",label='teb曲线',linewidth=0.8)
plt.plot(x2, angulars2, color="blue",label='改进teb曲线',linewidth=0.8)
plt.legend()

#x轴的刻度间隔设置为5
x_major_locator=MultipleLocator(5)
ax=plt.gca()
ax.xaxis.set_major_locator(x_major_locator)

plt.show()
