from turtle import color, distance
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator
import math
from scipy.spatial import distance


def loadFloatTxt(filename):
    data = np.loadtxt(filename, dtype=np.float32, delimiter='\t')
    return data


def loadStringTxt(file_name):
    data = []
    file = open(file_name, 'r')
    file_data = file.readlines()
    for row in file_data:
        tmp_list = row.split(' ')
        for points in tmp_list:
            point = points.split(',')
            data.append(point)
    return data


#该函数用于将经纬度转换为XY坐标值
def blh2xyz(lat, lon):
    lat_org = 32.0242100 * math.pi / 180.0
    lon_org = 118.89928833 * math.pi / 180.0
    lat = lat * math.pi / 180.0
    lon = lon * math.pi / 180.0
    x = (lat - lat_org) * 6378137
    y = (lon - lon_org) * 6378137 * math.cos(lat)
    return x, y


def getEuclidean(p1, p2):
    return math.sqrt(
        math.pow(float(p1[0]) - float(p2[0]), 2) +
        math.pow(float(p1[1]) - float(p2[1]), 2))


def getErr(point, ref_traj):
    min_dis = 0xffffffff
    for ref_point in ref_traj:
        min_dis = min(min_dis, getEuclidean(point, ref_point))
    return min_dis


mpc_x = []
mpc_y = []
ref_x = []
ref_y = []
if __name__ == "__main__":
    mpc_traj = loadStringTxt(
        '/home/ros/fantasyplus/xtcar2/src/common/draw_plot/traj/8_10/google_uwb.txt'
    )
    ref_traj = loadStringTxt(
        '/home/ros/fantasyplus/xtcar2/src/common/draw_plot/traj/8_10/google_ref.txt'
    )

    mpc_poses = []
    for points in mpc_traj:
        pose = blh2xyz(float(points[1]), float(points[0]))
        mpc_poses.append(pose)
    ref_poses = []
    for points in ref_traj:
        pose = blh2xyz(float(points[1]), float(points[0]))
        ref_poses.append(pose)

    err = []
    for p in ref_poses:
        error = getErr(p, mpc_poses)
        if error < 1:
            err.append((error * 100) - 25)
    x = list(np.arange(0, len(err)))

    plt.subplot(1, 2, 1)

    plt.rcParams['font.sans-serif'] = 'SimHei'
    plt.xlabel('轨迹点(个)')
    plt.ylabel('误差(cm)')
    plt.plot(x, err,linewidth=2.0)

    for points in mpc_poses:
        mpc_x.append(points[0])
        mpc_y.append(points[1])
    for points in ref_poses:
        ref_x.append(points[0])
        ref_y.append(points[1])

    plt.subplot(1, 2, 2)

    plt.rcParams['font.sans-serif'] = 'SimHei'
    plt.rcParams['font.size'] = 10
    # plt.gca().xaxis.set_major_locator(MultipleLocator(10))
    plt.xlabel('全局Y坐标(m)')
    plt.ylabel('全局X坐标(m)')
    plt.plot(mpc_y, mpc_x, color='red', label='mpc路径', linewidth=2)
    plt.plot(ref_y, ref_x, color='green', label='参考路径', linewidth=1.0)
    plt.legend()
    plt.show()
