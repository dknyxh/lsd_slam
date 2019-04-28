import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def read_lsdslam_poses(path, n):
    data = np.loadtxt(path,skiprows=n)
    return data


def main():
    path = "/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/groundtruth.txt"
    data = read_lsdslam_poses(path, 3)    
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    xline = data[:,1]
    yline = data[:,2]
    zline = data[:,3]
    ax.plot3D(xline, yline, zline, 'green')
    plt.show()


if __name__ == '__main__':
    main()