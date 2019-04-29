import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt



def read_lsdslam_poses(path):
    with open(path, 'rb') as file:
        ba = np.array(bytearray(file.read()))

    F, I, J = ba[:12].view(np.int32)
    assert I == 3 and J == 4, "Not poses?"

    T = ba[12:].view(float).reshape(F,I,J)

    Rs = np.empty((F,3,3))
    ts = np.empty((F,3))
    for f in range(F):
        R = T[f,:,:3]
        s = np.linalg.norm(R[0])
        R /= s
        Rs[f] = R.T
        ts[f] = -np.dot(R.T, T[f,:,3])

    return Rs, ts


def main():
    path = "/media/rpl/Data/lsd_results/lsd_pose.dat"
    Rs, ts = read_lsdslam_poses(path)
    print(ts)
    print(ts.shape)
    print(Rs)
    print(Rs.shape)
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    xline = ts[:,0]
    yline = ts[:,1]
    zline = ts[:,2]
    ax.plot3D(-xline, -zline, yline, 'green')
    plt.show()


if __name__ == '__main__':
    main()