import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R



def calculate_rmse(algorithm_path, associate_groundtruth_path):
    with open(algorithm_path, 'rb') as file:
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

    data = np.loadtxt(associate_groundtruth_path,skiprows=0, usecols = (3,4,5))

    return Rs, ts, data


def main():
    algorithm_path = "/media/rpl/Data/lsd_results/lsd_pose.dat"
    associate_groundtruth_path = "/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/associate.txt"
    Rs, ts, data = calculate_rmse(algorithm_path, associate_groundtruth_path)
    # print(Rs.shape) # Rs is a 1261 * 3 * 3 matrix

    # --------- below is to save data to file---------
    rawdata = np.loadtxt(associate_groundtruth_path,skiprows=0, usecols = (0))
    quater = np.zeros((Rs.shape[0], 4))
    for i in range(Rs.shape[0]):
        r = R.from_dcm(np.array(Rs[i,:,:]))
        quater[i,:] = r.as_quat()
    print(rawdata.shape)
    print("\n")
    print(quater[3:,:].shape)
    print("\n")
    print(ts[3:,:].shape)
    print("\n")
    np.savetxt('/home/rpl/ros_workspace/src/lsd_slam/scripts/algorithm_trajectory.txt', np.hstack((np.reshape(rawdata, (-1,1)), np.hstack((ts[3:,:], quater[3:,:])))), fmt='%4f')
    # ------------------------------------------------    

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    x = -ts[3:,0]
    y = -ts[3:,2]
    z = ts[3:,1]

    x_groudtruth = data[:,0] - data[0,0]
    y_groudtruth = data[:,1] - data[0,1]
    z_groudtruth = data[:,2] - data[0,2]

    # rmse = np.sqrt(np.sum((x-x_groudtruth) * (x-x_groudtruth) + (y-y_groudtruth) * (y-y_groudtruth) + (z-z_groudtruth) * (z-z_groudtruth)) / data.shape[0])
    # print("rmse: " + str(rmse))


    # with open("/media/rpl/Data/lsd_results/original.txt", "a") as resultfile:
    #     resultfile.write(str(rmse)+"\n")
    # resultfile.close()




    # ax.plot3D(x, y, z, 'green')
    # ax.plot3D(x_groudtruth, y_groudtruth, z_groudtruth, 'red')
    # plt.show()




if __name__ == '__main__':
    main()
