import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

def test_which_pic_not_in(path_rgb, path_associate):
    rgb_data = np.loadtxt(path_rgb,skiprows=3, usecols = (0,))
    associate_data = np.loadtxt(path_associate,skiprows=0, usecols = (0,))

    for i in range(rgb_data.shape[0]):
        if (rgb_data[i] not in associate_data):
            print(rgb_data[i])
            print(i+1)
    


def main():
    path_rgb = "/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/rgb.txt"
    path_associate = "/media/rpl/Data/rgbd_dataset_freiburg3_sitting_xyz/associate.txt"
    test_which_pic_not_in(path_rgb, path_associate)



if __name__ == '__main__':
    main()