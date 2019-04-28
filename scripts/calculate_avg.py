import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


def main():
    original_path = "/media/rpl/Data/lsd_results/original.txt"
    mask_path = "/media/rpl/Data/lsd_results/mask.txt"
    original = np.loadtxt(original_path,skiprows=0)
    mask = np.loadtxt(mask_path,skiprows=0)
    print("original: " + str(np.sum(original)/original.shape))
    print("mask: " + str(np.sum(mask)/mask.shape))



if __name__ == '__main__':
    main()