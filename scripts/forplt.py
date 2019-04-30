import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

data = np.load("forplt.npz")
first_xyz = data['x']
second_xyz_full_aligned = data['y']
fig = plt.figure()
ax = plt.axes(projection='3d')
print(second_xyz_full_aligned.shape)
ax.plot3D(first_xyz[0,:].squeeze(), first_xyz[1,:].squeeze(), first_xyz[2,:].squeeze(), 'green')
ax.plot3D(second_xyz_full_aligned[0,:].squeeze(), second_xyz_full_aligned[1,:].squeeze(), second_xyz_full_aligned[2,:].squeeze(), 'red')
plt.show()