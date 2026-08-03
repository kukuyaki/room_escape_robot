import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

al = 0
point = [[1, 2, 3],[2,2,3]]
for jj in point:
    al += 1/len(point)
    ax.scatter(jj[0], jj[1], jj[2], color='red',alpha=al, s=100, label='Target Point')


x = np.linspace(0, 5, 100)
y = np.linspace(0, 5, 100)
xx, yy = np.meshgrid(x, y)

zz = np.full_like(xx, 8.0)
ax.plot_surface(xx, yy, zz, color='cyan', alpha=0.5, label='Z = 3 Plane')
zz = np.full_like(xx, 12.0)
ax.plot_surface(xx, yy, zz, color='cyan', alpha=0.5, label='Z = 3 Plane')

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_zlim(0, 10)

ax.legend()
plt.show()