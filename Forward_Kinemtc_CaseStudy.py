import numpy as np
import matplotlib.pyplot as plt
#panjang : Femur = 65, Tibia = 11
#Sudut a = 40, b = 30

l1=65
l2=11

print("Panjang Femur = 65 dan Tibia = 11")
a=np.deg2rad(40) 
b=np.deg2rad(30)

x0,y0 = 0,0
x1 = l1*np.cos(a)
y1 = l1*np.sin(a)

x = l2*np.cos(a+b) + l1*np.cos(a)
y = l2*np.sin(a+b) + l1*np.sin(a)

print("Hasil x dan y berturut turut: ")
print(x, y)

plt.figure(figsize = (8,8))
plt.plot([x0,x1,x],[y0,y1,y], 'o-', linewidth=3, markersize=8, color='b')
plt.plot(x, y, 'ro', label='End Effector')
plt.text(x1, y1, 'Joint 1', fontsize=10, ha='right')
plt.text(x, y, 'End', fontsize=10, ha='right')

plt.xlim(-6, 6)
plt.ylim(-6, 6)
plt.xlabel('X axis')
plt.ylabel('Y axis')
plt.title('2-Link Planar Robot Forward Kinematics')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()