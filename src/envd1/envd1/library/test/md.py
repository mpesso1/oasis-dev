import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers


np.random.seed(19680801)

x1 = np.linspace(0,10,100)
x2 = pow(x1,2)

c1 = 1
c2 = 1
b = 0

e = np.random.random(100)*2

y = c1*x1 + c2*pow(x1,2) + b

def plot_data(x1,x2,y,e=0):
    fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')
    ax = fig.add_subplot(projection='3d')



    ax.scatter(x1+e,x2+e,y+e)
    ax.plot(x1,x2,y)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    plt.show()

# plot_data(x1,x2,y, np.random.random(100))





# x1 = x1.reshape(1,100)
e = np.random.random(100)*2
x_train = np.array([x1,x2])
print(x1, ' ', x_train[0,:])
# print([x_train[:,5],x1[5]])
y_train = x_train[:,0] + x_train[:,1]
print(np.shape(y_train))
# plot_data(x_train[:,1], x_train[:,0], y_train)
# plot_data(x1, x2, y)

# e = np.random.random(100)*2
# x_val = np.array([x1+e,x2+e]).reshape((100,2))
# y_val = x1+e + pow(x1,2)+e
# plot_data(x_val[:,0]-e, x_val[:,1]-e, y_val-e)