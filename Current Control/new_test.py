import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d


class ZTranferFunction:
    def __init__(self, num, den):
        self.num = num
        self.den = den
        self.last_y_needed = len(den) - 1
        # List in the form [y_1, y_2, y_3, ... y_n-1]
        self.last_y = np.zeros(self.last_y_needed)
        self.last_u_needed = len(num)
        # List in the form [u_1, u_2, u_3, ... u_n]
        self.last_u = np.zeros(self.last_u_needed)
    
    def __call__(self, u):
        # Calculate Response
        y = np.dot(self.num, self.last_u) - np.dot(self.den[1:], self.last_y)
        # Update last_u and last_y
        self.last_y = np.roll(self.last_y, 1)
        self.last_y[0] = y
        self.last_u = np.roll(self.last_u, 1)
        self.last_u[0] = u
        return y


class HammersteinWiener:
    def __init__(self, linearTf: ZTranferFunction, input_nl_mat, output_nl_mat):
        self.linearTf = linearTf
        self.inputNL  = interp1d(input_nl_mat[0, :], input_nl_mat[1, :], kind='linear', fill_value='extrapolate')
        self.outputNL = interp1d(output_nl_mat[0, :], output_nl_mat[1, :], kind='linear', fill_value='extrapolate')

    def __call__(self, u):
        u_nl = self.inputNL(u)
        y_lin = self.linearTf(u_nl)
        y = self.outputNL(y_lin)
        return y

    
model_lin = ZTranferFunction([1, -0.9605], [1, -1.627, 0.8026, -0.1465])
x_inp = [-0.4731, 0.2448, 0.419, 0.4531, 0.6003, 0.8796, 1]
y_inp = [-0.039, 0.0438, 0.1095, 0.1460, 0.2493, 0.5134, 0.627248]
x_out = [-4.1721, -2.6285, -0.7644, 0.0366, 0.1922, 1.071, 2.2186, 3.0185, 3.0625, 3.6097, 6.8054]#, 15]
y_out = [-3.1008, -2.8004, 0.2749, -0.0683, 0.0711, 5.0916, 14.0317, 22.6874, 22.5085, 25.7928, 49.701]#, 111]
model = HammersteinWiener(model_lin, np.array([x_inp, y_inp]), np.array([x_out, y_out]))


U = np.zeros(100)
U[5:] = 1
Y = np.zeros(100)
for k in range(0, len(U)):
    Y[k] = model(U[k])

plt.plot(Y)
plt.grid()
plt.show()