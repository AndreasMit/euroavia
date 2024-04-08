import numpy as np
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
    
    def reset(self):
        self.last_y = np.zeros(self.last_y_needed)
        self.last_u = np.zeros(self.last_u_needed)
    
    def backup(self):
        self.bu_last_y = self.last_y.copy()
        self.bu_last_u = self.last_u.copy()
    
    def restore(self):
        self.last_y = self.bu_last_y.copy()
        self.last_u = self.bu_last_u.copy()
    
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
    
    # When the model starts it jumps for a while. 
    # So we need to reset it and let it run for a couple iterations with 0 input
    def burn(self, center_point=0, num_iterations=20):
        self.linearTf.reset()
        for i in range(num_iterations):
            self(center_point)

# MPC
from scipy.optimize import minimize

class NMPC:
    def __init__(self, model, horizon, u_init):
        self.model = model
        self.horizon = horizon
        self.u_prev = u_init

    def cost_function(self, u):
        # Backup model
        self.model.linearTf.backup()

        # Define cost function here, e.g., tracking error
        resp = np.zeros(u.shape)
        for i in range(len(u)):
            resp[i] = self.model(u[i])
        
        # Restore model
        self.model.linearTf.restore()

        return np.sum((resp - self.target)**2)

    def optimize(self, target=25):

        self.target = target

        bounds = [(0, 1)] * self.horizon  # example bounds for control inputs
        result = minimize(self.cost_function, self.u_prev, bounds=bounds,
                            method='SLSQP', 
                            options={'disp': False, 'maxiter': 5})
        self.u_prev = result.x
        
        return result.x