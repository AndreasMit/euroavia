from ClassesAndFunctions import *
import matplotlib.pyplot as plt
import time as time_module

# MODEL DEFINITION --------------------------------------------
# Input Nonlinearity        
x_inp = [-0.4731, 0.2448, 0.419, 0.4531, 0.6003, 0.8796, 1]
y_inp = [-0.039, 0.0438, 0.1095, 0.1460, 0.2493, 0.5134, 0.627248]
# Output Nonlinearity
x_out = [-4.1721, -2.6285, -0.7644, 0.0366, 0.1922, 1.071, 2.2186, 3.0185, 3.0625, 3.6097, 6.8054]#, 15]
y_out = [-3.1008, -2.8004, 0.2749, -0.0683, 0.0711, 5.0916, 14.0317, 22.6874, 22.5085, 25.7928, 49.701]#, 111]

# Define Model
model_lin = ZTranferFunction([1, -0.9605], [1, -2.557, +2.214, -0.6516])
model = HammersteinWiener(model_lin, np.array([x_inp, y_inp]), np.array([x_out, y_out]))

# Define NMPC
HORIZON = 2
USE_FROM_HORIZON = 1
mpc_controller = NMPC(model, horizon=HORIZON, u_init=np.ones(HORIZON)*0.8)
u_found = np.zeros(HORIZON)

# SIMULATION --------------------------------------------
# Time
dt = 0.01
t_final = 2
time = np.linspace(0, t_final, int(t_final/dt))

# Input Commands
U = np.zeros(time.shape)
u_opt = np.zeros(time.shape)
U[:1] = 0
U[30:] = 1

# Y = output of the system
Y = np.zeros(time.shape)

# MAIN WHILE LOOP --------------------------------------------
TARGET = 28

model.burn()

s_t = time_module.time()
p_t = s_t

for k in range(len(time)):
    print(f"Iteration {k+1} of {len(time)} \t ({time[k]:.2f} [s]) \t {(time_module.time() - p_t)*1000:.0f} [ms]")
    p_t = time_module.time()

    # Change target real time
    TARGET = 25 + 10*np.sin(2*np.pi*0.5*time[k])

    if k % USE_FROM_HORIZON == 0:
        u_found = mpc_controller.optimize(target=TARGET)

    # Control Input
    u = u_found[k%HORIZON]
    u_opt[k] = u

    # Model Response
    y = model(u)
    Y[k] = y


e_t = time_module.time()

print(f"Elapsed time: {e_t - s_t} [s]")

# PLOT RESULTS --------------------------------------------
# same plots but in a 2x1 grid
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
# plt.step(time, U, 'r', label='Input')
plt.step(time, u_opt, 'b', label='Optimal Input')

plt.grid()
plt.legend()
plt.ylabel("Throttle [%]")

plt.subplot(2, 1, 2)
plt.step(time, Y, 'r-', label='Output Model')
# horizontal dashed line at TARGET
plt.axhline(y=TARGET, color='k', linestyle='--', alpha=0.5, label='Target')
plt.grid()
plt.legend()
plt.ylabel("Current [A]")
plt.xlabel("Time [s]")



plt.show()

