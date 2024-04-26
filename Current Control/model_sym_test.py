from ClassesAndFunctions import *
import matplotlib.pyplot as plt


# MODEL DEFINITION --------------------------------------------
# Input Nonlinearity        
x_inp = [-0.4731, 0.2448, 0.419, 0.4531, 0.6003, 0.8796, 1]
y_inp = [-0.039, 0.0438, 0.1095, 0.1460, 0.2493, 0.5134, 0.627248]
# Output Nonlinearity
x_out = [-4.1721, -2.6285, -0.7644, 0.0366, 0.1922, 1.071, 2.2186, 3.0185, 3.0625, 3.6097, 6.8054]
y_out = [-3.1008, -2.8004, 0.2749, -0.0683, 0.0711, 5.0916, 14.0317, 22.6874, 22.5085, 25.7928, 49.701]

# Define Model
model_lin = ZTranferFunction([1, -0.9605], [1, -2.557, +2.214, -0.6516])
model = HammersteinWiener(model_lin, np.array([x_inp, y_inp]), np.array([x_out, y_out]))
model_lin2 = ZTranferFunction([1, -0.9605], [1, -2.557, +2.214, -0.6516])
model_c = HammersteinWiener(model_lin2, np.array([x_inp, y_inp]), np.array([x_out, y_out]))

# SIMULATION --------------------------------------------

# Read Input from File
# U = np.genfromtxt("measurements/test_full_train_mat_rs_10ms.csv", delimiter=",", skip_header=1)[:, 2]
# U = (U - 1000)/1000

# Time
dt = 0.01
# t_final = U.shape[0]*dt
t_final = 10.3
time = np.linspace(0, t_final, int(t_final/dt))

# Input Commands
# U = step input at time = 1
U = np.zeros(time.shape)
U[:30]     = 0 
U[30:60]   = 0.1
U[60:90]   = 0.2
U[90:120]  = 0.3
U[120:150] = 0.4
U[150:180] = 0.5
U[180:210] = 0.6
U[210:240] = 0.7
U[240:270] = 0.8
U[270:300] = 0.9
U[300:350] = 1
U[350:400] = 0
U[400:450] = 0.8
U[450:500] = 0.2
U[500:550] = 0
U[550:580] = 0.6
U[580:610] = 0.4
U[610:640] = 0.9
U[640:670] = 1
U[670:700] = 0.9
U[700:730] = 0.8  
U[730:760] = 0.55
U[760:880] = 0.8
U[880:910] = 1
U[910:940] = 0.3
U[940:970] = 0.8    
U[970:1000] = 0.5
U[1000:1030] = 0

Uc = np.zeros(time.shape)

# Y = output of the system
Y = np.zeros(time.shape)
Yc = np.zeros(time.shape)
error_prev = 0

# MAIN WHILE LOOP --------------------------------------------

TARGET = 20
CTRL_CENTER_U = 0.75
int_sum = 0

model.burn()
model_c.burn()

first_time_control_flag = True

for k in range(1, len(time)-1):

    error = TARGET - Yc[k-1]
    de_dt = (error - error_prev)/dt
    # int_sum = 0
    int_sum += error*dt

    # CONTROL LOGIC -----------

    # If Y > TARGET
    if Yc[k-1] > TARGET or (Yc[k-1] < TARGET and U[k] > 0.7):
        if first_time_control_flag:
            int_sum = 0
            first_time_control_flag = False
        Uc[k] = CTRL_CENTER_U + 0.008*error + 0.01*int_sum + 0*de_dt
    else:
        first_time_control_flag = True
        Uc[k] = U[k]

    if Uc[k] > U[k]:
        first_time_control_flag = True
        Uc[k] = U[k]


    # -------------------------
    # Uc[k] = 0.8 + 0.01*error + 0.1*int_sum + 0.0003*de_dt
    Uc[k] = np.clip(Uc[k], 0, 1)


    Y[k] = model(U[k])
    Yc[k] = model_c(Uc[k])  

    error_prev = error




# PLOTTING --------------------------------------------
# same plots but in a 2x1 grid
plt.figure(figsize=(10, 8))
plt.subplot(2, 1, 1)
plt.step(time, Uc*100, 'r', label='Input with Control')
plt.step(time, U*100, 'b--', label='Input')
plt.grid()
plt.legend()
plt.ylabel("Throttle [%]")

plt.subplot(2, 1, 2)
plt.step(time, Yc, 'r-', label='Output with Control')
plt.step(time, Y, 'b--', label='Output Model')
# horizontal dashed line at TARGET
plt.axhline(y=TARGET, color='k', linestyle='--', alpha=0.5, label='Target')
plt.grid()
plt.legend()
plt.ylabel("Current [A]")
plt.xlabel("Time [s]")



plt.show()

