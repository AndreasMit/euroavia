import numpy as np
import matplotlib.pyplot as plt

file_name = "measurements/test_with_simulation.csv"

# Load data from CSV file
data = np.genfromtxt(file_name, delimiter=',', dtype=str, skip_header=6)

# Remove double quotes from each element in the array
data = np.char.strip(data, '"')
# Convert array to float
data = data.astype(float)

print(data)
print(data.shape)

# Save data to csv with same name but with numpy
np.savetxt('measurements/test_with_simulation_mat.csv', data, delimiter=',', fmt='%f')