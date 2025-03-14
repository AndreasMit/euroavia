import numpy as np
import matplotlib.pyplot as plt
import sys

# Filename without extension
# file_name = "measurements/2024_05_20/test_1"
file_name = sys.argv[1]

# Load data from CSV file
data = np.genfromtxt(f"{file_name}.csv", delimiter=',', dtype=str, skip_header=10)

# Remove double quotes from each element in the array
data = np.char.strip(data, '"')
# print(data[1:10])
# Convert array to float
data = data.astype(float)
# print(data[1:10])

print(data)
print(data.shape)

# Save data to csv file
np.savetxt(f"{file_name}_mat.csv", data, delimiter=',', fmt='%f')
print(f"Data converted and saved to {file_name}_mat.csv.")