import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load data from CSV file
csv_file = 'odom_data.csv'  # Replace with your CSV file path
data = pd.read_csv(csv_file)

# Ensure the CSV file has at least three columns
if len(data.columns) < 3:
    raise ValueError("CSV file must contain at least three columns for X, Y, and Z coordinates.")

# Assign the first three columns to X, Y, and Z
x = data.iloc[:, 0]
y = data.iloc[:, 1]
z = data.iloc[:, 2]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Scatter plot
ax.scatter(x, y, z, c='blue', marker='o')

# Set labels for the axes
ax.set_xlabel('X (Front)')
ax.set_ylabel('Y (Left)')
ax.set_zlabel('Z (Up)')

# Adjust the view angle
ax.view_init(elev=90, azim=-90)

# Show plot
plt.show()
