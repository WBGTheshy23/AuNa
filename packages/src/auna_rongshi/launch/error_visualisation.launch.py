import pandas as pd
import matplotlib.pyplot as plt

# read the CSV file
df = pd.read_csv('pose_errors.csv')

robots = df['robot_name'].unique()

plt.figure(figsize=(14, 6))

# postion error subplot
plt.subplot(1, 2, 1)
for robot in robots:
    robot_data = df[df['robot_name'] == robot]
    plt.plot(robot_data['timestamp'], robot_data['position_error'], label=robot)
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.title('Position Error Over Time')
plt.legend()
plt.grid(True)

# yaw error subplot
plt.subplot(1, 2, 2)
for robot in robots:
    robot_data = df[df['robot_name'] == robot]
    plt.plot(robot_data['timestamp'], robot_data['yaw_error_deg'], label=robot)
plt.xlabel('Time (s)')
plt.ylabel('Yaw Error (degrees)')
plt.title('Yaw Error Over Time')
plt.legend()
plt.grid(True)

plt.tight_layout()

# save the figure
plt.savefig('pose_error_comparison.png', dpi=300)

# show the plot
plt.show()
