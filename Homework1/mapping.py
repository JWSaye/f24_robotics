import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image
import numpy as np

# Load image
image = Image.open('Homework1/apartment.png')

# Initial positions on the map (in pixels)
initial_positions = [
    (1500, 1200),   # ZONE 1
    (1050, 750),    # ZONE 2
    (500, 700),     # ZONE 3
    (300, 1660)     # ZONE 4
]

colors = ['red', 'green', 'blue', 'orange', 'purple']

def calculate_total_path_length(df):
    """Calculate the total path length by summing up the distance between consecutive points."""
    total_length = 0
    for i in range(1, len(df)):
        x_diff = df['x'].iloc[i] - df['x'].iloc[i - 1]
        y_diff = df['y'].iloc[i] - df['y'].iloc[i - 1]
        total_length += np.sqrt(x_diff**2 + y_diff**2)
    return total_length

def calculate_furthest_point(df, initial_x, initial_y):
    """Calculate the distance to the furthest point from the starting point."""
    furthest_distance = 0
    for i in range(len(df)):
        x_diff = df['x'].iloc[i] - initial_x
        y_diff = df['y'].iloc[i] - initial_y
        distance = np.sqrt(x_diff**2 + y_diff**2)
        if distance > furthest_distance:
            furthest_distance = distance
    return furthest_distance

for j in range(1, 5):
    # Plot the image
    plt.imshow(image)

    plt.title(f'TurtleBot Paths from Zone {j}')

    for i in range(1, 6):
        # Load the CSV with robot positions (assuming columns 'x', 'y' for positions in meters)
        df = pd.read_csv(f'Homework1/Zone{j}_Run{i}_turtlebot_positions.csv')

        # Real-world dimensions and corresponding image size
        real_world_width = 12.5  # in meters (example)
        real_world_height = 11  # in meters (example)
        image_width, image_height = image.size  # in pixels

        # Calculate scaling factors
        scale_x = image_width / real_world_width  # pixels per meter
        scale_y = image_height / real_world_height  # pixels per mete

        # Transform coordinates
        df['x_pixel'] = df['x'] * scale_x + initial_positions[j-1][0]
        df['y_pixel'] = -(df['y'] * scale_y) + initial_positions[j-1][1]

        # Plot the robot's path
        plt.plot(df['x_pixel'].values, df['y_pixel'].values, ls='dotted', color=colors[i-1])
        
        #Calculate total path length in meters
        total_path_length = calculate_total_path_length(df)
        print(f"Zone {j} Run {i}: Total Path Length = {total_path_length:.2f} meters")

        # Calculate the distance to the furthest point from the initial position
        initial_x_meter, initial_y_meter = initial_positions[j-1][0] / scale_x, initial_positions[j-1][1] / scale_y
        furthest_distance = calculate_furthest_point(df, 0, 0)
        print(f"Zone {j} Run {i}: Furthest Point from Start = {furthest_distance:.2f} meters")

    plt.figlegend(['Run 1', 'Run 2', 'Run 3', 'Run 4', 'Run 5'], loc='center right')

    # Show the plot
    # plt.show()
