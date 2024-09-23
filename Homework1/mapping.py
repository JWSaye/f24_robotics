import matplotlib.pyplot as plt
import pandas as pd
from PIL import Image

# Load image
image = Image.open('Homework1/apartment.png')

# Plot the image
plt.imshow(image)

colors = ['red', 'green', 'blue', 'orange', 'purple']

plt.title('TurtleBot Paths from Zone 2')

for i in range(1, 6):
    # Load the CSV with robot positions (assuming columns 'x', 'y' for positions in meters)
    df = pd.read_csv(f'Homework1/Zone1_Run{i}_turtlebot_positions.csv')

    # Real-world dimensions and corresponding image size
    real_world_width = 12.5  # in meters (example)
    real_world_height = 11  # in meters (example)
    image_width, image_height = image.size  # in pixels

    # Calculate scaling factors
    scale_x = image_width / real_world_width  # pixels per meter
    scale_y = image_height / real_world_height  # pixels per meter
    
    # ZONE 1 Initial position on the map (in pixels)
    initial_position_x = 1500  # example pixel position
    initial_position_y = 1200  # example pixel position

    # ZONE 4 Initial position on the map (in pixels)
    # initial_position_x = 300  # example pixel position
    # initial_position_y = 1660  # example pixel position
    
    # ZONE 2 Initial position on the map (in pixels)
    # initial_position_x = 1050  # example pixel position
    # initial_position_y = 750  # example pixel position
    
    # ZONE 3 Initial position on the map (in pixels)
    # initial_position_x = 500  # example pixel position
    # initial_position_y = 700  # example pixel position

    # Transform coordinates
    df['x_pixel'] = df['y'] * scale_x + initial_position_x
    df['y_pixel'] = df['x'] * scale_y + initial_position_y

    # Plot the robot's path
    plt.plot(df['x_pixel'], df['y_pixel'], ls='dotted', color=colors[i-1])

plt.figlegend(['Run 1', 'Run 2', 'Run 3', 'Run 4', 'Run 5'], loc='center right')

# Show the plot
plt.show()
