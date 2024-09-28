import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Create figure and axis for plotting
fig, ax = plt.subplots()

# Initialize empty lists to store x, y1, and y2 data points
x_vals = []
y1_vals = []
y2_vals = []
y3_vals = []
y4_vals = []

# Define update function
def update(frame):
    # Read the CSV file with real-time data
    # csv_file_path should point to a continuously updating CSV file
    csv_file_path = 'records.csv'  # Replace with your real-time CSV file path
    data = pd.read_csv(csv_file_path)

    # Assuming the CSV has columns 'y1' and 'y2' and we use index as x-axis (line number)
    x_vals.clear()
    y1_vals.clear()
    y2_vals.clear()
    y3_vals.clear()
    y4_vals.clear()
    
    x_vals.extend(data.index)  # Line numbers as x-axis
    y1_vals.extend(data['mass'])  # Replace 'y1' with the first line's y-axis data
    y2_vals.extend(data['y'])  # Replace 'y2' with the second line's y-axis data
    y3_vals.extend(data['crash'])  # Replace 'y2' with the second line's y-axis data
    y4_vals.extend(data['v'])  # Replace 'y2' with the second line's y-axis data

    # Clear the previous plot
    ax.clear()

    # Plot both sets of data on the same y-axis
    ax.plot(x_vals, y1_vals, 'g-',  label='mass Data')  # Line 1
    ax.plot(x_vals, y2_vals, 'b-',  label='y Data')  # Line 2
    ax.plot(x_vals, y3_vals, 'r-',  label='crash Data')  # Line 3
    ax.plot(x_vals, y4_vals, 'c-',  label='v Data')  # Line 3

    # Adding labels and title
    ax.set_xlabel('Line Number')
    ax.set_ylabel('Y-axis Label')  # Customize the y-axis label
    ax.set_title('Real-time Data Plot with Two Lines on One Y-axis')

    # Add a legend
    ax.legend(loc='upper left')

# Set up the animation
ani = animation.FuncAnimation(fig, update, interval=1000, cache_frame_data=False)  # Update every second (1000 ms)

# Show the plot
plt.show()
