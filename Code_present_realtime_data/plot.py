import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file
csv_file_path = 'records.csv'  # Replace with your CSV file path
data = pd.read_csv(csv_file_path)

# Assuming the CSV has columns 'y1', 'y2', 'y3', 'y4', and using index as x-axis
x_vals = data.index
y1_vals = data['y']  # First y-axis, first value
y2_vals = data['v']  # First y-axis, second value
y3_vals = data['crash']  # Second y-axis, first value
y4_vals = data['mass']  # Second y-axis, second value

# Create a figure and first axis for the first y-axis (left)
fig, ax1 = plt.subplots()

# Plot two values on the first y-axis (y1 and y2)
ax1.plot(x_vals, y1_vals, 'g-', label='Y Data')
ax1.plot(x_vals, y2_vals, 'b-', label='v Data')
ax1.set_xlabel('Index')
ax1.set_ylabel('Y1-axis', color='g')
ax1.tick_params(axis='y', labelcolor='g')

# Highlight max and min for y1 and y2
y1_max_idx = y1_vals.idxmax()
y1_min_idx = y1_vals.idxmin()
y2_max_idx = y2_vals.idxmax()
y2_min_idx = y2_vals.idxmin()

ax1.annotate(f'Max Y1: {y1_vals.max()}', xy=(y1_max_idx, y1_vals.max()), xytext=(y1_max_idx, y1_vals.max()+0.05),
              fontsize=8)
ax1.annotate(f'Min Y1: {y1_vals.min()}', xy=(y1_min_idx, y1_vals.min()), xytext=(y1_min_idx, y1_vals.min()-0.05),
              fontsize=8)

ax1.annotate(f'Max Y2: {y2_vals.max()}', xy=(y2_max_idx, y2_vals.max()), xytext=(y2_max_idx, y2_vals.max()+0.05),
             fontsize=8)
ax1.annotate(f'Min Y2: {y2_vals.min()}', xy=(y2_min_idx, y2_vals.min()), xytext=(y2_min_idx, y2_vals.min()-0.05),
             fontsize=8)

# Create a second y-axis for the second y-values (right)
ax2 = ax1.twinx()

# Plot two values on the second y-axis (y3 and y4)
ax2.plot(x_vals, y3_vals, 'r-', label='crash Data')
# ax2.plot(x_vals, y4_vals, 'm-', label='mass Data')
ax2.set_ylabel('Y2-axis', color='r')
ax2.tick_params(axis='y', labelcolor='r')

# Highlight max and min for y3 and y4
y3_max_idx = y3_vals.idxmax()
y3_min_idx = y3_vals.idxmin()
y4_max_idx = y4_vals.idxmax()
y4_min_idx = y4_vals.idxmin()

ax2.annotate(f'Max Y3: {y3_vals.max()}', xy=(y3_max_idx, y3_vals.max()), xytext=(y3_max_idx, y3_vals.max()+0.05),
             arrowprops=dict(facecolor='red', shrink=0.05), fontsize=8)
ax2.annotate(f'Min Y3: {y3_vals.min()}', xy=(y3_min_idx, y3_vals.min()), xytext=(y3_min_idx, y3_vals.min()-0.05),
             arrowprops=dict(facecolor='red', shrink=0.05), fontsize=8)

# ax2.annotate(f'Max Y4: {y4_vals.max()}', xy=(y4_max_idx, y4_vals.max()), xytext=(y4_max_idx, y4_vals.max()+0.05),
#              arrowprops=dict(facecolor='magenta', shrink=0.05), fontsize=8)
# ax2.annotate(f'Min Y4: {y4_vals.min()}', xy=(y4_min_idx, y4_vals.min()), xytext=(y4_min_idx, y4_vals.min()-0.05),
#              arrowprops=dict(facecolor='magenta', shrink=0.05), fontsize=8)

# Add legends for both y-axes
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Show the plot
plt.title('Two Y-axes: 2 Lines on First, 2 Lines on Second')
plt.show()
