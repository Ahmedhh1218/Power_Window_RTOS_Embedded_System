import matplotlib.pyplot as plt

# Define the start and end times for each function call
red_led_times = [(50, 1000), (1050, 2000), (2050, 3000), (3050, 4000), (4050, 5000)]
blue_led_times = [(0, 50), (1000, 1050), (2000, 2050), (3000, 3050), (4000, 4050)]
idle_times = [(0, 0)]

# Create a new figure
fig, ax = plt.subplots()

# Plot the red LED function calls
for start, end in red_led_times:
    ax.plot([start, end], [2, 2], color='red')

# Plot the blue LED function calls
for start, end in blue_led_times:
    ax.plot([start, end], [3, 3], color='blue')

# Plot the idle_hook function calls
for start, end in idle_times:
    ax.plot([start, end], [1, 1], color='black')

# Set the axis labels and title
ax.set_xlabel('Time (ms)')
ax.set_ylabel('Task')
ax.set_title('Task Timing Diagram')

# Set the axis limits and ticks
ax.set_xlim(0, 5000)
ax.set_xticks(range(0, 5100, 1000))
ax.set_ylim(0.5, 3.5)
ax.set_yticks([1, 2, 3])
ax.set_yticklabels(['Idle', 'Red', 'Blue'])

# Show the plot
plt.show()
