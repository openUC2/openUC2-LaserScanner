import matplotlib.pyplot as plt
import numpy as np

# Define parameters similar to the draw() function
X_MIN, X_MAX = 0, 1000  # X-axis range
Y_MIN, Y_MAX = 0, 1000  # Y-axis range
STEP = 100              # Step size for both X and Y
tPixelDwelltime = 10    # Microseconds for each pixel dwell
nFrames = 2             # Number of frames to render

# Calculate timing intervals
nX = (X_MAX - X_MIN) // STEP  # Number of X steps
nY = (Y_MAX - Y_MIN) // STEP  # Number of Y steps
frame_time = nX * nY * tPixelDwelltime  # Time for one frame (µs)
line_time = nY * tPixelDwelltime        # Time for one line (µs)
total_time = nFrames * frame_time       # Total simulation time (µs)

# Create time points
time_resolution = 1  # Microsecond resolution
time_points = np.arange(0, total_time, time_resolution)

# Initialize signals
pixel_trigger = np.zeros_like(time_points)
line_trigger = np.zeros_like(time_points)
frame_trigger = np.zeros_like(time_points)

# Generate the trigger signals
time_idx = 0
for iFrame in range(nFrames):
    # Set frame trigger
    frame_trigger[time_idx] = 1

    for x in range(X_MIN, X_MAX + 1, STEP):
        # Set line trigger
        line_trigger[time_idx] = 1

        for y in range(Y_MIN, Y_MAX + 1, STEP):
            # Set pixel trigger
            pixel_trigger[time_idx] = 1
            time_idx += tPixelDwelltime

        # Advance time for line delay
        time_idx += (tPixelDwelltime * (Y_MAX - Y_MIN) // STEP) - line_time

    # Advance time for frame delay
    time_idx += frame_time - (line_time * nX)

# Plot the trigger signals
plt.figure(figsize=(12, 6))
plt.step(time_points, pixel_trigger, where="post", label="Pixel Trigger")
plt.step(time_points, line_trigger, where="post", label="Line Trigger")
plt.step(time_points, frame_trigger, where="post", label="Frame Trigger")
plt.xlabel("Time (µs)")
plt.ylabel("Trigger State")
plt.title("Trigger Pattern for Pixel, Line, Frame")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()