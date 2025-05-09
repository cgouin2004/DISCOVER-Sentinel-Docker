import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend

import matplotlib.pyplot as plt
import numpy as np

# Generate sample data
x = np.linspace(0, 10, 100)
y = np.sin(x)
z = np.cos(x)
data = np.random.randn(1000)

# Set the delay time (in seconds)
delay_time = 3

# Create a line plot
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Sine Wave')
plt.plot(x, z, label='Cosine Wave')
plt.title('Line Plot')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)
plt.savefig('line_plot.png')
plt.close()
