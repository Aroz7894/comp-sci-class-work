""" Graphical demonstration of a simple PID controller.  

In this simulation the controller needs to decide how much upward
force to apply to an object to keep it at a fixed height.

Author: Nathan Sprague
Date: 1/17/2013

"""

import numpy as np
import matplotlib.pyplot as plt
import pid

ball_mass = 1.0
ball_pos = .5
ball_vel = 0.0
g = -9.8 # gravity


# Straight proportinal controller.  This will oscillate. 
#controller = pid.PID(70.0, 0.0, 0.0, -20.0, 20.0)

# Add a derivative term.  Stops the oscillation, but has "droop"
#controller = pid.PID(70.0, 0.0, 5.0, -20.0, 20.0)

# Add the integral term.  This now works OK. 
controller = pid.PID(70.0, 50.0, 5.0, -20.0, 20.0)

target = 1.0 

t = 0
dt = .05
steps = 60
vels = np.zeros(steps)
positions = np.zeros(steps)
forces = np.zeros(steps)
times = np.zeros(steps)
p_errors = np.zeros(steps)
i_errors = np.zeros(steps)
d_errors = np.zeros(steps)

for i in range(steps):
    error = target - ball_pos
    up_force = controller.update_PID(error, dt)
    g_force = ball_mass * g
    total_force = g_force + up_force
    acceleration = total_force / ball_mass
    t += dt
    ball_vel += acceleration * dt
    ball_pos += ball_vel * dt
    
    positions[i] = ball_pos
    p_errors[i] = controller.p_error
    i_errors[i] = controller.i_error
    d_errors[i] = controller.d_error

    vels[i] = ball_vel
    forces[i] = up_force
    times[i] = t
    
    

plt.subplot(511)
plt.plot(times, positions)
plt.xlabel("time (s)")
plt.ylabel("position (m)")
plt.grid()

plt.subplot(512)
plt.plot(times, p_errors)
plt.xlabel("time (s)")
plt.ylabel("p error (m)")
plt.grid()

plt.subplot(513)
plt.plot(times, i_errors)
plt.xlabel("time (s)")
plt.ylabel("i error")
plt.grid()

plt.subplot(514)
plt.plot(times, d_errors)
plt.xlabel("time (s)")
plt.ylabel("d error")
plt.grid()

plt.subplot(515)
plt.plot(times, forces)
plt.xlabel("time (s)")
plt.ylabel("force")
plt.grid()

plt.show()
    
