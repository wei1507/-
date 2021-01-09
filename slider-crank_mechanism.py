import numpy as np
import math
import matplotlib.pyplot as plt


#  參考課本p.330~337
#  r2:曲柄, r3:連桿 (mm)
r2 = 38.2
r3 = 130.0
r4 = 0.0

# Wheel (m)
wheel_diameter = 0.632
wheel_circumference = wheel_diameter*math.pi

# vehicle's velocity (m/s)
VehicleVelocity = 27.78
final_reduction_gear_ratio = 3.444
fourth_transmission_gear_ratio = 1.242

# rpm
VehicleRpm = 27.78*60/wheel_circumference
EngineRpm = VehicleRpm * final_reduction_gear_ratio * fourth_transmission_gear_ratio

# Radian
theta_1 = 0
theta_2 = 0
r1_Velocity = 0
r1_Acceleration = 0
theta_2_degree = 0

x = np.array([])
y = np.array([])
while theta_2_degree <= 360.1:
    theta_2 = math.radians(theta_2_degree)
    y = np.append(y, theta_2)
    # Position
    A = -2*r2*(math.cos(theta_1)*math.cos(theta_2) + math.sin(theta_1)*math.cos(theta_2))
    B = r2*r2 - r3*r3
    r1 = (-A + (A*A-4*B)**0.5)/2
    theta_3 = math.atan((-r2*math.sin(theta_2))/(r1*math.cos(theta_1) - r2*math.cos(theta_2)))
    # 只在 i 方向移動
    r1_position_i = r2*math.cos(theta_2) + r3*math.cos(theta_3)
    x = np.append(x, r1_position_i)
    # Velocity
    theta_2_v = EngineRpm*2*math.pi/60
    C = np.array([[1, r3*math.sin(theta_3)],
                [0, -r3*math.cos(theta_3)]])
    D = np.array([[-r2*theta_2_v*math.sin(theta_2)],
                [r2*theta_2_v*math.cos(theta_2)]])
    C_inverse = np.linalg.inv(C)
    E = C_inverse.dot(D)
    r1_Velocity_temp = E[0]
    theta_3_v = E[1]
    if abs(r1_Velocity_temp) > r1_Velocity:
        r1_Velocity = r1_Velocity_temp
        max_theta_2_v = theta_2_degree

    # Acceleration
    theta_2_a = 0
    F = np.array([-r2*math.cos(theta_2)*theta_2_v**2 - r3*math.cos(theta_3)*theta_3_v**2,
                -r2*math.sin(theta_2)*theta_2_v**2 - r3*math.sin(theta_3)*theta_3_v**2])
    G = C_inverse.dot(F)
    r1_Acceleration_temp = G[0]
    theta_3_a = G[1]
    if abs(r1_Acceleration_temp) > r1_Acceleration:
        r1_Acceleration = r1_Acceleration_temp
        max_theta_2_a = theta_2_degree
    theta_2_degree += 0.1

print("Max velocity: ", r1_Velocity/1000)
print("theta_2: ", max_theta_2_v)
print("Max acceleration:", r1_Acceleration/1000)
print("theta_2: ", max_theta_2_a)
plt.plot(y, x)
plt.show()
