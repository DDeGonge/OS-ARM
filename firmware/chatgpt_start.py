# 3DOF code attempt 1
import numpy as np

# DH Parameters
a1 = 0.3  # Link 1 offset
alpha1 = 0  # Link 1 twist
theta1 = np.pi/2  # Joint 1 angle

a2 = 0.2  # Link 2 offset
alpha2 = 0  # Link 2 twist
theta2 = np.pi/2  # Joint 2 angle

a3 = 0.1  # Link 3 offset
alpha3 = 0  # Link 3 twist
theta3 = np.pi/2  # Joint 3 angle

# Homogeneous Transformation Matrices
T1 = np.array([[np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
               [np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
               [0, np.sin(alpha1), np.cos(alpha1), 0],
               [0, 0, 0, 1]])
T2 = np.array([[np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
               [np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
               [0, np.sin(alpha2), np.cos(alpha2), 0],
               [0, 0, 0, 1]])
T3 = np.array([[np.cos(theta3), -np.sin(theta3)*np.cos(alpha3), np.sin(theta3)*np.sin(alpha3), a3*np.cos(theta3)],
               [np.sin(theta3), np.cos(theta3)*np.cos(alpha3), -np.cos(theta3)*np.sin(alpha3), a3*np.sin(theta3)],
               [0, np.sin(alpha3), np.cos(alpha3), 0],
               [0, 0, 0, 1]])

# Homogeneous Transformation Matrix for end-effector
T_EE = T1 @ T2 @ T3

# Inverse Kinematics
# Position of end-effector
p_EE = T_EE[:3, 3]

# Rotation of end-effector
R_EE = T_EE[:3, :3]

# Euler angles from Rotation matrix
theta1 = np.arctan2(R_EE[1,0], R_EE[0,0])
theta2 = np.arctan2(-R_EE[2,0], np.sqrt(R_EE[0,0]**2 + R_EE



# 3DOF code attempt 2
import numpy as np

# DH Parameters
d1 = 0.2 # Link 1 length
a1 = 0.1 # Link 1 offset
alpha1 = np.pi/2 # Link 1 twist
theta1 = np.pi/4 # Joint 1 angle

d2 = 0.3 # Link 2 length
a2 = 0.2 # Link 2 offset
alpha2 = 0 # Link 2 twist
theta2 = np.pi/2 # Joint 2 angle

d3 = 0.2 # Link 3 length
a3 = 0.1 # Link 3 offset
alpha3 = np.pi/2 # Link 3 twist
theta3 = np.pi/4 # Joint 3 angle

# Homogeneous Transformation Matrices
T1 = np.array([[np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
               [np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
               [0, np.sin(alpha1), np.cos(alpha1), d1],
               [0, 0, 0, 1]])
T2 = np.array([[np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
               [np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
               [0, np.sin(alpha2), np.cos(alpha2), d2],
               [0, 0, 0, 1]])
T3 = np.array([[np.cos(theta3), -np.sin(theta3)*np.cos(alpha3), np.sin(theta3)*np.sin(alpha3), a3*np.cos(theta3)],
               [np.sin(theta3), np.cos(theta3)*np.cos(alpha3), -np.cos(theta3)*np.sin(alpha3), a3*np.sin(theta3)],
               [0, np.sin(alpha3), np.cos(alpha3), d3],
               [0, 0, 0, 1]])

# Calculate the end-effector position using forward kinematics
T_end = np.dot(T1,np.dot(T2,T3))

# Desired position
x_d = 0.5
y_d = 0.4
z_d = 0.3

# Calculate the position error
err_pos = T_end[:3,3] - np.array([x_d,y_d,z_d,1])

# Implement inverse kinematics algorithm to calculate the joint




# 6DOF partial code

import numpy as np

# DH Parameters
d1 = 0.2 # Link 1 length
a1 = 0.1 # Link 1 offset
alpha1 = np.pi/2 # Link 1 twist
theta1 = np.pi/4 # Joint 1 angle

d2 = 0.3 # Link 2 length
a2 = 0.2 # Link 2 offset
alpha2 = 0 # Link 2 twist
theta2 = np.pi/2 # Joint 2 angle

d3 = 0.2 # Link 3 length
a3 = 0.1 # Link 3 offset
alpha3 = np.pi/2 # Link 3 twist
theta3 = np.pi/4 # Joint 3 angle

d4 = 0.1 # Link 4 length
a4 = 0 # Link 4 offset
alpha4 = 0 # Link 4 twist
theta4 = np.pi/2 # Joint 4 angle

d5 = 0.1 # Link 5 length
a5 = 0 # Link 5 offset
alpha5 = 0 # Link 5 twist
theta5 = np.pi/2 # Joint 5 angle

d6 = 0.1 # Link 6 length
a6 = 0 # Link 6 offset
alpha6 = 0 # Link 6 twist
theta6 = np.pi/2 # Joint 6 angle

# Homogeneous Transformation Matrices
T1 = np.array([[np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
               [np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
               [0, np.sin(alpha1), np.cos(alpha1), d1],
               [0, 0, 0, 1]])
T2 = np.array([[np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
               [np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
               [0, np.sin(alpha2), np.cos(alpha2), d2],
               [0, 0, 0, 1]])
T3 = np.array([[np.cos(theta3), -np.sin(theta3)*np.cos(alpha3), np.sin(theta3)*np.sin(alpha3), a3*np.cos(theta3)],
               [np.sin(theta3), np.cos(theta3)*np.cos(alpha3), -np.cos(theta3)*np.sin(alpha3), a3*np.sin(theta3)],
               [0, np.sin(alpha3), np.cos(alpha3), d3],
               [0, 0, 0
