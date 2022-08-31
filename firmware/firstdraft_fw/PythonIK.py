#######################################################################################
# Progam: Inverse Kinematics for a Robotic Arm Using the Pseudoinverse of the Jacobian
# Description: Given a desired end position (x, y, z) of the end effector of a robot, 
#   calculate the joint angles (i.e. angles for the servo motors).
# Author: Addison Sears-Collins
# Website: https://automaticaddison.com
# Date: October 15, 2020
#######################################################################################
 
from turtle import position
import numpy as np # Scientific computing library
import math
import time
from matplotlib import pyplot as plt

"""
GROUND - J0 - J1 - J2 - J3 - J4 - EFFECTOR (trainer)

COORDINATES (TOP DOWN VIEW)
Z+ is up duh

    ^ y+
    |
    |
    0  --> +x

ZERO POSITION
J0 - facing Y+
J1 - parallel to ground facing Y+
J2 - straight
J3 - straight
J4 - doesn't matter lol
J5 - working on it
   o - o - o - | - _ - x  
  _|_
___|___
 / / /


"""
links_mm = [6.0, 140.82, 250.0, 250.0, 83.5, 50.0]
angles_min_rad = [-3.0, 0.0, 0.0, -1.5, -3.0, -1.5]
angles_max_rad = [ 3.0, 2.0, 2.0,  1.5,  3.0,  1.5]

samplerate_hz = 1000

class pose:
    def __init__(self, positions=None, rotations=None):
        self.position_mm = [0,0,0] if positions is None else positions
        self.rotation_xyz_rad = [0,0,0] if rotations is None else rotations
        self.joint_angles = [0,0,0,0,0,0]

    def apply_limits(self, a_min, a_max):
        new_angles = []
        for j, amin, amax in zip(self.joint_angles, a_min, a_max):
            new_angles.append(max(min(j, amax), amin))
        self.joint_angles = new_angles

def pthg(x, y):
    return pow(pow(x, 2) + pow(y, 2), 0.5)

def cosines_law(a, b, c):
    # Solve for angle A
    numsum = (-pow(a, 2) + pow(b, 2) + pow(c, 2)) / (2 * b * c)
    numsum = max(min(numsum, 1), -1)
    return math.acos(numsum)

# Starting with 3 dof-ish
def solve_ik(tar_pose):
    [x, y, z] = tar_pose.position_mm

    # Rotations are in link J3 coordinate frame to make my life easier
    # Start with J0 cuz it's fully defined by x and y only, #ez
    a0 = math.atan2(x, y)

    # Also J4 is just rotation about x
    a4 = tar_pose.rotation_xyz_rad[0]

    # Calculate where J3 needs to be in xyz space
    end_disp_xy = sum(links_mm[4:6]) * math.cos(tar_pose.rotation_xyz_rad[1])
    end_disp_z  = sum(links_mm[4:6]) * math.sin(tar_pose.rotation_xyz_rad[1])
    j3pos = [x - (end_disp_xy * math.sin(a0)), y - (end_disp_xy * math.cos(a0)), z - end_disp_z]

    if j3pos[2] < 0:
        print("UHOH bad news bears my friend")

    # print("J3pos: ", j3pos)

    # Finally calculate J1 and J2
    # on [xy, z] plane
    dz = j3pos[2] - sum(links_mm[0:2])
    dxy = pthg(j3pos[0], j3pos[1])
    dxyz = pthg(dxy, dz)
    # print(dxy, dz, dxyz)
    temp_a1a = math.atan2(dz, dxy)
    temp_a1b = cosines_law(links_mm[3], links_mm[2], dxyz)
    # print(temp_a1a, temp_a1b)
    a1 = temp_a1a + temp_a1b
    a2 = math.pi - cosines_law(dxyz, links_mm[3], links_mm[2])
    a3 = a2 - a1 + tar_pose.rotation_xyz_rad[1]

    tar_pose.joint_angles = [a0, a1, a2, a3, a4, 0]
    tar_pose.apply_limits(angles_min_rad, angles_max_rad)

def solve_fk(tar_pose):
    a = tar_pose.joint_angles
    hinges_xylen_mm = links_mm[2]*math.cos(a[1]) + links_mm[3]*math.cos(a[1] - a[2]) + (links_mm[4]+ links_mm[5]) * math.cos(a[1] - a[2] + a[3])
    xpos = hinges_xylen_mm * math.sin(a[0])
    ypos = hinges_xylen_mm * math.cos(a[0])
    zpos = links_mm[0] + links_mm[1] + links_mm[2]*math.sin(a[1]) + links_mm[3]*math.sin(a[1] - a[2]) + (links_mm[4]+ links_mm[5]) * math.sin(a[1] - a[2] + a[3])

    return [xpos, ypos, zpos]

# def plot_robot(tar_pose):
#     points = [[0,0,0]]
#     points.append([0, 0, 0 + links_mm[0]])
#     points.append([0, 0, 0 + sum(links_mm[0:2])])

#     print(points)

#     nppts = np.array(points)
#     print(nppts)
#     plt.scatter(nppts[:,0], nppts[:,1], s=20, label='xy')
#     plt.scatter(nppts[:,0], nppts[:,2], s=20, label='xz')
#     plt.scatter(nppts[:,1], nppts[:,2], s=20, label='yz')
#     plt.show()

def test_pose(tar_pose):
    xyz_fw = solve_fk(tar_pose)
    errors = [abs(fw - nm) for fw, nm in zip(xyz_fw, tar_pose.position_mm)]
    print("POSITION ERRORS: ", errors)
    if sum(errors) > 0.1:
        return False
    return True

def generate_line(start_pose, end_pose, velocity, acceleration=0):
    def euclidian_dist(pt1, pt2):
        return pow(pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2) + pow(pt1[2] - pt2[2], 2), 0.5)
    vec_len_mm = euclidian_dist(start_pose.position_mm, end_pose.position_mm)
    steps = int((vec_len_mm / velocity) * samplerate_hz)
    position_vecs = [np.linspace(p0, p1, steps) for p0, p1 in zip(start_pose.position_mm, end_pose.position_mm)]
    rotation_vecs = [np.linspace(p0, p1, steps) for p0, p1 in zip(start_pose.rotation_xyz_rad, end_pose.rotation_xyz_rad)]

    def generate_pose(i):
        tp = pose([p[i] for p in position_vecs], [r[i] for r in rotation_vecs])
        solve_ik(tp)
        return tp

    pose_steps = [generate_pose(i) for i in range(steps)]
    return pose_steps

def execute_command(poses):
    tNext = time.time() + (1 / samplerate_hz)
    for p in poses:
        print(p.joint_angles)

        if time.time() > tNext: print("UHOH slow down please")
        while(time.time() < tNext): pass
        tNext += (1 / samplerate_hz)

def main():
    goal_pose2 = pose()
    goal_pose2.position_mm = [0,633,147]
    goal_pose2.rotation_xyz_rad = [0, 0, 0]

    tstart = time.time()
    solve_ik(goal_pose2)
    print("Runtime: {}".format(time.time() - tstart))
    print("Joint Angles:", goal_pose2.joint_angles)
    print(test_pose(goal_pose2))


    pt1 = pose()
    pt1.position_mm = [0,500,147]
    pt1.rotation_xyz_rad = [0, 0, 0]
    pt2 = pose()
    pt2.position_mm = [0,600,147]
    pt2.rotation_xyz_rad = [0, 0, 0]
    pose_steps = generate_line(pt1, pt2, 100)

    execute_command(pose_steps)
 
if __name__ == '__main__':
  main()