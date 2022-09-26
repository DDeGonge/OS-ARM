from unittest import skip
import numpy as np # Scientific computing library
import math
import time
import SerialDevice as sd
from matplotlib import pyplot as plt
# from PyQt5.QtWidgets import QApplication, QWidget

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

samplerate_hz = 15

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

class Robot:
    active_axes = ['x','y','z','a','b']
    angles_min_rad = [-3.0, 0.0, 0.0, -1.7, -3.0, -1.5]
    angles_max_rad = [ 3.0, 2.25, 2.25,  1.7,  3.0,  1.5]
    links_mm = [6.0, 140.82, 250.0, 250.0, 83.5, 50.0]
    sdevice = None
    def __init__(self, skip_sd=False):
        if not skip_sd:
            self.sdevice = sd.SerialDevice()
        self.last_pose = pose()
        self.update_pose()
        time.sleep(0.2)

    # Starting with 3 dof-ish
    def solve_ik(self, tar_pose):
        [x, y, z] = tar_pose.position_mm

        # Rotations are in link J3 coordinate frame to make my life easier
        # Start with J0 cuz it's fully defined by x and y only, #ez
        a0 = math.atan2(x, y)

        # Also J4 is just rotation about x
        a4 = tar_pose.rotation_xyz_rad[0]

        # Calculate where J3 needs to be in xyz space
        end_disp_xy = sum(self.links_mm[4:6]) * math.cos(tar_pose.rotation_xyz_rad[1])
        end_disp_z  = sum(self.links_mm[4:6]) * math.sin(tar_pose.rotation_xyz_rad[1])
        j3pos = [x - (end_disp_xy * math.sin(a0)), y - (end_disp_xy * math.cos(a0)), z - end_disp_z]

        if j3pos[2] < 0:
            print("UHOH bad news bears my friend")

        # Finally calculate J1 and J2
        # on [xy, z] plane
        dz = j3pos[2] - sum(self.links_mm[0:2])
        dxy = self.pthg(j3pos[0], j3pos[1])
        dxyz = self.pthg(dxy, dz)
        temp_a1a = math.atan2(dz, dxy)
        temp_a1b = self.cosines_law(self.links_mm[3], self.links_mm[2], dxyz)
        a1 = temp_a1a + temp_a1b
        a2 = math.pi - self.cosines_law(dxyz, self.links_mm[3], self.links_mm[2])
        a3 = a2 - a1 + tar_pose.rotation_xyz_rad[1]

        tar_pose.joint_angles = [a0, a1, a2, a3, a4, 0]
        tar_pose.apply_limits(self.angles_min_rad, self.angles_max_rad)

    def solve_fk(self, tar_pose):
        a = tar_pose.joint_angles
        hinges_xylen_mm = self.links_mm[2]*math.cos(a[1]) + self.links_mm[3]*math.cos(a[1] - a[2]) + (self.links_mm[4]+ self.links_mm[5]) * math.cos(a[1] - a[2] + a[3])
        xpos = hinges_xylen_mm * math.sin(a[0])
        ypos = hinges_xylen_mm * math.cos(a[0])
        zpos = self.links_mm[0] + self.links_mm[1] + self.links_mm[2]*math.sin(a[1]) + self.links_mm[3]*math.sin(a[1] - a[2]) + (self.links_mm[4]+ self.links_mm[5]) * math.sin(a[1] - a[2] + a[3])
        xang = a[4]
        yang = a[1] - a[2] + a[3]
        zang = 0.0
        return [xpos, ypos, zpos], [xang, yang, zang]

    def update_pose(self):
        positions, angles = self.solve_fk(self.last_pose)
        self.last_pose.position_mm = positions
        self.last_pose.rotation_xyz_rad = angles

    def moveto(self, newpose, velocity, acceleration=0):
        # Calculate moves
        def euclidian_dist(pt1, pt2):
            return pow(pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2) + pow(pt1[2] - pt2[2], 2), 0.5)
        vec_len_mm = euclidian_dist(self.last_pose.position_mm, newpose.position_mm)
        steps = int((vec_len_mm / velocity) * samplerate_hz)
        position_vecs = [np.linspace(p0, p1, steps) for p0, p1 in zip(self.last_pose.position_mm, newpose.position_mm)]
        rotation_vecs = [np.linspace(p0, p1, steps) for p0, p1 in zip(self.last_pose.rotation_xyz_rad, newpose.rotation_xyz_rad)]
        def generate_pose(i):
            tp = pose([p[i] for p in position_vecs], [r[i] for r in rotation_vecs])
            self.solve_ik(tp)
            if self.test_pose(tp) is False:
                print("ERRA")
            return tp
        pose_steps = [generate_pose(i) for i in range(steps)]
        
        # Execute moves
        tNext = time.time() + (1 / samplerate_hz)
        for p in pose_steps:
            self.cmd_position(p.joint_angles)
            self.last_pose.joint_angles = p.joint_angles

            if time.time() > tNext: print("UHOH slow down please")
            while(time.time() < tNext): pass
            tNext += (1 / samplerate_hz)

        # Update last pose
        self.update_pose()

    def zero(self):
        cmd = 'g92'
        for ax in self.active_axes:
            cmd += " {}".format(ax)
        # print(cmd)
        self.sdevice.command(cmd)

    def pose_command(self, angles, feedrate=None):
        self.last_pose.joint_angles = angles
        self.update_pose()
        cmd = 'g1'
        for ax, pos in zip(self.active_axes, angles):
            cmd += " {}{}".format(ax, pos)
        if feedrate is not None:
            cmd += " f{}".format(feedrate)
        self.sdevice.command(cmd)

    def cmd_position(self, angles):
        cmd = 'g0'
        for ax, pos in zip(self.active_axes, angles):
            cmd += " {}{}".format(ax, pos)
        self.sdevice.command(cmd)

    def go_home(self):
        self.sdevice.command("g0 z0")
        cmd = 'g0'
        for ax in self.active_axes:
            cmd += " {}0".format(ax)
        self.sdevice.command(cmd)

    def test_pose(self, tar_pose):
        xyz_fw, thetaxyz_fw = self.solve_fk(tar_pose)
        errors_pos = [abs(fw - nm) for fw, nm in zip(xyz_fw, tar_pose.position_mm)]
        errors_ang = [abs(fw - nm) for fw, nm in zip(thetaxyz_fw, tar_pose.rotation_xyz_rad)]

        if (sum(errors_pos) + sum(errors_ang)) > 0.1:
            print("POSITION ERRORS: ", errors_pos)
            print("ANGULAR ERRORS: ", errors_ang)
            return False
        return True

    def pthg(self, x, y):
        return pow(pow(x, 2) + pow(y, 2), 0.5)

    def cosines_law(self, a, b, c):
        # Solve for angle A
        numsum = (-pow(a, 2) + pow(b, 2) + pow(c, 2)) / (2 * b * c)
        numsum = max(min(numsum, 1), -1)
        return math.acos(numsum)

# def plot_robot(tar_pose):
#     points = [[0,0,0]]
#     points.append([0, 0, 0 + links_mm[0]])
#     points.append([0, 0, 0 + sum(links_mm[0:2])])

#     nppts = np.array(points)
#     plt.scatter(nppts[:,0], nppts[:,1], s=20, label='xy')
#     plt.scatter(nppts[:,0], nppts[:,2], s=20, label='xz')
#     plt.scatter(nppts[:,1], nppts[:,2], s=20, label='yz')
#     plt.show()

def shakey_shakey():
    joint = 0
    amplitude = 0.2
    cycles = 10
    rbot = Robot()

    p0 = [0,0,0,0,0]
    p1 = [0,0,0,0,0]
    p0[joint] = 2 * amplitude

    for _ in range(cycles):
        rbot.pose_command(p0)
        rbot.pose_command(p1)

def run_not_ik_routine():
    rbot = Robot()

    commands = [
        [0, 0.4, 0, 0, 0],
        [0, 0.6, 0, 0, 0],
        [0, 0.4, 0, 0, 0],
        [0, 0.6, 0, 0, 0],
        [0, 0.4, 0, 0, 0],
        [0, 0.6, 0.6, 0, 0],
        [0, 1.8, 1.8, 0, 0],
        [0, 1.5, 1.5, 0, 0],
        [0, 1.8, 1.8, 0, 0],
        [0, 1.5, 1.5, 0, 0],
        [-1, 1.5, 1.5, 0, 0],
        [1, 1.5, 1.5, 0, 0],
        [0, 0.6, 0.0, 0, 0],
        [0, 0, 0, 0, 0]
    ]

    for c in commands:
        rbot.pose_command(c)

def run_ik_routine():
    # Establish CONNECTION
    rbot = Robot()

    poses = []

    pt1 = pose()
    pt1.position_mm = [0,625,147]
    pt1.rotation_xyz_rad = [0, 0, 0]

    newpt = pose()
    newpt.position_mm = [0,450,250]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,450,300]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,400,300]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,400,250]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,450,250]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,450,300]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,400,300]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    newpt = pose()
    newpt.position_mm = [0,400,400]
    newpt.rotation_xyz_rad = [0, 0, 0]
    poses.append(newpt)

    rbot.moveto(pt1, velocity=2)
    for p in poses:
        rbot.moveto(p, velocity=40)
    time.sleep(1)
    rbot.go_home()

    # pt2 = pose()
    # pt2.position_mm = [0,600,147]
    # pt2.rotation_xyz_rad = [0, 0, 0]
    # pose_steps = generate_line(pt1, pt2, 100)
    # execute_command(pose_steps)
 
# def ik_test(pos_target, rot_target):
#     goal_pose2 = pose()
#     goal_pose2.position_mm = pos_target
#     goal_pose2.rotation_xyz_rad = rot_target

#     tstart = time.time()
#     solve_ik(goal_pose2)
#     print("Runtime: {}".format(time.time() - tstart))
#     print("Joint Angles:", goal_pose2.joint_angles)
#     print(test_pose(goal_pose2))

if __name__ == '__main__':
    run_ik_routine()
    # run_not_ik_routine()
    # shakey_shakey()
    # ik_test([100,500,175], [0, 0, 0])