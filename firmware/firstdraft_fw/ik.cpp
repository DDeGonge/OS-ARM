/*
#include <Arduino.h>
#include <math.h>
#include "ik.h"

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

static uint16_t samplerate_hz = 100;
static char axes = ['x','y','z','a','b'];

struct pose
{
    pose(float[] positions, float[] rotations);
    apply_limits(float[] a_min, float []a_max);

    position_mm = [0,0,0];
    rotation_xyz_rad = [0,0,0];
    joint_angles[] = [0,0,0,0,0,0];
};

pose::pose(float[] positions, float[] rotations)
{
    self.position_mm = [0,0,0] if positions is None else positions
    self.rotation_xyz_rad = [0,0,0] if rotations is None else rotations
    self.joint_angles = [0,0,0,0,0,0]
}

void pose::apply_limits(float[] a_min, float []a_max)
{
    new_angles = []
    for j, amin, amax in zip(self.joint_angles, a_min, a_max):
        new_angles.append(max(min(j, amax), amin))
    self.joint_angles = new_angles
}

struct Robot
{
    Robot();

    char active_axes[] = ['x','y','z','a','b'];
    float angles_min_rad[] = [-3.0, 0.0, 0.0, -1.5, -3.0, -1.5];
    float angles_max_rad[] = [ 3.0, 2.0, 2.0,  1.5,  3.0,  1.5];
    float links_mm[] = [6.0, 140.82, 250.0, 250.0, 83.5, 50.0];
}

Robot::Robot()
{
    last_pose = pose();
    update_pose();
    delay(100);
}

void Robot::solve_ik(pose tar_pose)
{
    [x, y, z] = tar_pose.position_mm;

    // Rotations are in link J3 coordinate frame to make my life easier
    // Start with J0 cuz it's fully defined by x and y only, #ez
    a0 = math.atan2(x, y)

    // Also J4 is just rotation about x
    a4 = tar_pose.rotation_xyz_rad[0]

    // Calculate where J3 needs to be in xyz space
    end_disp_xy = sum(self.links_mm[4:6]) * math.cos(tar_pose.rotation_xyz_rad[1])
    end_disp_z  = sum(self.links_mm[4:6]) * math.sin(tar_pose.rotation_xyz_rad[1])
    j3pos = [x - (end_disp_xy * math.sin(a0)), y - (end_disp_xy * math.cos(a0)), z - end_disp_z]

    if j3pos[2] < 0:
        print("UHOH bad news bears my friend")

    // Finally calculate J1 and J2
    // on [xy, z] plane
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
}

        

void Robot::solve_fk(self, tar_pose):
    a = tar_pose.joint_angles
    hinges_xylen_mm = self.links_mm[2]*math.cos(a[1]) + self.links_mm[3]*math.cos(a[1] - a[2]) + (self.links_mm[4]+ self.links_mm[5]) * math.cos(a[1] - a[2] + a[3])
    xpos = hinges_xylen_mm * math.sin(a[0])
    ypos = hinges_xylen_mm * math.cos(a[0])
    zpos = self.links_mm[0] + self.links_mm[1] + self.links_mm[2]*math.sin(a[1]) + self.links_mm[3]*math.sin(a[1] - a[2]) + (self.links_mm[4]+ self.links_mm[5]) * math.sin(a[1] - a[2] + a[3])
    xang = a[4]
    yang = a[1] - a[2] + a[3]
    zang = 0.0
    return [xpos, ypos, zpos], [xang, yang, zang]

void Robot::update_pose(self):
    positions, angles = self.solve_fk(self.last_pose)
    self.last_pose.position_mm = positions
    self.last_pose.rotation_xyz_rad = angles

void Robot::moveto(self, newpose, velocity, acceleration=0):
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

void Robot::zero(self):
    cmd = 'g92'
    for ax in self.active_axes:
        cmd += " {}".format(ax)
    # print(cmd)
    self.sdevice.command(cmd)

void Robot::go_home(self):
    self.sdevice.command("g0 z0")
    cmd = 'g0'
    for ax in self.active_axes:
        cmd += " {}0".format(ax)
    self.sdevice.command(cmd)

void Robot::test_pose(self, tar_pose):
    xyz_fw, thetaxyz_fw = self.solve_fk(tar_pose)
    errors_pos = [abs(fw - nm) for fw, nm in zip(xyz_fw, tar_pose.position_mm)]
    errors_ang = [abs(fw - nm) for fw, nm in zip(thetaxyz_fw, tar_pose.rotation_xyz_rad)]

    if (sum(errors_pos) + sum(errors_ang)) > 0.1:
        print("POSITION ERRORS: ", errors_pos)
        print("ANGULAR ERRORS: ", errors_ang)
        return False
    return True

void Robot::pthg(self, x, y):
    return pow(pow(x, 2) + pow(y, 2), 0.5)

void Robot::cosines_law(self, a, b, c):
    # Solve for angle A
    numsum = (-pow(a, 2) + pow(b, 2) + pow(c, 2)) / (2 * b * c)
    numsum = max(min(numsum, 1), -1)
    return math.acos(numsum)

*/
