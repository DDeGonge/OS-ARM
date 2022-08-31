

# base to J0, J0 to J1 .... etc
link_lens = [5, 100, 250, 250, 50, 50]

# For reference only
# Joints
# 0 - Rotary
# 1 - Hinge
# 2 - Hinge
# 3 - Hinge
# 4 - Rotary
# 5 - Idk

class robot_pose:
    def __init__(self, xpos, ypos, zpos, xrot, yrot, zrot):
        self.xpos = xpos
        self.ypos = ypos
        self.zpos = zpos
        self.xrot = xrot
        self.yrot = yrot
        self.zrot = zrot

class joint_angles:
    def __init__(self, j0, j1, j2, j3, j4, j5):
        self.j0 = j0
        self.j1 = j1
        self.j2 = j2
        self.j3 = j3
        self.j4 = j4
        self.j5 = j5


def calc_ik(rpose: robot_pose):
    # Alright here's the plan
    # Do top-down vision IK first
    # Then calc J3
    # Then calc the rest of the hinges
    # Then check yoself idk
    pass

def forward_ik(jangles: joint_angles):
    pass
