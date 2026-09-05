import camera_tools
ref_mixer_cleantable = [0.4, 0.1, 0.1, 2.231, -2.212, 0]
ref_cleanstation = [0.38, -0.16, -0.1, 2.231, -2.212, 0]
ref_sampletable = [-0.22, -0.37, 0.12, -2.18860535, 2.25379435, 0]
distance_gripper_tag = 0.05
# move sth out by 50 mm
# move sav down by 30 mm
# after grap, have to move 0.20 m up to clear the needle.
sample_table = []
cleaning_station = []
mixer_cleaning_station = []
mixer_station = []
needle_clear_height = 0.20
mixer_height = 0.05
grab_depth = 0.01

def locate_apriltag(robot, pos = ''):
    # Record the taught position of a station. Returns the pose it found, and
    # also stores it in sample_table / cleaning_station.
    global sample_table, cleaning_station, mixer_cleaning_station, mixer_station
    ref_pos = []
    if pos == 'sample_table':
        ref_pos = ref_sampletable
    if pos == 'cleaning_station':
        ref_pos = ref_cleanstation
    if pos == 'mixer_cleaning_station':
        ref_pos = ref_mixer_cleantable
    if pos == 'mixer_station':
        ref_pos = ref_mixer_cleantable
    print(f"Looking for {pos} ....")
    if len(ref_pos)==0:
        ref_pos = ref_sampletable
    camera_tools.search_apriltag_by_tilt(robot, ref_pos=ref_pos)
    robot.put_tcp2camera()

    robot.grab()
    robot.bump(z=-1,backoff=distance_gripper_tag)
    robot.set_tcp(robot.tcp)
    p = robot.get_pose()
    v = p.get_pose_vector().tolist()
    if pos == 'sample_table':
        sample_table = v
    if pos == 'cleaning_station':
        cleaning_station = v
    if pos == 'mixer_cleaning_station':
        mixer_cleaning_station = v
    if pos == 'mixer_station':
        mixer_station = v
    return v

def pickup(robot, height = needle_clear_height):
    robot.release()
    # cm go deeper from the standard height
    robot.mvr2z(-grab_depth-distance_gripper_tag)
    robot.grab()
    # move high enough so the needle is cleared
    robot.mvr2z(height)

def dropdown(robot, height = needle_clear_height):
    # move down
    robot.mvr2z(-1*(height+grab_depth-0.005))
    robot.release()
    # move back up to standard height
    robot.mvr2z(distance_gripper_tag)

def test_pickup(robot, height=needle_clear_height):
    pickup(robot, height=height)
    dropdown(robot, height=height)

def test_mixer_pickup(robot):
    test_pickup(robot, height=mixer_height)

def goto_default(robot):
    robot.moveto(ref_mixer_cleantable)

def transport(robot, p1, p2):
    # assuming robot is empty
    robot.activate_gripper()
    robot.moveto(p1)
    pickup(robot)
    # Z position should be the needle cleared position. Work on a copy: writing
    # p2[2] in place edits the caller's list, so a taught position passed in
    # (sample_table / cleaning_station) would creep upward on every transport.
    p2 = list(p2)
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    dropdown(robot)