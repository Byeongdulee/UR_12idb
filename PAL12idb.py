import camera_tools
ref_mixer_cleantable = [0.4, 0.1, 0.1, 2.231, -2.212, 0]
ref_cleanstation = [0.38, -0.16, -0.1, 2.231, -2.212, 0]
ref_sampletable = [-0.22, -0.37, 0.12, -2.18860535, 2.25379435, 0]
distance_gripper_tag = 0.05
# move sth out by 50 mm
# move sav down by 30 mm
# after grap, have to move 0.20 m up to clear the needle.
sample_table = []
cleaning_station1 = []
cleaning_station2 = []
mixer_cleaning_station = []
mixer_station = []
needle_clear_height = 0.20
mixer_height = 0.05
grab_depth = 0.01
flowcell_ID = 1

# Basic operation functions
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

def transport(robot, p1, p2, height = needle_clear_height):
    # picking up the flowcell at p1 and dropping it at p2. The robot is assumed to be empty.
    # assuming robot is empty
    robot.activate_gripper()
    robot.moveto(p1)
    pickup(robot)
    # Z position should be the needle cleared position. Work on a copy: writing
    # p2[2] in place edits the caller's list, so a taught position passed in
    # (sample_table / cleaning_station) would creep upward on every transport.
    p2 = list(p2)
    p2[2] = p2[2]-distance_gripper_tag+height
    robot.moveto(p2)
    dropdown(robot)
# Actual transport functions. These functions are used to move the flowcell between the cleaning station, mixer station, and sample table.
# mixer head to its cleaning station. The robot is assumed to be empty.
def mixer2cleaningstation(robot):
    if flowcell_ID == 1:
        cleaning_station = cleaning_station1
    else:
        cleaning_station = cleaning_station2
    transport(robot, mixer_station, cleaning_station, height=mixer_height)

# mixer head to the mixer station. The robot is assumed to be empty.
def mixer2mixingstation(robot):
    if flowcell_ID == 1:
        cleaning_station = cleaning_station1
    else:
        cleaning_station = cleaning_station2
    transport(robot, cleaning_station, mixer_station, height=mixer_height)

# Bring the flowcell parked at the cleaning station to the beam, ready for data collection. The robot is assumed to be empty.
# This is for measuring water background. The flowcell is not loaded with sample.
def load_flowcell_from_cleaningstation_to_beam(robot):
    if flowcell_ID == 1:
        cleaning_station = cleaning_station1
    else:
        cleaning_station = cleaning_station2
    transport(robot, cleaning_station, sample_table, height=needle_clear_height)

# Bring the flowcell from the beam to the cleaning station. 
def load_flowcell_from_beam_to_cleaningstation(robot):
    if flowcell_ID == 1:
        cleaning_station = cleaning_station1
    else:
        cleaning_station = cleaning_station2
    transport(robot, sample_table, cleaning_station, height=needle_clear_height)

# Bring the flowcell parked at the cleaning station to the mixing station, 
# Ready to draw solution from the mixer. The robot is assumed to be empty.
def ready_flowcell_to_draw(robot):
    if flowcell_ID == 1:
        cleaning_station = cleaning_station1
    else:
        cleaning_station = cleaning_station2
    # draw solution and put it in the beam
    # assuming the robot is empty.
    robot.moveto(cleaning_station)
    pickup(robot)
    p2 = list(mixer_station)
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    robot.bump(z=-1,backoff=0.002)

# after drawing, the robot is holding the flowcell. Move it to the beam and drop it.
def load_sample_to_beam(robot):
    robot.mvr2z(needle_clear_height)
    p2 = list(sample_table)
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    dropdown(robot)

# after data collection, pick up the flowcell from the beam and move it to the mixer to aspirate.
def return_sample(robot):
    # move up to the sample table height.
    p = robot.get_pos()
    p[2] = sample_table[2]
    robot.moveto(p)
    # move to the sample table
    robot.moveto(sample_table)
    robot.pickup()
    p2 = list(mixer_station)
    p2[2] = p2[2]-distance_gripper_tag+needle_clear_height
    robot.moveto(p2)
    robot.bump(z=-1,backoff=0.005)

# after aspirating, move the flowcell to the cleaning station and drop it.
def wash_flowcell_after_return(robot):
    if flowcell_ID == 1:
        cleaning_station = cleaning_station1
    else:
        cleaning_station = cleaning_station2
    robot.mvr2z(needle_clear_height)
    robot.moveto(cleaning_station)
    dropdown(robot, height=mixer_height)

## Configuration functions. These functions are used to locate the positions of the sample table and cleaning station using AprilTags.
def locate_apriltag(robot, pos = ''):
    # Record the taught position of a station. Returns the pose it found, and
    # also stores it in sample_table / cleaning_station.
    global sample_table, cleaning_station1, cleaning_station2, mixer_cleaning_station, mixer_station
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
        if flowcell_ID == 1:
            cleaning_station1 = v
        else:
            cleaning_station2 = v   
    if pos == 'mixer_cleaning_station':
        mixer_cleaning_station = v
    if pos == 'mixer_station':
        mixer_station = v
    return v
