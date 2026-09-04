import camera_tools
refposition_sampletable = (0.065, -0.355, 0.183)
refposition_cleanstation = (-0.138, -0.363, 0.390)

def script1(robot):
    print("Running UserScript1")
    camera_tools.search_apriltag_by_tilt(robot, ref_pos=refposition_sampletable)
    robot.put_tcp2camera()

    robot.grab()
    robot.bump(z=-1,backoff=0.1)


def script(robot):
    print("Running UserScript1")
    camera_tools.search_apriltag_by_tilt(robot, ref_pos=refposition_cleanstation)
    robot.put_tcp2camera()

    robot.grab()
    robot.bump(z=-1,backoff=0.1)