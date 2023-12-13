import urx
import logging
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)

    rob = urx.Robot("10.10.10.3", use_rt=True)
    robotiqgrip = Robotiq_Two_Finger_Gripper(rob, socket_host="10.10.10.3")

    rob.set_tcp((0,0,0,0,0,0))

    rob.set_payload(0.5, (0,0,0))
    try:
        l = 0.05
        v = 0.05
        a = 0.2
        pose = rob.getl()
        print("robot tcp is at: ", pose)
        print("absolute move in base coordinate ")
        pose[2] += l
        rob.movel(pose, acc=a, vel=v)
        print("relative move in base coordinate ")
        robotiqgrip.close_gripper()
        robotiqgrip.open_gripper()

        rob.translate((0, 0, -l), acc=a, vel=v)
        print("relative move back and forth in tool coordinate")
        rob.translate_tool((0, 0, -l), acc=a, vel=v)
        rob.translate_tool((0, 0, l), acc=a, vel=v)
        robotiqgrip.close_gripper()
        robotiqgrip.open_gripper()
    finally:
        rob.close()