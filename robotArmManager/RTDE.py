import urx
import time
import numpy as np


class RobotController():
    def __init__(self, a, v):
        self.curDest = np.array([0, -np.pi/2, 0, -np.pi/2, 0, 0]).astype(float)
        self.rob = None
        self.a = a
        self.v = v

        try: 
            print("Initializing Robot, A = {}, V = {}".format(self.a, self.v))
            self.rob = urx.Robot("169.254.121.1")
            print("here 1")
            self.rob.set_tcp((0, 0, 0.1, 0, 0, 0))
            print("here 2")
            self.rob.set_payload(2, (0, 0, 0.1))
            print("here")
            self.rob.movej((0, -np.pi/2, 0, -np.pi/2, 0, 0), 1, 1, wait=False)
            print("Moving")
            time.sleep(5)
            print("ready to run")
        except:
            print("Something went wrong1")
            if(self.rob is not None):
                self.rob.close()
            quit()
        
    def isSafe(self, command):
        safe = True
        if(command[0] < -6.1 or command[0] > 6.1):
            safe = False
            print('unsafe 0')
        if(command[1] < -3.6 or command[1] > 0.5):
            safe = False
            print('unsafe 1')
        if(command[2] < -2.5 or command[2] > 2.5):
            safe = False
            print('unsafe 2')
        # if(command[3] < -3.1):
        #     if(command[4] < -0.87 or command[4] > 2.6):
        #         safe = False
        #         print('unsafe 34 c1')
        # if(command[3] >  0):
        #     if(command[4] < -2.6 or command[4] > 0.87):
        #         safe = False
        #         print('unsafe 34 c2')
        
        # if seg[4] rotated + or - 90 deg. cannot rotate seg[3] more than 90
        #TODO: add more safety tests (illegal position, hits itself [might not be able to do that], unsafe position [hitting robot console], etc)
        return safe

    def controlRobot(self, command):
        if(self.isSafe(command)):
            if(not(np.allclose(command, self.curDest))):
                self.curDest = np.copy(command)
                self.rob.stopj()
                time.sleep(0.1)
                self.rob.movej(command, self.a, self.v, wait=False)
        else:
            print("Unsafe command, canceled")
        time.sleep(1)

    def mainLoop(self):
        try:
            f = open("log.txt", "r")
            line = f.readline().rstrip()
            f.close()
            text = np.array([float(i) for i in line.split()])
            command = text[:6]
            txt = np.round(text,4)
            print(txt[:6],txt[6:])
            self.controlRobot(command)
        except:
            print("Something went wrong")
            if(self.rob is not None):
                self.rob.stopj()
                time.sleep(0.1)
                self.rob.movej((0, -np.pi/2, 0, -np.pi/2, 0, 0), 1, 1, wait=False)
                self.rob.close()
            quit()


if __name__ == "__main__":
    a = 1
    v = 0.4
    rob = RobotController(a, v)
    try:
        while True:
            rob.mainLoop()
    except KeyboardInterrupt:
        print('interrupted')


# 0 -1.57 0 -1.57 0 0