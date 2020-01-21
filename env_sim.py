import vrep.vrep as vrep
import vrep.vrepConst as const_v
import time
import sys
import numpy as np
import cv2
import math
import os
import gym
from tqdm import trange
import subprocess,signal
from threading import Thread

p = subprocess.Popen(['ps', '-A'], stdout=subprocess.PIPE)
out, err = p.communicate()
for line in out.splitlines():
    if b'vrep' in line:
        pid = int(line.split(None, -1)[0])
        os.kill(pid, signal.SIGKILL)

vrep_root = "/home/ali/Downloads/VREP"
scene_file = "/home/ali/PycharmProjects/Multi/multi.ttt"

os.chdir(vrep_root)
os.system("./vrep.sh -s " + scene_file + " &")

vrep.simxFinish(-1)
time.sleep(1)

# get the ID of the running simulation
ID = vrep.simxStart('127.0.0.1', 19999, True, False, 5000, 5)
# check the connection
if ID != -1:
    print("Connected")
else:
    sys.exit("Error")


class Robot:

    def __init__(self, ID, num):
        self.ID = ID
        self.num = num

        # motors
        self.left_handle = self.get_handle("motor%d_left" % (self.num))
        self.right_handle = self.get_handle("motor%d_right" % (self.num))

        self.ground = self.get_handle("Ground")
        self.robot_handle = self.get_handle("R%d" % (self.num))
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.robot_handle, -1, const_v.simx_opmode_streaming)
        time.sleep(0.1)

    def get_handle(self, name):
        (check, handle) = vrep.simxGetObjectHandle(self.ID, name, const_v.simx_opmode_oneshot_wait)
        if check != 0:
            print("Couldn't find %s" % name)
        return handle

    def get_position(self, handle):
        (code, pose) = vrep.simxGetObjectPosition(self.ID, handle, -1, const_v.simx_opmode_buffer)
        return np.array(pose)

    def position_xy(self):
        pose = self.get_position(self.robot_handle)
        x = (pose[0] + 2.25) /0.5
        y = (pose[1] + 2.25) /0.5
        return x,y

    def goto_xy(self,x,y):
        x_= x*0.5-2.25
        y_=y*0.5-2.25
        z_=0.1388
        pose=[x_,y_,z_]
        code=vrep.simxSetObjectPosition(self.ID,self.robot_handle,-1,pose,const_v.simx_opmode_oneshot)

    def get_image(self, cam_handle):
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, cam_handle, 0, const_v.simx_opmode_buffer)
        # print(code)
        img = np.array(im, dtype=np.uint8)
        img.resize([res[0], res[1], 3])
        img = cv2.flip(img, 0)
        return img

    def move_robot(self, v_left, v_right):
        code = vrep.simxSetJointTargetVelocity(self.ID, self.left_handle, v_left, const_v.simx_opmode_streaming)
        code = vrep.simxSetJointTargetVelocity(self.ID, self.right_handle, v_right, const_v.simx_opmode_streaming)


num_robots = 5
R = []
for i in range(num_robots):
    R.append(Robot(ID, i + 1))
    x,y=R[i].position_xy()

total_time_steps=10
times=trange(total_time_steps)
for _ in times:
    for i in range(num_robots):
        R[i].goto_xy(i,i)
        time.sleep(0.5)