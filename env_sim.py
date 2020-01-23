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
import subprocess, signal
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

start = 0.0


class Robot:

    def __init__(self, ID, num):
        self.ID = ID
        self.num = num
        self.angles = [np.pi, np.pi / 2.0, 0.0,3*np.pi / 2.0]
        self.dir = 1
        self.r = 0.0975
        self.l = 0.55

        # motors
        self.left_handle = self.get_handle("motor%d_left" % (self.num))
        self.right_handle = self.get_handle("motor%d_right" % (self.num))

        self.ground = self.get_handle("Ground")
        self.robot_handle = self.get_handle("R%d" % (self.num))
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.robot_handle, -1, const_v.simx_opmode_streaming)
        (code, rot) = vrep.simxGetObjectOrientation(self.ID, self.robot_handle, -1, const_v.simx_opmode_streaming)
        self.points = []
        time.sleep(0.1)

    def get_handle(self, name):
        # print(name)
        (check, handle) = vrep.simxGetObjectHandle(self.ID, name, const_v.simx_opmode_oneshot_wait)
        if check != 0:
            print("Couldn't find %s" % name)
        return handle

    def get_position(self):
        (code, pose) = vrep.simxGetObjectPosition(self.ID, self.robot_handle, -1, const_v.simx_opmode_buffer)
        return np.array(pose)

    def get_phi(self):
        (code, rot) = vrep.simxGetObjectOrientation(self.ID, self.robot_handle, -1, const_v.simx_opmode_buffer)
        phi = rot[2]
        return phi

    def position_xy(self):
        pose = self.get_position()
        x = (pose[0] + 4.5)+0.1
        y = (pose[1] + 4.5)+0.1
        return x, y

    def rotate(self, dir):
        self.dir=dir
        angle = self.angles[int(dir)]
        phi = self.get_phi()
        if phi < 0:
            phi += 2 * np.pi
        if phi < angle:
            tmp = angle
            angle = phi
            phi = tmp
        if phi - angle < angle + (2 * np.pi - phi):
            err = -(phi - angle)
        else:
            err = angle + 2 * np.pi - phi
        global start
        dt = 1.0 - (time.time() - start)
        w = err * self.l / (4.0 * self.r)
        if abs(w) > 0.1:
            self.move_robot(-w, w)

    def goto_xy(self, x, y):
        xr, yr = self.position_xy()
        if self.dir == 0 or self.dir == 2:
            dis = x - xr
        else:
            dis = y - yr
        # print(x,xr,y,yr)
        global start
        dt = 1.0 - (time.time() - start)
        w = dis / (self.r)
        self.move_robot(w / 2, w / 2)
        if abs(w) < 0.05:
            self.move_robot(0.0, 0.0)

    def get_image(self, cam_handle):
        (code, res, im) = vrep.simxGetVisionSensorImage(self.ID, cam_handle, 0, const_v.simx_opmode_buffer)
        img = np.array(im, dtype=np.uint8)
        img.resize([res[0], res[1], 3])
        img = cv2.flip(img, 0)
        return img

    def move_robot(self, v_left, v_right):
        code = vrep.simxSetJointTargetVelocity(self.ID, self.left_handle, v_left, const_v.simx_opmode_streaming)
        code = vrep.simxSetJointTargetVelocity(self.ID, self.right_handle, v_right, const_v.simx_opmode_streaming)


file = open("/home/ali/PycharmProjects/Multi/in.txt", "r")
num_robots = 5
R = []
for i in range(num_robots):
    R.append(Robot(ID, i + 1))

# while line!=None:
robot_points = []
for line in file.read().split('\n'):
    numbers = []
    if len(line)==0:
        break
    for y in line.split(' '):
        if y:
            numbers.append(float(y))
    for i in range(num_robots):
        R[i].points.append(np.array([numbers[4 * i + 1]/1000.0,numbers[4 * i + 2]/1000.0,int(numbers[4*i+3])]))
d=[]
for i in range(num_robots):
    d.append(R[i].points[0][2])

for p in range(len(R[0].points)):
    start = time.time()
    for i in range(5):
        if abs(R[i].points[p][2]-d[i])>0.1:
            # print("rot")
            R[i].rotate(R[i].points[p][2])
        else:
            R[i].goto_xy(i + R[i].points[p][0], R[i].points[p][1])
        d[i] = R[i].points[p][2]
    total_time = time.time() - start
    time.sleep(1.0 - total_time)
for r in R:
    r.move_robot(0.0,0.0)
# time.sleep(10)
