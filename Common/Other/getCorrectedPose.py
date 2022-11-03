import numpy as np
from scipy.spatial.transform import *
import random

def getCorrectedPose(center_offset, pre_pose):
    r = Rotation.from_euler('xyz', pre_pose[0:3], degrees=True)
    rotm = r.as_matrix()
    t = np.array(pre_pose[3:6]).reshape(3,1) - np.array(center_offset).reshape(3,1) + np.dot(rotm, np.array(center_offset).reshape(3,1))
    return [pre_pose[0], pre_pose[1], pre_pose[2], t[0,0], t[1,0], t[2,0]]

# deltax、deltay、deltaz为新旋转中心相对于原旋转中心的偏移量
# rx、ry、rz、tx、ty、tz为原旋转中心下pose

# getCorrectedPose求新旋转中心下对应pose
# 欧拉角旋转顺序是先X再Y最后Z,scipy库里定义与matlab刚好相反
	
deltax = random.uniform(-5, 5)
deltay = random.uniform(-5, 5)
deltaz = random.uniform(-5, 5)
rx = random.uniform(-90, 90)
ry = random.uniform(-90, 90)
rz = random.uniform(-90, 90)
tx = random.uniform(-10, 10)
ty = random.uniform(-10, 10)
tz = random.uniform(-10, 10)
pose = getCorrectedPose([cx, cy, cz], [rx, ry, rz, tx, ty, tz])
print("pose = ", pose)