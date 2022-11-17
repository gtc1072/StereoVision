from pyquaternion import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class DualQuaternion_gtc():
    def __init__(self, *args, **kwargs):
        s = len(args)
        if s == 0:
            if kwargs:
                if ("matrix" in kwargs):
                    self.dq = DualQuaternion_gtc.from_matrix(kwargs["matrix"]).dq
                elif ("pose" in kwargs) or ("order" in kwargs):
                    matrix = self.from_pose_to_matrix(kwargs["pose"], kwargs["order"])
                    # matrix = self.from_pose_to_matrix(kwargs["pose"], 'zyx')
                    self.dq = DualQuaternion_gtc.from_matrix(matrix).dq
                elif ("axis" in kwargs) or ("angle" in kwargs) or ("translation" in kwargs):
                    self.dq = DualQuaternion_gtc.from_axis_angle_translation(kwargs["axis"],kwargs["angle"],kwargs["translation"]).dq
                elif ("euler" in kwargs) or ("order" in kwargs) or ("translation" in kwargs):
                    matrix = self.from_euler_translation_to_matrix(kwargs["euler"], kwargs["order"], kwargs["translation"])
                    # matrix = self.from_euler_translation_to_matrix(kwargs["euler"],'zyx',kwargs["translation"])
                    self.dq = DualQuaternion_gtc.from_matrix(matrix).dq
                else:
                    self.dq = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            else:
                self.dq = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        elif s == 1:
            if isinstance(args[0], DualQuaternion_gtc):
                self.dq = args[0].dq
                return
            if args[0] is None:
                raise TypeError("Object cannot be initialised from {}".format(type(args[0])))
            self.dq = self._validate_number_sequence(args[0], 8)
        else:
            self.dq = self._validate_number_sequence(args, 8)

    @classmethod
    def from_matrix(cls, m):
        try:
            shape = m.shape
        except AttributeError:
            raise TypeError("Invalid matrix type: Input must be a 3x3 or 4x4 numpy array or matrix")
        if shape == (4, 4):
            R = m[:-1][:, :-1]
            q0 = Quaternion._from_matrix(R)
            q1 = Quaternion(scalar=0.0, vector=[m[0][3],m[1][3],m[2][3]])
            qe = (q1 * q0)/2.0
            return cls(q0.w, q0.x, q0.y, q0.z, qe.w, qe.x, qe.y, qe.z)
        else:
            raise ValueError("Invalid matrix size: Input must be a 3x3 or 4x4 numpy array or matrix")

    @classmethod
    def from_axis_angle_translation(cls, axis, angle, translation):
        q0 = Quaternion._from_axis_angle(axis, angle / 180.0 * 3.1415926)
        q1 = Quaternion(scalar=0.0, vector=[translation[0], translation[1], translation[2]])
        qe = (q1 * q0) / 2.0
        return cls(q0.w, q0.x, q0.y, q0.z, qe.w, qe.x, qe.y, qe.z)

    @staticmethod
    def from_pose_to_matrix(pose, order):
        RMatrix=R.identity(3)
        if order == 'zyx':
            RMatrix = R.from_euler(order, [pose[2], pose[1], pose[0]], degrees=True)
        elif order == 'xyz':
            RMatrix = R.from_euler(order, [pose[0], pose[1], pose[2]], degrees=True)
        # RMatrix = R.from_euler(order, [pose[2]/180.0*3.1415926,pose[1]/180.0*3.1415926,pose[0]/180.0*3.1415926])
        matrix = np.identity(4)
        matrix[:-1,:-1]=RMatrix.as_matrix()
        matrix[0,3]=pose[3]
        matrix[1,3]=pose[4]
        matrix[2,3]=pose[5]
        return matrix

    @staticmethod
    def from_euler_translation_to_matrix(euler, order, translation):
        RMatrix=R.identity(3)
        if order == 'zyx':
            RMatrix = R.from_euler(order, [euler[2], euler[1], euler[0]], degrees=True)
        elif order == 'xyz':
            RMatrix = R.from_euler(order, [euler[0], euler[1], euler[2]], degrees=True)
        matrix = np.identity(4)
        matrix[:-1, :-1] = RMatrix
        matrix[0, 3] = translation[0]
        matrix[1, 3] = translation[1]
        matrix[2, 3] = translation[2]
        return matrix

    def _validate_number_sequence(self, seq, n):
        if seq is None:
            return np.zeros(n)
        if len(seq) == n:
            try:
                l = [float(e) for e in seq]
            except ValueError:
                raise ValueError("One or more elements in sequence <{!r}> cannot be interpreted as a real number".format(seq))
            else:
                return np.asarray(l)
        elif len(seq) == 0:
            return np.zeros(n)
        else:
            raise ValueError("Unexpected number of elements in sequence. Got: {}, Expected: {}.".format(len(seq), n))

    def matrix2euler(slef, m, order='xyz'):
        d = np.clip
        e = m.reshape(-1)
        a = e[0]
        f = e[1]
        g = e[2]
        h = e[3]
        k = e[4]
        l = e[5]
        m = e[6]
        n = e[7]
        e = e[8]
        if "xyz" == order:
            y = np.arcsin(d(g, -1, 1))
            if 0.99999 > np.abs(g):
                x = np.arctan2(- l, e)
                z = np.arctan2(- f, a)
            else:
                x = np.arctan2(n, k)
                z = 0
        elif "yxz" == order:
            x = np.arcsin(- d(l, -1, 1))
            if 0.99999 > np.abs(l):
                y = np.arctan2(g, e)
                z = np.arctan2(h, k)
            else:
                y = np.arctan2(- m, a)
                z = 0
        elif "zxy" == order:
            x = np.arcsin(d(n, -1, 1))
            if 0.99999 > np.abs(n):
                y = np.arctan2(- m, e)
                z = np.arctan2(- f, k)
            else:
                y = 0
                z = np.arctan2(h, a)
        elif "zyx" == order:
            y = np.arcsin(- d(m, -1, 1))
            if 0.99999 > np.abs(m):
                x = np.arctan2(n, e)
                z = np.arctan2(h, a)
            else:
                x = 0
                z = np.arctan2(- f, k)
        elif "yzx" == order:
            z = np.arcsin(d(h, -1, 1))
            if 0.99999 > np.abs(h):
                x = np.arctan2(- l, k)
                y = np.arctan2(- m, a)
            else:
                x = 0
                y = np.arctan2(g, e)
        elif "xzy" == order:
            z = np.arcsin(- d(f, -1, 1))
            if 0.99999 > np.abs(f):
                x = np.arctan2(n, k)
                y = np.arctan2(g, a)
            else:
                x = np.arctan2(- l, e)
                y = 0
        else:
            print("not support order", order)
            return None
        return np.array([x/3.1415926*180, y/3.1415926*180, z/3.1415926*180], dtype=np.float32)

    def quaternion2matrix(self, quaternion):
        a = quaternion[0]
        b = quaternion[1]
        c = quaternion[2]
        d = quaternion[3]
        m = np.array([[1 - 2 * (c * c + d * d), 2 * b * c - 2 * a * d, 2 * a * c + 2 * b * d],
                      [2 * b * c + 2 * a * d, 1 - 2 * (b * b + d * d), 2 * c * d - 2 * a * b],
                      [2 * b * d - 2 * a * c, 2 * a * b + 2 * c * d, 1 - 2 * (b * b + c * c)]], dtype=np.float32)
        return m

    def quaternion2euler(self, quaternion, order='xyz'):
        m = self.quaternion2matrix(quaternion)
        euler_angle = self.matrix2euler(m, order=order)
        return euler_angle

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return np.array([roll_x/3.1415926*180, pitch_y/3.1415926*180, yaw_z/3.1415926*180])  # in radians

    def as_pose(self, order):
        q0 = Quaternion(self.dq[0:4])
        qe = Quaternion(self.dq[4:8])
        # angle = self.quaternion2euler([q0.w, q0.x, q0.y, q0.z], order)
        angle = self.euler_from_quaternion(q0.x, q0.y, q0.z, q0.w)
        # angle = R.from_quat([q0.w, q0.x, q0.y, q0.z]).as_euler(order, degrees=True)
        q = qe * q0.conjugate
        pose=[]
        if order == 'zyx':
            pose=[angle[0], angle[1], angle[2], q.x * 2.0, q.y * 2.0, q.z * 2.0]
        elif order == 'xyz':
            pose=[angle[0], angle[1], angle[2], q.x * 2.0, q.y * 2.0, q.z * 2.0]
        return pose

    def as_axis_angle_translation(self):
        pass

    def as_matrix(self):
        pass


    @staticmethod
    def multiply(dq0, dq1):
        q00 = Quaternion(dq0.dq[0:4])
        q0e = Quaternion(dq0.dq[4:8])
        q10 = Quaternion(dq1.dq[0:4])
        q1e = Quaternion(dq1.dq[4:8])
        q0 = q00 * q10
        qe = q00 * q1e + q0e * q10
        return DualQuaternion_gtc([q0.w, q0.x, q0.y, q0.z, qe.w, qe.x, qe.y, qe.z])

    @staticmethod
    def conjugate(q):
        return DualQuaternion_gtc([q.dq[0],-q.dq[1],-q.dq[2],-q.dq[3],q.dq[4],-q.dq[5],-q.dq[6],-q.dq[7]])

    @staticmethod
    def getScrewPara(q):
        normA = np.sqrt(q.dq[1]**2 + q.dq[2]**2 + q.dq[3]**2)
        # pure translation
        if(normA < 1e-15):
            l = np.asarray([q.dq[5], q.dq[6], q.dq[7]])
            m = np.asarray([0, 0, 0])
            ang = np.asarray([0, 2.0 * np.sqrt(q.dq[5]**2 + q.dq[6]**2 + q.dq[7]**2)])
            return (normA, l, m, ang)
        l = np.asarray([q.dq[1] / normA, q.dq[2] / normA, q.dq[3] / normA])
        ang = np.asarray([2.0 * math.atan2(normA, q.dq[0]), -2.0 * q.dq[4] / normA])
        B = np.asarray([q.dq[5], q.dq[6], q.dq[7]])
        m = q.dq[0] * q.dq[4] / normA**2 * l + 1.0 / normA * B
        return (normA, l, m, ang)

    @staticmethod
    def setScrewPara(l, m, theta, alpha):
        cosa = math.cos(theta / 2.0)
        sina = math.sin(theta / 2.0)
        A = sina * l
        b = -alpha / 2 * sina
        B = sina * m + alpha / 2 * cosa * l
        dq = DualQuaternion_gtc([cosa, A[0], A[1], A[2], b, B[0], B[1], B[2]])
        return DualQuaternion_gtc.normalise(dq)

    @staticmethod
    def normalise(q):
        a = q.dq[0]**2 + q.dq[1]**2 + q.dq[2]**2 + q.dq[3]**2
        b = 2.0 * (q.dq[0] * q.dq[4] + q.dq[1] * q.dq[5] + q.dq[2] * q.dq[6] + q.dq[3] * q.dq[7])

        a = math.sqrt(a)
        b = b / 2 / a

        a = 1.0 / a
        b = -b * a * a

        tq = np.zeros(8)

        tq[5] = q.dq[5] * a + q.dq[1] * b
        tq[6] = q.dq[6] * a + q.dq[2] * b
        tq[7] = q.dq[7] * a + q.dq[3] * b
        tq[4] = q.dq[0] * b + q.dq[4] * a

        tq[0] = a * q.dq[0]
        tq[1] = a * q.dq[1]
        tq[2] = a * q.dq[2]
        tq[3] = a * q.dq[3]

        return DualQuaternion_gtc(tq)

    @staticmethod
    def invert(q):
        a = q.dq[0] ** 2 + q.dq[1] ** 2 + q.dq[2] ** 2 + q.dq[3] ** 2
        b = 2.0 * (q.dq[0] * q.dq[4] + q.dq[1] * q.dq[5] + q.dq[2] * q.dq[6] + q.dq[3] * q.dq[7])

        a = math.sqrt(a)
        b = b / 2 / a

        a = 1.0 / a
        b = -b * a * a

        tq = np.zeros(8)

        tq[5] = -q.dq[5] * a - q.dq[1] * b
        tq[6] = -q.dq[6] * a - q.dq[2] * b
        tq[7] = -q.dq[7] * a - q.dq[3] * b
        tq[4] = q.dq[0] * b + q.dq[4] * a

        tq[0] = a * q.dq[0]
        tq[1] = -a * q.dq[1]
        tq[2] = -a * q.dq[2]
        tq[3] = -a * q.dq[3]

        return DualQuaternion_gtc(tq)

    @staticmethod
    def pow(dq, e):
        normA, l, m, ang = DualQuaternion_gtc.getScrewPara(dq)
        #pure translation
        if (normA < 1e-15):
            q = DualQuaternion_gtc(dq)
            q.dq[5] = q.dq[5] * e
            q.dq[6] = q.dq[6] * e
            q.dq[7] = q.dq[7] * e
            return DualQuaternion_gtc.normalise(q)

        theta = ang[0] * e
        alpha = ang[1] * e

        d = DualQuaternion_gtc.setScrewPara(l, m, theta, alpha)
        return DualQuaternion_gtc.normalise(d)

    @staticmethod
    def log(q):
        normA, l, m, ang = DualQuaternion_gtc.getScrewPara(q)
        r = DualQuaternion_gtc()
        r.dq[0] = 0
        r.dq[4] = 0
        r.dq[1] = ang[0] / 2 * l[0]
        r.dq[2] = ang[0] / 2 * l[1]
        r.dq[3] = ang[0] / 2 * l[2]
        r.dq[1] = ang[1] / 2 * l[0] + ang[0] / 2 * m[0]
        r.dq[2] = ang[1] / 2 * l[1] + ang[0] / 2 * m[1]
        r.dq[3] = ang[1] / 2 * l[2] + ang[0] / 2 * m[2]
        return r

    @staticmethod
    def exp(q):
        theta = np.sqrt(q.dq[1] ** 2 + q.dq[2] ** 2 + q.dq[3] ** 2)
        if(theta < 1e-15):
            return DualQuaternion_gtc()
        AB = q.dq[1] * q.dq[5] + q.dq[2] * q.dq[6] + q.dq[3] * q.dq[7]

        alpha = AB / 2 / theta

        AB = -AB / 2 / (theta**3)
        l = np.asarray([q.dq[1] / theta, q.dq[2] / theta, q.dq[3] / theta])
        m = np.asarray([q.dq[5] / theta + AB * q.dq[1], q.dq[6] / theta + AB * q.dq[2], q.dq[7] / theta + AB * q.dq[3]])
        return DualQuaternion_gtc.setScrewPara(l,m,theta,alpha)

    @staticmethod
    def screwLinearInterpolate(q1, q2, t):
        tq = DualQuaternion_gtc.conjugate(q1)
        tq1 = DualQuaternion_gtc.multiply(tq, q2)
        tq2 = DualQuaternion_gtc.pow(tq1, t)
        tq3 = DualQuaternion_gtc.multiply(q1, tq2)
        return DualQuaternion_gtc.normalise(tq3)

    @staticmethod
    def linearBlending(ws, dqs):
        s = len(ws)
        if s == 0:
            return DualQuaternion_gtc()
        dq = ws[0] * dqs[0].dq
        for i in range(1, s):
            dq += ws[i] * dqs[i].dq

        q = DualQuaternion_gtc(dq)

        return DualQuaternion_gtc.normalise(q)

    @staticmethod
    def IterativeBlending(ws, dqs):
        b = DualQuaternion_gtc.linearBlending(ws, dqs)
        err = 10
        loop = 0
        s = len(ws)
        while err > 1e-8 and loop < 30:
            loop = loop + 1
            #method 1:
            # bTmp = b
            #method 2:
            bTmp = DualQuaternion_gtc.conjugate(b)
            rtmp = np.zeros(3)
            dtmp = np.zeros(3)
            x = DualQuaternion_gtc()
            x.dq[0] = 0
            for i in range(0, s):
                bqTmp = DualQuaternion_gtc.multiply(bTmp, dqs[i])
                # method 1:
                # x.dq += self.log(bqTmp).dq
                # method 2:
                normA, l, m, ang = DualQuaternion_gtc.getScrewPara(bqTmp)
                rtmp += ws[i] * ang[0] / 2 * l
                dtmp += ws[i] * ang[1] / 2 * l + ws[i] * ang[0] / 2 * m
            # method 1:
            # bqTmp = self.exp(x)
            # method 2:
            bqTmp = DualQuaternion_gtc.exp(DualQuaternion_gtc([0, rtmp[0], rtmp[1], rtmp[2], 0, dtmp[0], dtmp[1], dtmp[2]]))
            b = DualQuaternion_gtc.multiply(b, bqTmp)
            err = np.sqrt(rtmp[0]**2 + rtmp[1]**2 + rtmp[2]**2) + np.sqrt(dtmp[0]**2 + dtmp[1]**2 + dtmp[2]**2)
        # print("err:", err, " loop:", loop)

        return DualQuaternion_gtc.normalise(b)
