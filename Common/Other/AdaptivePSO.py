import numpy as np
from scipy.spatial.transform import *
from dual_quaternions import DualQuaternion
from oapackage import *

class AdaptivePSO():
    def __init__(self,
        n_particles,
        n_dimensions,
        options,
        bounds,
        velocity_clamp):
        self.particles = n_particles
        self.dimensions = n_dimensions
        self.options = options
        self.bounds = bounds
        self.velocity_clamp = velocity_clamp

    def optimize(
        self, objective_func, iters, **kwargs):
        self.swarm = self.createSwarm()
        self.velocity = np.zeros((self.particles, self.dimensions))
        self.pbest_cost = np.repeat(np.inf, self.particles, axis = 0)
        self.pbest_pos = np.zeros((self.particles, self.dimensions))
        self.gbest_pos = np.zeros((1, self.dimensions))
        self.gbest_cost = np.inf
        self.gbest_idx = 0
        self.pre_stage = -1
        self.cur_stage = 0
        for i in range(iters):
            self.current_cost = self.compute_objective_function(self.swarm, objective_func, **kwargs)
            self.compute_pbest(self.swarm, self.current_cost)
            best_cost_yet_found = self.gbest_cost
            self.compute_gbest(self.swarm)
            self.distance = self.averageDistance()
            dg = self.distance[self.gbest_idx]
            f = self.evolFactor(dg)
            self.options["w"] = self.momentWeight(f)
            self.cur_stage = self.evolStage(f)
            self.cvalueAdust()
            if(self.cur_stage == 2):
                temp_best = self.elsStageUpdate(i,iters)
                temp_cost = self.compute_objective_function(np.array(temp_best).reshape(1,self.dimensions), objective_func, **kwargs)
                if(temp_cost[0] < self.gbest_cost):
                    self.gbest_pos = temp_best
                    self.gbest_cost = temp_cost[0]
                else:
                    worst_index = np.argmax(self.current_cost)
                    self.swarm[worst_index] = temp_best
                    self.current_cost[worst_index] = temp_cost[0]
                    self.compute_pbest(self.swarm, self.current_cost)
            self.compute_velocity()
            self.compute_position()
            # self.swarm[self.gbest_idx,:] = self.addOneSwarm()
        return (self.gbest_cost, self.gbest_pos)

    def compute_gbest(self, swarm):
        if np.min(self.pbest_cost) < self.gbest_cost:
            self.gbest_idx = np.argmin(self.pbest_cost)
            best_pos = self.pbest_pos[self.gbest_idx]
            best_cost = np.min(self.pbest_cost)
            self.gbest_pos = best_pos
            self.gbest_cost = best_cost

    def pos2Dq(self, pos):
        dqs=[]
        for p in pos:
            r = Rotation.from_euler('xyz', p[0:3], degrees=True)
            rotm = r.as_matrix()
            m = np.zeros((4, 4))
            for i in range(3):
                tl = list(rotm[i])
                tl.append(p[i+3])
                m[i] = np.array(tl)
            m[3][3] = 1
            dq = DualQuaternion.from_homogeneous_matrix(m)
            dqs.append(dq)
        return np.array(dqs)

    def dq2Pos(self, dqs):
        poss=[]
        for dq in dqs:
            r = dq.homogeneous_matrix()
            r2 = r[0:3, 0:3]
            r3 = Rotation.from_matrix(r2)
            r4 = r3.as_euler('xyz', degrees=True)
            pos=[r4[0],r4[1],r4[2],r[0][3],r[1][3],r[2][3]]
            poss.append(pos)
        return poss

    def compute_velocity(self):
        c1 = self.options["c1"]
        c2 = self.options["c2"]
        w = self.options["w"]
        cognitive = (
                c1
                * np.random.uniform(0, 1, self.dimensions)
                * (self.pbest_pos - self.swarm)
        )
        social = (
                c2
                * np.random.uniform(0, 1, self.dimensions)
                * (self.gbest_pos - self.swarm)
        )
        # Compute temp velocity (subject to clamping if possible)
        temp_velocity = (w * self.velocity) + cognitive + social
        # temp_velocity = np.floor(temp_velocity)
        for i in range(self.particles):
            for j in range(self.dimensions):
                if np.abs(temp_velocity[i][j]) < self.velocity_clamp[0][j]:
                    temp_velocity[i][j] = np.sign(temp_velocity[i][j]) * self.velocity_clamp[0][j]
                if np.abs(temp_velocity[i][j]) > self.velocity_clamp[1][j]:
                    temp_velocity[i][j] = np.sign(temp_velocity[i][j]) * self.velocity_clamp[1][j]

        left = np.round(10.0 * temp_velocity[:, 0:3]) / 10.0
        right = np.round(10.0 * temp_velocity[:, 3:6]) / 10.0
        for i in range(self.particles):
            for j in range(self.dimensions):
                if j<3:
                    temp_velocity[i][j] = left[i][j]
                else:
                    temp_velocity[i][j] = right[i][j-3]
        self.velocity = temp_velocity

    def compute_position(self):
        temp_position = self.swarm.copy()
        temp_position += self.velocity
        for i in range(self.particles):
            for j in range(self.dimensions):
                if temp_position[i][j] < self.bounds[0][j]:
                    temp_position[i][j] = self.bounds[0][j]
                if temp_position[i][j] > self.bounds[1][j]:
                    temp_position[i][j] = self.bounds[1][j]
        self.swarm = temp_position

    def createSwarm(self):
        # run_size = self.particles
        # strength = self.dimensions
        # number_of_factors = self.dimensions
        # factor_levels = [6, 6, 6, 6, 11, 6]
        # arrayclass = oapackage.arraydata_t(factor_levels, run_size, strength, number_of_factors)
        # al = arrayclass.create_root()
        # X = np.array(al)
        pos = np.zeros((self.particles, self.dimensions))
        # for i in range(self.particles):
        #     for j in range(self.dimensions):
        #         if j==4:
        #             pos[i, j] = X[i, j] * 2 + self.bounds[0][j]
        #         else:
        #             pos[i, j] = X[i, j] * 4 + self.bounds[0][j]
        for i in range(self.dimensions):
            pos[:,i] = np.random.random_integers(low = self.bounds[0][i], high = self.bounds[1][i], size=(1, self.particles))
        return pos

    def averageDistance(self):
        dis =[]
        for i in range(self.particles):
            dis_j = 0
            for j in range(self.particles):
                dis_k = 0
                for k in range(self.dimensions):
                    dis_k += (self.swarm[i][k] - self.swarm[j][k])**2
                dis_k = np.sqrt(dis_k)
                dis_j += dis_k
            dis.append(dis_j/(self.particles - 1))
        return dis

    def evolFactor(self, dg):
        d_max = np.array(self.distance).max()
        d_min = np.array(self.distance).min()
        return (dg - d_min) / (d_max - d_min)

    def explorationValue(self, f):
        if f<=0.4:
            return 0
        elif f<=0.6:
            return 5*f-2
        elif f<=0.7:
            return 1
        elif f<=0.8:
            return 8 - (10*f)
        else:
            return 0

    def exploitationValue(self,f):
        if f<=0.2:
            return 0
        elif f<=0.3:
            return 10*f-2
        elif f<=0.4:
            return 1
        elif f<=0.6:
            return 3-5*f
        else:
            return 0

    def convergenceValue(self, f):
        if f<=0.1:
            return 1
        elif f<=0.3:
            return 1.5 - 5*f
        else:
            return 0

    def jumpoutValue(self, f):
        if f<=0.7:
            return 0
        elif f<=0.9:
            return -3.5 - 5*f
        else:
            return 1

    def evolStage(self, f):
        er = self.explorationValue(f)
        ei = self.exploitationValue(f)
        cr = self.convergenceValue(f)
        jo = self.jumpoutValue(f)
        data = np.array([er, ei, cr, jo])
        index = np.argsort(data)
        if(self.pre_stage == -1):
            return index[3]
        else:
            if(self.pre_stage - index[3] == -1):
                return index[3]
            elif(self.pre_stage == 3 and index[3] == 0):
                return index[3]
            else:
                return self.pre_stage

    def momentWeight(self, f):
        return 1.0 / (1.0 + 1.5 * np.exp(-2.6 * f))

    def clampC1AndC2(self, C1, C2):
        if C1 < 1.5:
            C1 = 1.5
        if C1 > 2.5:
            C1 = 2.5
        if C2 < 1.5:
            C2 = 1.5
        if C2 > 2.5:
            C2 = 2.5
        if C1 + C2 < 3.0:
            C1 = C1 * 3.0 / (C1 + C2)
            C2 = C2 * 3.0 / (C1 + C2)
        if C1 + C2 > 4.0:
            C1 = C1 * 4.0 / (C1 + C2)
            C2 = C2 * 4.0 / (C1 + C2)
        return (C1, C2)

    def cvalueAdust(self):
        if self.cur_stage == 0:
            delta = np.random.uniform(0.05, 0.1, 1)
            C1 = self.options["c1"]
            C1 = C1 + delta
            delta = np.random.uniform(0.05, 0.1, 1)
            C2 = self.options["c2"]
            C2 = C2 - delta
        elif self.cur_stage == 1:
            delta = np.random.uniform(0.05, 0.1, 1)
            C1 = self.options["c1"]
            C1 = C1 + 0.5 * delta
            delta = np.random.uniform(0.05, 0.1, 1)
            C2 = self.options["c2"]
            C2 = C2 - 0.5 * delta
        elif self.cur_stage == 2:
            delta = np.random.uniform(0.05, 0.1, 1)
            C1 = self.options["c1"]
            C1 = C1 + 0.5 * delta
            delta = np.random.uniform(0.05, 0.1, 1)
            C2 = self.options["c2"]
            C2 = C2 + 0.5 * delta
        else:
            delta = np.random.uniform(0.05, 0.1, 1)
            C1 = self.options["c1"]
            C1 = C1 - delta
            delta = np.random.uniform(0.05, 0.1, 1)
            C2 = self.options["c2"]
            C2 = C2 + delta

        C1, C2 = self.clampC1AndC2(C1, C2)
        self.options["c1"] = C1
        self.options["c2"] = C2

    def elsStageUpdate(self, g, G):
        index = np.random.randint(0, self.dimensions, 1)
        #delta max: 1.0
        #delta min: 0.1
        delta = 1.0 - (1.0 - 0.1) * g / G
        rate = np.random.normal(0, delta, 1)
        cur_best = self.gbest_pos.copy()
        cur_best[index] += (self.bounds[1][index] - self.bounds[0][index]) * rate
        if(cur_best[index] < self.bounds[0][index]):
            cur_best[index] = self.bounds[0][index]
        if(cur_best[index] > self.bounds[1][index]):
            cur_best[index] = self.bounds[1][index]
        return cur_best

    def compute_objective_function(self, swarm, objective_func, **kwargs):
        return objective_func(swarm, **kwargs)

    def compute_pbest(self, swarm, cost):
        for i in range(self.particles):
            if cost[i] < self.pbest_cost[i]:
                self.pbest_pos[i] = swarm[i]
                self.pbest_cost[i] = cost[i]

    def addOneSwarm(self):
        pos = np.zeros((1, self.dimensions))
        threshold = 5.0
        loop = 0
        while(1):
            for i in range(self.dimensions):
                pos[0, i] = np.random.random_integers(low=self.bounds[0][i], high=self.bounds[1][i], size=1)
            valid = 1
            for i in range(self.particles):
                dis = 0
                for j in range(self.dimensions):
                    dis += (pos[0, j]-self.swarm[i][j])**2
                dis=np.sqrt(dis / self.dimensions)
                if(dis < threshold):
                    valid = 0
                    break
            if valid == 1:
                break;
            else:
                loop += 1
                if loop > 5:
                    threshold -= 1.0
        return pos