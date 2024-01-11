#backup
import pybullet as p
import numpy as np
import math
import time
import pybullet_data
import os
import random
import shutil
import copy


def joint_position(amplitude, frequency, phase, offset, t):
    out = (amplitude * math.sin((frequency * t) + phase)) + offset
    return out


def eval_singlesol(genome, s, **kwargs):
    arr = [0]
    Allfitness = []
    Allpos = []
    for i in arr:
        gui = kwargs.get('gui', False)
        rec_vid = kwargs.get('record_vid', False)

        if gui:
            client = p.connect(p.GUI)
            if rec_vid:
                file_path = kwargs.get('rec_file_path', "data//videos//best.mp4")
                logId = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, file_path)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        else:
            client = p.connect(p.DIRECT)

        sim_time = s.sim_time
        file_path = s.file_path + s.id
        min_x = s.ctrl_env_xlim[0]
        min_y = s.ctrl_env_ylim[0]
        max_y = s.ctrl_env_ylim[1]
        dim_factor = 20
        # setup environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, 0)

        individual_starts = []
        if s.num_components == 1:
            length = 1
            individual_starts.append(0.0)
        else:
            for k in range(0, s.num_components):
                if k == 0:
                    individual_starts.append(0)
                else:
                    individual_starts.append(individual_starts[k-1]+8)

            length = max(individual_starts)

        centre = p.createCollisionShape(p.GEOM_BOX, halfExtents=[(length/2)/dim_factor, (0.5/dim_factor), (0.5/dim_factor)])
        color = {"0.25": [0, 255, 0, 1],  # lime (green)
                "0.5": [255, 0, 0, 1],  # red
                "0.75": [0, 0, 255, 1],  # blue
                "1.0": [255, 255, 0, 1]}  # yellow
        axis = []
        leg = []
        jointTypes = []
        link_Masses = []
        linkInertialFramePositions = []
        linkOrientations = []
        linkInertialFrameOrientations = []
        indices = []
        linkPositions = []
        link_friction = []
        link_restitution = []

        mass = 1
        basePosition = [0, i, 0.0]
        baseOrientation = [0, 0, 0, 1]
        start = 0-((length/2)/dim_factor)
        comp_start = 0
        all_joints = []

        for i in range(0, s.num_components):
            for j in range(0, len(s.parts[i].x)):
                temp_offset = list(np.array(s.parts[i].offset[j]) / dim_factor)
                leg.append(p.createCollisionShape(p.GEOM_MESH, fileName=file_path+"//leg"+str(i+1)+"//leg"+str(i)+"_part" +
                                                                        str(j) + ".obj", meshScale=[1/dim_factor, 1/dim_factor, 1/dim_factor]))
                leg.append(p.createCollisionShape(p.GEOM_MESH, fileName=file_path + "//leg" + str(i + 1) + "//leg" + str(
                    i) + "_part" + str(j) + "_sym.obj", meshScale=[1/dim_factor, 1/dim_factor, 1/dim_factor]))
                all_joints.append(s.joints[i][j])
                for k in range(0,2):
                    link_Masses.append(0.05)
                    link_friction.append(s.parts[i].friction[j])
                    link_restitution.append(s.parts[i].restitution[j])
                    linkOrientations.append([0, 0, 0, 1])
                    linkInertialFramePositions.append([0, 0, 0])
                    linkInertialFrameOrientations.append([0, 0, 0, 1])
                    if s.joints[i][j] == 1:
                        axis.append([0, 0, 1])
                        jointTypes.append(p.JOINT_REVOLUTE)
                    elif s.joints[i][j] == 0:
                        axis.append([1, 0, 0])
                        jointTypes.append(p.JOINT_REVOLUTE)
                    else:
                        axis.append([1, 0, 0])
                        jointTypes.append(p.JOINT_FIXED)
                if j == 0:
                    if i == 0:
                        x_coord = start + (individual_starts[i]/dim_factor)
                    else:
                        x_coord = start + ((individual_starts[i]-1)/dim_factor)
                    linkPositions.append([x_coord, 0.6/dim_factor, -0.5/dim_factor])
                    linkPositions.append([x_coord, -1.6/dim_factor, -0.5/dim_factor])
                    indices.append(0)
                    indices.append(0)
                else:
                    offset = temp_offset
                    linkPositions.append([offset[0], offset[1], 0])
                    linkPositions.append([offset[0], -1 * offset[1], 0])
                    indices.append(comp_start+(j*2)-1)
                    indices.append(comp_start+(j * 2))
            comp_start = comp_start + (len(s.parts[i].x) * 2)


        roboId = p.createMultiBody(mass,
                                centre,
                                centre,
                                basePosition,
                                baseOrientation,
                                linkMasses=link_Masses,
                                linkCollisionShapeIndices=leg,
                                linkVisualShapeIndices=leg,
                                linkPositions=linkPositions,
                                linkOrientations=linkOrientations,
                                linkInertialFramePositions=linkInertialFramePositions,
                                linkInertialFrameOrientations=linkInertialFrameOrientations,
                                linkParentIndices=indices,
                                linkJointTypes=jointTypes,
                                linkJointAxis=axis,
                                #flags=p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
                                )

        p.setGravity(0, 0, -10)

        # identify initial position
        o_pos, o_orn = p.getBasePositionAndOrientation(roboId)
        prev_pos = np.array(o_pos)

        if length == 1:
            weight_factor = 2
        else:
            weight_factor = 2.5

        # get joint information
        _link_name_to_index = {p.getBodyInfo(roboId)[0].decode('UTF-8'): -1, }
        for _id in range(p.getNumJoints(roboId)):
            _name = p.getJointInfo(roboId, _id)[12].decode('UTF-8')
            _link_name_to_index[_name] = _id
            p.changeDynamics(roboId, _id, jointLowerLimit=s.joint_llim, jointUpperLimit=s.joint_ulim,
                            restitution=link_restitution[int(_name.split('link')[1])-1],
                            lateralFriction=link_friction[int(_name.split('link')[1])-1])
            p.changeVisualShape(roboId, _id, rgbaColor=color[str(link_friction[int(_name.split('link')[1])-1])])
        p.changeDynamics(0, -1, restitution=s.ctrl_env_restitution, lateralFriction=s.ctrl_env_friction)
        p.changeDynamics(roboId, -1, restitution=s.cross_section[0], lateralFriction=s.cross_section[1], mass=s.num_components*weight_factor)

        step = 0
        violation = False
        while step <= sim_time:
            phase = genome[0]
            amplitude = genome[1]
            frequency = genome[2]
            jid = 0
            for i in range(0, int(p.getNumJoints(roboId) / 2)):
                if all_joints[i] != -1:
                    # scale output into appropriate range and update joint position

                    idx1 = 'link' + str((i * 2) + 1)
                    idx2 = 'link' + str((i * 2) + 2)
                    if step % 240 > 20:
                        val = np.clip(joint_position(amplitude, frequency, phase, genome[jid+3], (step % 240)/4), -2, 2)

                        val = (val--2)/4

                        val = val * (s.ctrl_delta*2)
                        val = val + (-1 * s.ctrl_delta)
                        pos = p.getJointState(roboId, _link_name_to_index[idx1])[0]
                        new_pos = np.clip(val + pos, s.joint_llim, s.joint_ulim)
                    else:
                        new_pos = 0

                    p.setJointMotorControl2(roboId, _link_name_to_index[idx1], p.POSITION_CONTROL, targetPosition=new_pos)
                    p.setJointMotorControl2(roboId, _link_name_to_index[idx2], p.POSITION_CONTROL, targetPosition=-1*new_pos)
                    jid = jid + 1


            # advance simulation
            p.stepSimulation()

            if gui:
                time.sleep(1 / 240)
                pos, orn = p.getBasePositionAndOrientation(roboId)
                if s.num_components == 1:
                    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=-90, cameraPitch=-30,
                                                cameraTargetPosition=pos, physicsClientId=client)
                elif s.num_components == 2:
                    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=-90, cameraPitch=-30,
                                                cameraTargetPosition=pos, physicsClientId=client)
                else:
                    p.resetDebugVisualizerCamera(cameraDistance=12, cameraYaw=-90, cameraPitch=-30,
                                                cameraTargetPosition=pos, physicsClientId=client)

            nxt_pos, n_orn = p.getBasePositionAndOrientation(roboId)

            if (step > 100) and (nxt_pos[1] > max_y or nxt_pos[1] < min_y or nxt_pos[0] < min_x or
                                n_orn[2] < -0.2 or n_orn[2] > 0.2):

                violation = True
                break

            step = step + 1

        if gui and rec_vid:
            p.stopStateLogging(logId)

        # identify final position and disconnect from simulator
        n_pos, n_orn = p.getBasePositionAndOrientation(roboId)
        p.disconnect()

        # compute distance covered
        n_pos = np.array(n_pos)
        if violation:
            fitness = 0
        else:
            x_dist = n_pos[0] - prev_pos[0]
            fitness = x_dist
        Allfitness.append(fitness)
        Allpos.append(n_pos)
    
    fitness = 0
    for i in Allfitness:
        fitness+=i
    fitness = fitness/len(arr)  
    
    n_pos = [0,0]
    for i in Allpos:
        n_pos[0]+=i[0]
        n_pos[1]+=i[1]
    
    n_pos[0] = n_pos[0]/len(arr)
    n_pos[1] = n_pos[1]/len(arr)
    return fitness, n_pos


def optimize(s):
    sol = []
    for i in range(0, int(s.num_joints/2)+3):
        sol.append(random.uniform(-1, 1))

    f, pos = eval_singlesol(sol, s)
    for gen in range(0, s.ctrl_trials_gen):
        new_sol = copy.deepcopy(sol)
        for i in range(0, int(s.num_joints/2)+3):
            if random.random() < 0.3:
                new_sol[i] = random.uniform(-1, 1)
        n_f, n_pos = eval_singlesol(new_sol, s)
        if n_f >= f:
            sol = new_sol
            f = n_f
            pos = n_pos

    return [sol, f, pos]


def simulate(s, mode):

    os.mkdir(s.file_path + s.id)
    num_joints = 0
    for i in range(0, s.num_components):
        leg_dir = s.file_path + s.id + "//leg"+str(i+1)
        os.mkdir(leg_dir)
        for j in range(0, len(s.parts[i].x)):
            s.parts[i].mesh[j].export(leg_dir+"//leg"+str(i)+"_part"+str(j)+".obj")
            s.parts[i].mesh_sym[j].export(leg_dir+"//leg" + str(i) + "_part" + str(j) + "_sym" + ".obj")
        num_joints += sum(x >= 0 for x in s.joints[i])

    s.num_joints = num_joints*2
    if mode == 1:
        # search for optimal controller
        out = optimize(s)
        shutil.rmtree(s.file_path + s.id)
        return out[0], out[1], out[2]
    else:
        fitness, pos = eval_singlesol(s.controller, s)
        shutil.rmtree(s.file_path + s.id)
        return fitness, pos
