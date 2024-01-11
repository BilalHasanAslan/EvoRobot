import numpy as np
import random
import os
import statistics
from mp.map import map_elites
from mp.robot import robot
from mp.irregularPoly import irregularPoly
import utils.sim_art as simart
import utils.vox_mesh as vm
import pickle
import math
import multiprocessing
import shutil


def obj_func(s):
    if s.evo_type == "component":
        # determine leg structure based on shape grammar
        s.decode_grammar()
        s.assemble()
        s.update_offset()
        # check if resulting leg is valid i.e. no overlaps
        if not s.is_valid():
            s.evaluated = False
            s.set_fitness(-1)
        else:
            # complexity computation
            if len(s.x) > 2:
                angle = []
                for i in range(0, len(s.x) - 2):
                    ctr = []
                    for j in range(0, 3):
                        ctr.append(list(s.x[i + j].centroid.coords[0]))
                    res = vm.component_conn_angle(ctr[0], ctr[2], ctr[1])
                    if res < 0:
                        res = res + 360
                    angle.append(res)
                complexity = np.median(angle)
            else:
                complexity = 180

            length = 0.5
            if len(s.x) > 1:
                for i in range(0, len(s.x)-1):
                    x_dist = s.x[i+1].centroid.x - s.x[i].centroid.x
                    y_dist = s.x[i+1].centroid.y - s.x[i].centroid.y
                    length += math.sqrt(pow(x_dist, 2) + pow(y_dist, 2))

            # covert 2D leg structure into 3D meshes and compute objective value
            s.generate_mesh()
            total = 0
            for i in range(0, len(s.mesh)):
                total = total + (s.mesh[i].volume / s.mesh[i].area)
            fitness = total / len(s.mesh)
            s.fitness = fitness
            # features: friction, restitution, length, complexity
            s.desc = [(statistics.mean(s.friction) - 0.25) / (1 - 0.25),
                      (statistics.mean(s.restitution) - 0.25) / (1 - 0.25), (length-0.5)/(8.5-0.5),
                      (np.clip(complexity, 45, 320)-45) / (320-45)]
            s.evaluated = True
    else:
        ctrl, fitness, pos = simart.simulate(s, 1)
        s.controller = ctrl
        if fitness == 0:
            s.evaluated = False
            s.set_fitness(0)
        else:
            leg_feat_sum = [0, 0, 0]
            for i in range(0, len(s.parts)):
                leg_feat_sum[0] = leg_feat_sum[0] + s.parts[i].desc[0]
                leg_feat_sum[1] = leg_feat_sum[1] + s.parts[i].desc[1]
                leg_feat_sum[2] = leg_feat_sum[2] + s.parts[i].desc[2]
            feature = []
            for i in range(0, 3):
                feature.append(leg_feat_sum[i] / len(s.parts))
            s.set_fitness(fitness)
            s.desc = np.array(feature)
            s.evaluated = True
    return s


def evaluate_env(input_set):
    s = input_set[0]
    s.ctrl_env_friction = input_set[1]
    s.ctrl_env_restitution = input_set[2]
    fitness, pos = simart.simulate(s, 2)
    return [fitness, pos]


def save_archive(arch, path):
    arch_list = list(arch.values())
    for i in range(0, len(arch_list)):
        with open(path + "//" + arch_list[i].id + "_fit" + str(arch_list[i].fitness), 'wb') as pickle_file:
            pickle.dump(arch_list[i], pickle_file)
    with open(path + "//archive", 'wb') as pickle_file:
        pickle.dump(arch, pickle_file)


def re_evaluate(archive, env_friction, env_restitution, run_folder):
    num_cores = multiprocessing.cpu_count()
    os.system('taskset -cp 0-%d %s' % (num_cores, os.getpid()))
    pool = multiprocessing.Pool(num_cores)
    sols = list(archive.values())
    eval_list = [sols, [env_friction]*len(sols), [env_restitution]*len(sols) ]
    eval_list = list(zip(*eval_list))
    out_list = pool.map(evaluate_env, eval_list)



    
    with open(run_folder+ "//robo_altenv.dat", 'w') as f:
        for idx, k in enumerate(sols):
            f.write(str(k.id) + ' ')
            f.write(str(out_list[idx][0]) + ' ')
            for i in k.desc:
                f.write(str(i) + ' ')
            f.write("\n")


def run_experiment(default_params, irrPolyInd, robotInd, friction_values, restitution_values, gen, niches,
                   alternate_env, run_id):

    materials = [[i, j] for i in friction_values
                 for j in restitution_values]

    folder_name = default_params['experiment_name'] + "_gen_"+ str(gen[0]) + "_" + str(gen[1])+"_niche_" + str(niches)
    root = "data//"+folder_name

    np.random.seed(run_id)
    random.seed(run_id)
    default_params['experiment_runid'] = run_id
    run_folder = root + "_run" + str(run_id)
    default_params['run_folder'] = run_folder
    os.mkdir(run_folder)
    os.mkdir(run_folder+"//temp_solutions")
    temp_solution_path = run_folder+"//temp_solutions"
    os.mkdir(run_folder+"//run_data")
    robotInd['file_path'] = temp_solution_path+"//temp"

    with open(run_folder + "//params", 'wb') as pickle_file:
        pickle.dump({'comp_params': irrPolyInd, 'robo_params': robotInd, 'overall_params': default_params,
                     'materials': materials, 'generations': gen, 'niches': niches, 'alt_env': alternate_env,
                     'run_id': run_id}, pickle_file)

    layer_info = {"layer": 'layer2', "lower_layer": materials, "ind_params": irrPolyInd}
    mp = map_elites(4, niches, gen[0], default_params, layer_info)
    archive = mp.compute(obj_func, irregularPoly,0)
    layer_data_path = run_folder + "//run_data//layer2"
    os.mkdir(layer_data_path)
    save_archive(archive, layer_data_path)
    
    pickle_out = open(run_folder+"/toContinueLowerLayer.pickle","wb")
    pickle.dump(archive, pickle_out)
    pickle_out.close()

    layer_info = {"layer": 'layer3', "lower_layer": archive, "ind_params": robotInd}
    mp = map_elites(3, niches, gen[1], default_params, layer_info)
    archive = mp.compute(obj_func, robot,0)
    layer_data_path = run_folder + "//run_data//layer3"
    os.mkdir(layer_data_path)
    save_archive(archive, layer_data_path)

    re_evaluate(archive, alternate_env[0], alternate_env[1], run_folder)


def continue_experiment(default_params, irrPolyInd, robotInd, friction_values, restitution_values, gen, niches,
                   alternate_env, run_id,newgen):
    materials = [[i, j] for i in friction_values
                 for j in restitution_values]

    folder_name = default_params['experiment_name'] + "_gen_"+ str(gen[0]) + "_" + str(gen[1])+"_niche_" + str(niches)
    root = "data//"+folder_name

    np.random.seed(run_id)
    random.seed(run_id)
    default_params['experiment_runid'] = run_id
    run_folder = root + "_run" + str(run_id)
    default_params['run_folder'] = run_folder
    temp_solution_path = run_folder+"//temp_solutions"
    robotInd['file_path'] = temp_solution_path+"//temp"
    try:
        shutil.rmtree(temp_solution_path)
    except:
        pass
    os.makedirs(temp_solution_path)
    print(run_folder)

    pickle_in = open(run_folder+"/toContinueLowerLayer.pickle","rb")
    archive = pickle.load(pickle_in)
    
    file2 = open(run_folder+"/LeftOffGeneration.txt","r")
    g = int(file2.readline())+1
    file2.close()

    layer_info = {"layer": 'layer3', "lower_layer": archive, "ind_params": robotInd}
    mp = map_elites(3, niches, newgen, default_params, layer_info)
    
    pickle_in = open(run_folder+"/toContinueUpperLayer.pickle","rb")
    archiveupper = pickle.load(pickle_in)
    mp.add_archive(archiveupper)
    archive = mp.compute(obj_func, robot,g)
    layer_data_path = run_folder + "//run_data//layer3"

    
    save_archive(archive, layer_data_path)

    re_evaluate(archive, alternate_env[0], alternate_env[1], run_folder)
