import run
import sys
import time
import random
from random import randrange

default_params = \
    {
        # more of this -> higher-quality CVT
        "cvt_samples": 25000,
        # we evaluate in batches to parallelize
        "batch_size": 100,
        # proportion of niches to be filled before starting
        "random_init": 0.1,
        # batch for random initialization
        "random_init_batch": 1000,
        # when to write results (one generation = one batch)
        "dump_period": 10,
        # do we use several cores? ---we set this individually for different layers later in the code
        "parallel": True,
        # do we cache the result of CVT and reuse?
        "cvt_use_cache": True,
        # use xover and mutation or just mutation
        "mutation_only": True,
        # experiment folder
        "experiment_name": sys.argv[1]
    }

# components layer paramters
irrPolyInd = {
    "num_components": 3,
    "evo_type": "component",
    "grammar_iterations": 3,
    "num_points": 10
}

# robot layer parameters
robotInd = {
    "evo_type": "robot",
    "simulation_time": 100000,
    "leg_lim": [1,3],
    "contrl_trials": 1,
    "contrl_trial_gen": 20,
    "contrl_env_friction": float(sys.argv[2]),
    "contrl_env_restitution": float(sys.argv[3]),
    "contrl_env_xlim": [0, 10],
    "contrl_env_ylim": [-5, 5],
    "joint_ulim": 0.2,
    "joint_llim": -0.2,
    "contrl_delta_angle": 0.05,
    "file_path": "",
}

# materials layer: created using combinations of different values for friction and restitution


friction_values = [0.25, 0.50, 0.75, 1.0]
restitution_values = [0.25, 0.50, 0.75, 1.0]

generations = [int(sys.argv[6]), int(sys.argv[7])]
niches = 1000
alternate_env = [float(sys.argv[4]), float(sys.argv[5])]
start = time.time()

if int(sys.argv[9])==1:
    irand = randrange(0, 120)
    time.sleep(irand)
    start = time.time()
    SEED = random.SystemRandom().randint(1,99999)
    file1 = open("UsedSeeds.txt","a+")
    Lines = file1.readlines()
    flag = True
    while flag:
        for line in Lines:
            if int(line) == SEED:
                SEED = random.SystemRandom().randint(1,99999)
                break
        flag = False 
        file1.write(str(SEED)+"\n")  
    file1.close()
    run.run_experiment(default_params, irrPolyInd, robotInd, friction_values, restitution_values, generations,
                niches, alternate_env, SEED)
if int(sys.argv[9])==2:
    run.continue_experiment(default_params, irrPolyInd, robotInd, friction_values, restitution_values, generations,
                niches, alternate_env, int(sys.argv[8]),int(sys.argv[10]))
    
end = time.time()
print(end - start)
