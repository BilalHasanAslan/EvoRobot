import random
import copy
import utils.vox_mesh as vm


def generate_joints_fixed(num_joints):
    joints = []
    for i in range(0, num_joints):
        joints.append(random.randint(-1,1))
    return joints


class robot(object):

    def __init__(self, params):
        self.desc = []
        self.fitness = 0
        self.num_components = 0
        self.evo_type = params["evo_type"]
        self.file_path = params["file_path"]
        self.joint_ulim = params["joint_ulim"]
        self.joint_llim = params["joint_llim"]
        self.ctrl_delta = params["contrl_delta_angle"]
        self.sim_time = params["simulation_time"]
        self.ctrl_trials = params["contrl_trials"]
        self.ctrl_trials_gen = params["contrl_trial_gen"]
        self.ctrl_env_friction = params["contrl_env_friction"]
        self.ctrl_env_restitution = params["contrl_env_restitution"]
        self.ctrl_env_xlim = params["contrl_env_xlim"]
        self.ctrl_env_ylim = params["contrl_env_ylim"]
        self.parts = []
        self.evaluated = False
        self.mesh = None
        self.joints = []
        self.leg_lim = params["leg_lim"]
        self.controller = None
        self.controller_config = None
        self.cross_section = [0.5, 0.5, 1]
        self.id = None

    def generate_random(self, archive):
        self.num_components = random.randint(self.leg_lim[0], self.leg_lim[1])
        # pick components from archive
        for i in range(0, self.num_components):
            idx = random.randint(0, len(archive) - 1)
            self.parts.append(copy.deepcopy(list(archive.values())[idx]))
            self.joints.append(generate_joints_fixed(len(self.parts[i].x)))
        self.id = vm.id_generator()

    def get_fitness(self):
        return self.fitness

    def set_fitness(self, fitness):
        self.fitness = fitness

    def mutate(self, archive):
        self.id = vm.id_generator()
        option = random.randint(0, 3)
        option = 5
        if option == 0:
            comp_id = random.randint(0, self.num_components - 1)
            idx = random.randint(0, len(archive) - 1)
            self.parts[comp_id] = copy.deepcopy(list(archive.values())[idx])
            self.joints[comp_id] = generate_joints_fixed(len(self.parts[comp_id].x))
        elif option == 1:
            joint_id = random.randint(0, len(self.joints)-1)
            for i in range(0, len(self.joints[joint_id])):
                prob = random.random()
                if prob < 0.1:
                    types = [*range(-1, 2)]
                    types.remove(self.joints[joint_id][i])
                    self.joints[joint_id][i] = types[random.randint(0, len(types) - 1)]
        elif option == 2:
            sizes = [*range(self.leg_lim[0], self.leg_lim[1]+1)]
            sizes.remove(self.num_components)
            new_nc = sizes[random.randint(0, len(sizes)-1)]
            if new_nc > self.num_components:
                difference = new_nc - self.num_components
                for i in range(0, difference):
                    idx = random.randint(0, len(archive) - 1)
                    self.parts.append(copy.deepcopy(list(archive.values())[idx]))
                    self.joints.append(generate_joints_fixed(len(self.parts[-1].x)))
            else:
                self.parts = self.parts[0:new_nc]
                self.joints = self.joints[0:new_nc]
            self.num_components = new_nc
        else:
            pass