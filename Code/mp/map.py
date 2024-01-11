from sklearn.neighbors import KDTree
from sklearn.cluster import KMeans
import multiprocessing
from pathlib import Path
import copy
import utils.vox_mesh as vm
import numpy as np
import os
import pickle
import time

class map_elites(object):
    def __init__(self, dim_map, n_niches, n_gen, params, layer_info):
        self.dim_map = dim_map
        self.n_niches = n_niches
        self.params = params
        self.runid = params['experiment_runid']
        self.n_gen = n_gen
        self.archive = {}
        self.kdt = self._create_kdt()
        self.ind_params = layer_info["ind_params"]
        self.lower_archive = layer_info["lower_layer"]
        self.layer_name = layer_info["layer"]


    def _add_to_archive(self, s):
        niche_index = self.kdt.query([s.desc], k=1)[1][0][0]
        niche = self.kdt.data[niche_index]
        n = self._make_hashable(niche)
        if n in self.archive:
            if s.fitness > self.archive[n].fitness:
                self.archive[n] = s
        else:
            self.archive[n] = s

    def _make_hashable(self, array):
        return tuple(map(float, array))

    def _centroids_filename(self):
        return self.params['run_folder'] + '//centroids_' + str(self.n_niches) + '_' + str(self.dim_map) + '.dat'

    def _save_archive_img(self):
        for k in self.archive.values():
            # k.mesh.export(str(np.round(k.desc,4)).replace(' ', '_')+'.obj')
            vm.save_mesh_png(k.mesh, self.layer_name + "_" + str(np.round(k.desc,4)).replace(' ', '_'))

    def _save_archive(self, gen):
        def write_array(a, f):
            for i in a:
                f.write(str(i) + ' ')

        filename = self.params['run_folder'] + '//'+ self.layer_name + '_archive_' + str(gen) + '.dat'
        with open(filename, 'w') as f:
            for k in self.archive.values():
                f.write(str(k.id) + ' ')
                f.write(str(k.fitness) + ' ')
                write_array(k.desc, f)
                f.write("\n")

    def _write_centroids(self, centroids):
        k = centroids.shape[0]
        dim = centroids.shape[1]
        filename = self._centroids_filename()
        with open(filename, 'w') as f:
            for p in centroids:
                for item in p:
                    f.write(str(item) + ' ')
                f.write('\n')

    def _create_kdt(self):
        # create the CVT
        c = self._cvt()
        kdt = KDTree(c, leaf_size=30, metric='euclidean')
        self._write_centroids(c)
        return kdt

    def _cvt(self):
        # check if we have cached values
        if self.params['cvt_use_cache']:
            fname = self._centroids_filename()
            if Path(fname).is_file():
                print("WARNING: using cached CVT:", fname)
                return np.loadtxt(fname)
        # otherwise, compute cvt
        x = np.random.rand(self.params['cvt_samples'], self.dim_map)
        k_means = KMeans(init='k-means++', n_clusters=self.n_niches,
                         n_init=1, verbose=1, algorithm="full")
        k_means.fit(x)
        return k_means.cluster_centers_
    
    def add_archive(self,archive):
        self.archive = archive

    def compute(self, fitness_function, ind_type,startinggeneration):
        start = time.time()
        num_cores = multiprocessing.cpu_count()
        os.system('taskset -cp 0-%d %s' % (num_cores, os.getpid()))
        pool = multiprocessing.Pool(num_cores)

        init_count = 0
        # main loop
        for g in range(startinggeneration, startinggeneration+self.n_gen + 1):
            eval_list = []
            if g == 0:  # random initialization
                while (init_count <= self.params['random_init'] * self.n_niches):
                    # generate random solutions
                    for i in range(0, self.params['random_init_batch']):
                        s = ind_type(self.ind_params)
                        if self.lower_archive:
                            s.generate_random(self.lower_archive)
                        else:
                            s.generate_random()

                        eval_list += [s]
                    # evaluate all
                    if self.params['parallel']:
                        s_list = pool.map(fitness_function, eval_list)
                        #s_list = pool.map(fitness_function, eval_list)
                    else:
                        s_list = map(fitness_function, eval_list)

                    # update archive
                    for sol in s_list:
                        if sol.evaluated == True:
                            self._add_to_archive(sol)

                    init_count = len(self.archive)
                    eval_list = []
            else:  # variation/selection loop
                # recombine
                keys = list(self.archive.keys())
                for n in range(0, self.params['batch_size']):
    
                    if self.params["mutation_only"]:
                        # parent selection
                        s = copy.deepcopy(self.archive[keys[0]])
                        # copy & add variation
                        if self.lower_archive:
                            #s.mutate(self.lower_archive)
                            pass
                        else:
                            #s.mutate()
                            pass
                    else:
                        # parent selection
                        p1 = copy.deepcopy(self.archive[keys[np.random.randint(len(keys))]])
                        p2 = copy.deepcopy(self.archive[keys[np.random.randint(len(keys))]])
                        # create new individual object and do crossover
                        s = ind_type(self.ind_params)
                        s.crossover(p1, p2)
                        if self.lower_archive:
                            s.mutate(self.lower_archive)
                        else:
                            s.mutate()

                    eval_list += [s]

                # parallel evaluation of the fitness
                if self.params['parallel']:
                    s_list = pool.map(fitness_function, eval_list)
                    #s_list = pool.map(fitness_function, eval_list)
                else:
                    s_list = map(fitness_function, eval_list)

                # natural selection
                for sol in s_list:
                    if sol.evaluated==True:
                        self._add_to_archive(sol)
            
            pickle_out = open(self.params['run_folder']+"/toContinueUpperLayer.pickle","wb")
            pickle.dump(self.archive, pickle_out)
            pickle_out.close()
            file2 = open(self.params['run_folder']+"/LeftOffGeneration.txt","w")
            file2.write(str(g))
            file2.close()
            # write archive
            if g % self.params['dump_period'] == 0 and self.params['dump_period'] != -1:
                print("generation:", g)
                self._save_archive(g)
                end = time.time()
                
                file3 = open(self.params['run_folder']+"/TimeTaken.txt","a")
                file3.write(str(g) +" : "+str(end-start)+"\n")
                file3.close()
                
            self._save_archive(self.n_gen)
        # self._save_archive_img()
        return self.archive






