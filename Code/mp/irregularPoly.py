import random
import utils.vox_mesh as vm
import shapely
import copy
from shapely import affinity
from shapely.geometry import LineString


def extract_edge(polyg, idx):
    return LineString([(polyg.exterior.xy[0][idx], polyg.exterior.xy[1][idx]), (polyg.exterior.xy[0][idx + 1],
                                                                                polyg.exterior.xy[1][idx + 1])])


def poly_to_mesh(poly):
    pc = []
    for j in range(0, len(poly.exterior.xy[0]) - 1):
        pc.append([poly.exterior.xy[0][j], poly.exterior.xy[1][j], 0])
        pc.append([poly.exterior.xy[0][j], poly.exterior.xy[1][j], 0.5])
    return vm.pc_to_mesh(pc)


def generate_poly(num_points,coords):
    edge_idx_out = []
    edge_idx_in = []
    #coords = [[random.random(), random.random()] for _ in range(num_points)]
    poly = vm.pc_to_2dCH(coords)
    poly = shapely.affinity.translate(poly, xoff=0.5 - poly.centroid.coords[0][0], yoff=0.5 - poly.centroid.coords[0][1]
                                      , zoff=0.0)
    idx = random.randint(0, len(poly.exterior.xy[0]) - 2)
    edge_idx_out.append(idx)
    ban_list = []
    if (len(poly.exterior.xy[0]) - 2) == idx:
        ban_list.append(0)
    else:
        ban_list.append(idx + 1)

    if idx == 0:
        ban_list.append(len(poly.exterior.xy[0]) - 2)
    else:
        ban_list.append(idx - 1)

    while len(edge_idx_in) < 1:
        idx = random.randint(0, len(poly.exterior.xy[0]) - 2)
        if (idx not in edge_idx_in) and (idx not in edge_idx_out):
            if len(poly.exterior.xy[0]) > 4:
                if (idx not in ban_list):
                    edge_idx_in.append(idx)
            else:
                edge_idx_in.append(idx)
    return poly, edge_idx_in, edge_idx_out


class irregularPoly(object):

    def __init__(self, params):
        self.desc = []
        self.fitness = 0
        self.dim_x = params["num_points"]
        self.num_components = params["num_components"]
        self.evo_type = params["evo_type"]
        self.grammar_iterations = params["grammar_iterations"]
        self.friction = []
        self.restitution = []
        self.x = []
        self.grammar = []
        self.poly = []
        self.poly_materials = []
        self.edge_idx_out = []
        self.edge_idx_in = []
        self.evaluated = False
        self.valid = True
        self.mesh = []
        self.mesh_sym = []
        self.union_mesh = None
        self.pattern = []
        self.connected_to = []
        self.connection_edge = []
        self.offset = []
        self.axiom = None
        self.id = None

    def generate_random(self, archive):
        for i in range(0, self.num_components):
            new_poly, new_edge_in, new_edge_out = generate_poly(self.dim_x)
            self.poly.append(new_poly)
            self.edge_idx_in.append(new_edge_in)
            self.edge_idx_out.append(new_edge_out)
            idx = random.randint(0, len(archive) - 1)
            self.poly_materials.append(copy.deepcopy(archive[idx]))
        self.grammar = []
        for i in range(0, self.num_components):
            if random.random() > 0.5:
                self.grammar.append([random.randint(0, self.num_components - 1), random.randint(0, self.num_components
                                                                                                -1)])
            else:
                self.grammar.append([random.randint(0, self.num_components - 1)])
        self.axiom = random.randint(0, len(self.grammar) - 1)
        self.id = vm.id_generator()

    def is_valid(self):
        cross = LineString([(-10, 0.1), (10, 0.1)])
        for i in range(0, len(self.x)):
            for j in range(0, len(self.x)):
                if i != j:
                    if self.x[i].intersects(self.x[j]) or self.x[i].intersects(cross):
                        return False
        return True

    def get_fitness(self):
        return self.fitness

    def set_fitness(self, fitness):
        self.fitness = fitness

    def mutate(self, archive):
        self.id = vm.id_generator()
        option = random.randint(1, 4)
        if option == 1:
            idx = random.randint(0, len(self.poly)-1)
            new_poly, new_edge_in, new_edge_out = generate_poly(self.dim_x)
            self.poly[idx] = new_poly
            self.edge_idx_out[idx] = new_edge_out
            self.edge_idx_in[idx] = new_edge_in
        elif option == 2:
            idx = random.randint(0, len(self.grammar)-1)
            if random.random() > 0.5:
                self.grammar[idx] = [random.randint(0, self.num_components - 1), random.randint(0, self.num_components - 1)]
            else:
                self.grammar[idx] = [random.randint(0, self.num_components - 1)]
        elif option == 3:
            ax = self.axiom
            while ax == self.axiom:
                ax = random.randint(0, len(self.grammar) - 1)
            self.axiom = ax
        elif option == 4:
            poly_id = random.randint(0, self.num_components - 1)
            idx = random.randint(0, len(archive) - 1)
            self.poly_materials[poly_id] = copy.deepcopy(archive[idx])

    def assemble(self):
        translation = []
        final = []
        friction = []
        restitution = []
        poly = self.poly
        connected_edge = self.connection_edge
        connected_to = self.connected_to
        pattern = self.pattern
        edge_idx_in = self.edge_idx_in
        edge_idx_out = self.edge_idx_out

        cross_section_edge = LineString([(0, 1), (1, 1)])
        first_shape = poly[pattern[0]]
        edge_out_first = extract_edge(first_shape, edge_idx_out[pattern[0]][0])
        angle = vm.poly_rotation_angle(cross_section_edge, edge_out_first)
        rot_poly = affinity.rotate(copy.deepcopy(first_shape), angle, edge_out_first.centroid)
        rot_poly = affinity.rotate(copy.deepcopy(rot_poly), 180, edge_out_first.centroid)
        rot_poly = shapely.affinity.translate(rot_poly, xoff=0.5 - rot_poly.centroid.coords[0][0],
                                              yoff=0.5 - rot_poly.centroid.coords[0][1], zoff=0.0)
        final.append(rot_poly)
        if len(pattern) > 1:
            for i in range(0, len(pattern) - 1):
                edge_in = extract_edge(final[connected_to[i]], edge_idx_in[pattern[connected_to[i]]][connected_edge[i]])
                edge_out = extract_edge(poly[pattern[i + 1]], edge_idx_out[pattern[i + 1]][0])

                angle = vm.poly_rotation_angle(edge_in, edge_out)
                x_dist = edge_in.centroid.coords[0][0] - 0.5
                y_dist = edge_in.centroid.coords[0][1] - 0.5
                x_dist_o = edge_out.centroid.coords[0][0] - 0.50
                y_dist_o = edge_out.centroid.coords[0][1] - 0.50

                temp = shapely.affinity.translate(poly[pattern[i + 1]], xoff=x_dist, yoff=y_dist, zoff=0.0)
                temp = shapely.affinity.translate(temp, xoff=-1 * x_dist_o, yoff=-1 * y_dist_o, zoff=0.0)
                temp_edge = extract_edge(temp, edge_idx_out[pattern[i + 1]][0])

                rot_poly = affinity.rotate(copy.deepcopy(temp), angle, temp_edge.centroid)
                rot_poly = affinity.rotate(copy.deepcopy(rot_poly), 180, temp_edge.centroid)

                translation.append([rot_poly.centroid.coords[0][0] - 0.5, rot_poly.centroid.coords[0][1] - 0.5])
                if translation[i][0] < 0:
                    translation[i][0] = translation[i][0]-0.05
                else:
                    translation[i][0] = translation[i][0] + 0.05
                if translation[i][1] < 0:
                    translation[i][1] = translation[i][1] - 0.05
                else:
                    translation[i][1] = translation[i][1] + 0.05

                final.append(shapely.affinity.translate(rot_poly, xoff=0.5 - rot_poly.centroid.coords[0][0],
                                                        yoff=0.5 - rot_poly.centroid.coords[0][1], zoff=0.0))
        for i in range(0, len(pattern)):
            friction.append(self.poly_materials[pattern[i]][0])
            restitution.append(self.poly_materials[pattern[i]][1])

        disp_poly = [final[0]]
        if len(final) > 1:
            for i in range(0, len(final) - 1):
                if i == 0:
                    disp_poly.append(shapely.affinity.translate(final[i + 1], xoff=translation[i][0],
                                                                yoff=translation[i][1], zoff=0.0))
                else:
                    temp = shapely.affinity.translate(final[i + 1], xoff=disp_poly[connected_to[i]].centroid.coords[0][0] - 0.5,
                                                      yoff=disp_poly[connected_to[i]].centroid.coords[0][1] - 0.5,
                                                      zoff=0.0)
                    disp_poly.append(shapely.affinity.translate(temp, xoff=translation[i][0],
                                                                yoff=translation[i][1], zoff=0.0))
        self.x = disp_poly
        self.friction = friction
        self.restitution = restitution

    def update_offset(self):
        offset = [[0, 0]]
        for i in range(0, len(self.x)-1):
            ctr1 = self.x[i].centroid.coords[0]
            ctr2 = self.x[i+1].centroid.coords[0]
            offset.append([ctr2[0] - ctr1[0], ctr2[1] - ctr1[1]])
        self.offset = offset

    def decode_grammar(self):

        def find_index(sequence, idx):
            if idx == 0:
                return idx + len(sequence[idx]) - 1
            else:
                index = 0
                for l in range(0, idx):
                    index = index + len(sequence[l])
                return index + len(sequence[idx]) - 1
        grammar = self.grammar
        pattern = [self.axiom]
        connected_to = []
        connection_edge = []
        for i in range(0, self.grammar_iterations):
            temp = []
            temp_connection = []
            temp_edge = []
            available_edges = []
            seq = []
            for j in range(len(pattern)):
                key = pattern[j]
                seq.append(grammar[key])
                for k in range(0, len(grammar[key])):
                    temp.append(grammar[key][k])
                    if len(temp) > 1:
                        if k == 0:
                            temp_connection.append(find_index(seq, connected_to[j-1]))
                            temp_edge.append(available_edges[temp_connection[-1]].pop(0))
                            base = len(temp)-1
                        else:
                            temp_connection.append(base)
                            temp_edge.append(available_edges[base].pop(0))
                        available_edges.append([0, 1])
                    else:
                        base = 0
                        available_edges.append([0, 1])

            pattern = temp
            connected_to = temp_connection
            connection_edge = temp_edge
        self.pattern = pattern
        self.connected_to = connected_to
        self.connection_edge = connection_edge

    def generate_mesh(self):
        poly = self.x
        mesh = []
        mesh_sym = []
        for i in range(0, len(poly)):

            poly_ctr = copy.deepcopy(poly[i])
            x_dist = poly_ctr.centroid.coords[0][0] - 0.5
            y_dist = poly_ctr.centroid.coords[0][1] - 0.5
            poly_ctr = shapely.affinity.translate(poly_ctr, xoff=-1*x_dist, yoff=-1*y_dist, zoff=0.0)
            mesh.append(poly_to_mesh(poly_ctr))
            sym = copy.deepcopy(mesh[i])
            sym.apply_transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            sym.apply_translation([0, 1, 0])
            mesh_sym.append(sym)
        self.mesh = mesh
        self.mesh_sym = mesh_sym
