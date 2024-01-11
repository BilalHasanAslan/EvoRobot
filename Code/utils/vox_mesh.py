import trimesh
import trimesh.interfaces.vhacd
import trimesh.smoothing
import numpy as np
# import open3d as o3d
from shapely.geometry import MultiPoint
from shapely import wkt
import matplotlib.pyplot as plt
import math
import string
import random


# function taken from https://github.com/dougsm/egad/blob/master/egad/viz.py
def plot_mesh(mesh, show_axis):
    sc = trimesh.scene.scene.Scene()
    sc.add_geometry(mesh)
    if show_axis:
        marker = trimesh.creation.axis(origin_size=0.01, axis_radius=0.01, axis_length=1)
        sc.add_geometry(marker)

    sc.set_camera(angles=(np.pi/4, 0, np.pi/4), distance=2, center=mesh.center_mass)

    sc.show()


# Code for converting point cloud to trimesh mesh taken from:
# https://stackoverflow.com/questions/56965268/how-do-i-convert-a-3d-point-cloud-ply-into-a-mesh-with-faces-and-vertices
# http://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.compute_convex_hull
def pc_to_mesh(pc):
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(pc)
    #
    # hull_mesh = pcd.compute_convex_hull()
    #
    # mesh = trimesh.Trimesh(np.asarray(hull_mesh[0].vertices), np.asarray(hull_mesh[0].triangles),
    #                            vertex_normals=np.asarray(hull_mesh[0].vertex_normals))
    #
    # #plot_mesh(mesh)
    # mesh = mesh.convex_hull
    #alternative approach---leads to some non-water-tight cases
    mesh = trimesh.points.PointCloud(pc).convex_hull
    return mesh


# convert a 2D point cloud to a convex polygon
def pc_to_2dCH(pc):
    mpt = MultiPoint(pc)
    shape = mpt.convex_hull.wkt
    poly = wkt.loads(shape)
    return poly


def plot_poly(poly, color):
    x, y = poly.exterior.xy
    plt.plot(x, y, color)


def plot_poly_batch(poly_set, pattern):
    colors = ['b-', 'g-', 'r-', 'y-', 'k-']
    for i in range(0, len(poly_set)):
        plot_poly(poly_set[i], colors[pattern[i]])


# calculate angle of rotation to ensure two lines are parallel
def poly_rotation_angle(line1, line2):
    angle1 = math.atan2(line1.coords.xy[1][0] - line1.coords.xy[1][1],
                        line1.coords.xy[0][0] - line1.coords.xy[0][1])

    angle2 = math.atan2(line2.coords.xy[1][0] - line2.coords.xy[1][1],
                        line2.coords.xy[0][0] - line2.coords.xy[0][1])

    angle = angle1 - angle2
    return math.degrees(angle)


# Calculate angle formed by two points and a centre (essentially 3 points) where ctr is the middle point.
# Taken from here: https://stackoverflow.com/questions/26076656/calculating-angle-between-two-points-java
def component_conn_angle(p1, p2, ctr):
    angle1 = np.math.atan2(p1[1] - ctr[1], p1[0] - ctr[0])
    angle2 = np.math.atan2(p2[1] - ctr[1], p2[0] - ctr[0])

    return math.degrees(angle1 - angle2)


def id_generator(size=5, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))
