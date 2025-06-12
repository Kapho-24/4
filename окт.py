import trimesh
import numpy as np
from trimesh.creation import box
from trimesh.transformations import translation_matrix
from trimesh.util import concatenate

# PARAMETERS
diameter = 100.0
thickness = 0.12 * diameter
target_cube_count = 500_000  # future use
octagon_sides = 8
center2d = np.array([0.0, 0.0])

# Utility Functions
def to_3d(p2d): return np.append(p2d, 0.0)
def create_regular_polygon(sides, radius):
    angles = np.linspace(0, 2 * np.pi, sides, endpoint=False)
    return np.stack((np.cos(angles), np.sin(angles)), axis=1) * radius
def edge_length(p1, p2): return np.linalg.norm(p2 - p1)

# Base Geometry
radius = diameter / 2
octagon_points = create_regular_polygon(octagon_sides, radius)
edge_len = edge_length(octagon_points[0], octagon_points[1])
box_size_xy = (diameter ** 2 * np.pi) / target_cube_count  # crude spacing estimate
box_size = np.array([box_size_xy, box_size_xy, thickness])
octagon_points_3d = [to_3d(p) for p in octagon_points]
boxes = []

# Equilateral triangle ring (8 triangles)
apex_list = []
for i in range(octagon_sides):
    p1 = octagon_points_3d[i]
    p2 = octagon_points_3d[(i + 1) % octagon_sides]
    mid = (p1 + p2) / 2
    dir_vec = mid[:2] - center2d
    dir_vec = dir_vec / np.linalg.norm(dir_vec) * (edge_len * np.sqrt(3) / 2)
    apex2d = mid[:2] - dir_vec
    apex = to_3d(apex2d)
    apex_list.append(apex)
    
    triangle_pts = [p1, p2, apex]
    for j in range(3):
        start, end = triangle_pts[j], triangle_pts[(j + 1) % 3]
        edge_vec = end - start
        steps = max(1, int(np.linalg.norm(edge_vec) / box_size_xy))
        for k in range(steps):
            pos = start + edge_vec * (k / steps)
            transform = translation_matrix([*pos[:2], thickness / 2])
            boxes.append(box(extents=box_size, transform=transform))

# Inner 24 triangles
for i in range(octagon_sides):
    v1 = octagon_points_3d[i]
    v2 = apex_list[i]
    dir_to_center = center2d - v1[:2]
    v3 = to_3d(v1[:2] + dir_to_center / np.linalg.norm(dir_to_center) * edge_len)
    triangle_pts = [v1, v2, v3]
    for j in range(3):
        start, end = triangle_pts[j], triangle_pts[(j + 1) % 3]
        edge_vec = end - start
        steps = max(1, int(np.linalg.norm(edge_vec) / box_size_xy))
        for k in range(steps):
            pos = start + edge_vec * (k / steps)
            transform = translation_matrix([*pos[:2], thickness / 2])
            boxes.append(box(extents=box_size, transform=transform))

# Central 8 triangles
for i in range(octagon_sides):
    v1 = apex_list[i]
    v2 = apex_list[(i + 1) % octagon_sides]
    v3 = to_3d(center2d)
    triangle_pts = [v1, v2, v3]
    for j in range(3):
        start, end = triangle_pts[j], triangle_pts[(j + 1) % 3]
        edge_vec = end - start
        steps = max(1, int(np.linalg.norm(edge_vec) / box_size_xy))
        for k in range(steps):
            pos = start + edge_vec * (k / steps)
            transform = translation_matrix([*pos[:2], thickness / 2])
            boxes.append(box(extents=box_size, transform=transform))

# Combine and export
combined = concatenate(boxes)
combined.export("octagon_molde.glb")
print("Exported: octagon_molde.glb")
