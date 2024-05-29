import open3d as o3d

if __name__ == "__main__":
    print("Testing IO for textured meshes ...")
    textured_mesh = o3d.io.read_point_cloud("config/obstacles/obstacle_00/mesh.ply")
    # textured_mesh = o3d.io.read_triangle_mesh("config/obstacles/obstacle_00/mesh.stl")
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(textured_mesh)
    vis.run()
    print(textured_mesh)


# import pymesh

# pymesh.meshio.load_mesh("config/obstacles/obstacle_00/mesh.stl")


# import numpy as np
# from stl import mesh

# stl_data = mesh.Mesh.from_file("config/obstacles/obstacle_00/mesh.stl")
