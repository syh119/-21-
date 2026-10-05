#!/usr/bin/env python3
import yaml
import numpy as np
from PIL import Image
import os
import struct

def main():
    
    yaml_path = "/root/ros2_ws/src/nav_pkg/maps/111.yaml"
    pcd_save_path = "/root/ros2_ws/src/nav_pkg/maps/my_map.pcd"
    occupied_threshold = 50
    

    
    with open(yaml_path, 'r', encoding='utf-8') as f:
        map_config = yaml.safe_load(f)
    resolution = map_config['resolution']
    origin = map_config['origin']
    pgm_path = map_config['image']
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)

    
    img = Image.open(pgm_path)
    img_array = np.array(img)
    img_array = np.flipud(img_array)
    map_height, map_width = img_array.shape

    
    obstacle_points = []
    for v in range(map_height):
        for u in range(map_width):
            if img_array[v, u] > occupied_threshold:
                world_x = origin[0] + u * resolution
                world_y = origin[1] + v * resolution
                world_z = 0.0
                obstacle_points.append([world_x, world_y, world_z])

    
    points_np = np.array(obstacle_points, dtype=np.float32)
    num_points = len(points_np)

    with open(pcd_save_path, 'wb') as f:
        
        header = f"""# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {num_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {num_points}
DATA binary
"""
        f.write(header.encode('ascii'))
        
        points_np.tofile(f)

    print(f"✅ 转换完成！共生成 {num_points} 个障碍物点")
    print(f"📁 PCD文件已保存到：{pcd_save_path}")

if __name__ == "__main__":
    main()