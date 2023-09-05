import open3d as o3d
import numpy as np
# z
MY_RATE=0.05

def random_add(pcd):
    aabb = pcd.get_axis_aligned_bounding_box()
    pt_sz = int(len(pcd.points)*MY_RATE)
    # z=np.random.rand(int(len(pcd.points)*0.01),3)
    zp = o3d.geometry.PointCloud()
    box = np.asarray(aabb.get_box_points())
    mi, ma = np. min(box, axis=0), np.max(box, axis=0)
    # print(

    ax = np.random.uniform(mi[0], ma[0], pt_sz)
    ay = np.random.uniform(mi[1], ma[1], pt_sz)
    az = np.random.uniform(mi[2], ma[2], pt_sz)
    print()
    zp.points = o3d.utility.Vector3dVector(np.vstack((ax, ay, az)).transpose())
    zp.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([zp, pcd, aabb])
    return zp


def filter_radius(pcd, MinPts=5, R=0.025):
    # 邻域球内的最少点个数，小于该个数为噪声点
    # 邻域半径大小
    # pc 去噪后的点云
    # idx 去噪保留的点索引
    pc, idx = pcd.remove_radius_outlier(MinPts, R)
    pc.paint_uniform_color([0, 0, 1])
    ror_noise_pcd = pcd.select_by_index(idx, invert=True)
    ror_noise_pcd.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([pc, ror_noise_pcd], window_name="半径滤波")
    return pc+ ror_noise_pcd,pc

def filter_statistical(pcd,k = 20,μ = 1.0):
    # 统计滤波
     # K邻域点的个数
     # 标准差乘数
    # 当判断点的k近邻的平均距离大于【平均距离+μ*σ】，即判定为噪声点，一般取μ=2或3为极限误差
    sor_pcd, idx = pcd.remove_statistical_outlier(k, μ)
    sor_pcd.paint_uniform_color([0, 0, 1])
    # 提取噪声点云
    sor_noise_pcd = pcd.select_by_index(idx, invert=True)
    sor_noise_pcd.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries(
    #     [sor_pcd, sor_noise_pcd], window_name="SOR")
    return sor_pcd+ sor_noise_pcd,sor_pcd

def ball_add(pcd, r=0.05, rate=0.15):
    ptsz = 10000
    aabb = pcd.get_axis_aligned_bounding_box()
    pt_sz = int(len(pcd.points))
    zp = o3d.geometry.PointCloud()
    box = np.asarray(aabb.get_box_points())
    pcdaz = np.asarray(pcd.points)

    t = np.random.uniform(0, 2*np.pi, ptsz)
    fy = np.random.uniform(-np.pi/2, np.pi/2, ptsz)
    ax = r*np.sin(t)*np.cos(fy)
    ay = r*np.sin(t)*np.sin(fy)
    az = r*np.cos(t)
    nparr = np.vstack((ax, ay, az)).transpose()
    zp.points = o3d.utility.Vector3dVector(nparr)
    # o3d.visualization.draw_geometries([zp,aabb])
    pcdaz_i = np.random.randint(0, pt_sz, int(pt_sz*rate))
    p2 = []
    for i in pcdaz_i:
        print(nparr[np.random.randint(0, ptsz)])  # fang hui
        p2.append(pcdaz[i]+nparr[np.random.randint(0, ptsz)])
    # pcd.remove_statistical_outlier(20,0.5,False) #
    # pcd.points=o3d.utility.Vector3dVector(pcdaz)
    # f1(pcd)
    # f2(pcd)
    ret = o3d.geometry.PointCloud()
    ret.points = o3d.utility.Vector3dVector(np.array(p2))
    ret.paint_uniform_color([1, 0, 0])
    return ret


pcd = o3d.io.read_point_cloud("p1.xyz")
o3d.io.write_point_cloud("data/src.ply",pcd,write_ascii=True, compressed=False, print_progress=False)

pcd.paint_uniform_color([0, 0, 1])
ret_noise1 = ball_add(pcd,0.05,MY_RATE)
ret_noise2 = random_add(pcd)
merge_1 = pcd+ret_noise1
merge_2 = pcd+ret_noise2
# o3d.visualization.draw_geometries([merge_1], "add noise")
import os
os.system("mkdir -p data")
o3d.io.write_point_cloud("data/noise2.ply",ret_noise1,write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud("data/noise1.ply",ret_noise2,write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud("data/merge_1.ply",merge_1,write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud("data/merge_2.ply",merge_2,write_ascii=True, compressed=False, print_progress=False)


ret,l1=filter_radius(merge_1)
o3d.io.write_point_cloud("data/radius_merge_1.ply",ret,write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud("data/radius_1.ply",l1,write_ascii=True, compressed=False, print_progress=False)

ret,l1=filter_radius(merge_2)
o3d.io.write_point_cloud("data/radius_merge_2.ply",ret,write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud("data/radius_2.ply",l1,write_ascii=True, compressed=False, print_progress=False)

ret,l1=filter_statistical(merge_1)
o3d.io.write_point_cloud("data/statistical_merge_1.ply",ret,write_ascii=True, compressed=False, print_progress=False)
o3d.io.write_point_cloud("data/statistical_1.ply",l1,write_ascii=True, compressed=False, print_progress=False)

ret,l1=filter_statistical(merge_2)
o3d.io.write_point_cloud("data/statistical_merge_2.ply",ret,write_ascii=True, compressed=False, print_progress=False)

o3d.io.write_point_cloud("data/statistical_2.ply",l1,write_ascii=True, compressed=False, print_progress=False)
