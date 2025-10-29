import laspy
import numpy as np
import pandas as pd
import open3d as o3d
from tqdm import tqdm
import dbh
def openlas2(path):
    las = laspy.read(path)
    print(las.header.point_count)
    scale = las.header.scales
    offset = las.header.offsets
    # 方法2
    coords = np.vstack((las.x, las.y, las.z)).transpose()
    return coords,offset,scale
    del coords
    # 打印两种方法的点

def openlas(path):
    las = laspy.read(path)
    print(las.header.point_count)
    scale = las.header.scales
    offset = las.header.offsets
    # 方法2
    coords = np.vstack((las.x, las.y, las.z, las.trees)).transpose()
    return coords,offset,scale
    del coords
    # 打印两种方法的点
def datatree_set(datatree,offset,scale,path):
    x = []
    y = []
    z = []
    clas = 0
    classes = []
   # print('yuan', len(datatree))
    for i in range(len(datatree)):
        data = np.array(datatree[i])
        if len(x) != 0:
            x = np.append(x, data[:, 0])
            y = np.append(y, data[:, 1])
            z = np.append(z, data[:, 2])
            classes = np.append(classes, np.full(len(datatree[i]), clas))
            clas = clas + 1
        else:
            x = data[:, 0]
            y = data[:, 1]
            z = data[:, 2]
            classes = np.full(len(datatree[i]), clas)
            clas = clas + 1
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.add_extra_dim(laspy.ExtraBytesParams(name="trees", type=np.int32))
    header.offsets = offset
    header.scales = scale
    # 创建 LAS 文件对象
    outFile = laspy.LasData(header)
    # 添加 LAS 点记录
    outFile.x = x
    outFile.y = y
    outFile.z = z
    outFile.trees = classes.astype(np.uint8)
    outFile.write(path)
####################路径###########################

####################单木###########################

path1 = input("Input file path1: ")
path2 = input("Input file path2: ")
path3 = input("Output file path: ")

datatree,offset,scale= openlas(path1)
#datatree,offset,scale= openlas(r'G:\computer\study\singtree\data\my48\t\single.las')
df = pd.DataFrame(datatree, columns=['A', 'B', 'C', 'D'])
# 按B列的值进行分组
grouped = df.groupby('D')
# 将分组结果转换为仅包含分组数据的列表
grouped_list = [group_data.to_numpy() for _, group_data in grouped]
datatree = grouped_list
print(len(datatree))
# #####################原始###########################
# dataset_points0,offset,scale=openlas2(r'G:\computer\study\singtree\data\Boreal3Dv1\Boreal3D'+lu+r'\MLS_ULS\result_dem.las')
# #dataset_points0,offset,scale=openlas2(r'G:\computer\study\singtree\data\my48\t\result_dem.las')
# pcd0 = o3d.geometry.PointCloud()
# pcd0.points = o3d.utility.Vector3dVector(dataset_points0)
# voxel_size=0.02
# pcd = pcd0.voxel_down_sample(voxel_size=voxel_size)
# del pcd0,dataset_points0
# dataset_points=np.asarray(pcd.points)
# # dataset_points=np.asarray(dataset_points0)

dataset_points0,offset,scale=openlas2(path2)
#dataset_points0,offset,scale=openlas2(r'G:\computer\study\singtree\data\my48\t\result_dem.las')
pcd0 = o3d.geometry.PointCloud()
pcd0.points = o3d.utility.Vector3dVector(dataset_points0)
voxel_size=0.05
pcd = pcd0.voxel_down_sample(voxel_size=voxel_size)
del pcd0,dataset_points0
dataset_points=np.asarray(pcd.points)


import ODR
######################过滤柱状##########################
datatree_new=[]
center_points=[]
Rs=[]
directions=[]
print('柱状过滤')
for i in tqdm(range(0,len(datatree))):
    tree=datatree[i][:,0:3]
    tree_newt,R,direction,center_point=ODR.zhu_filter(tree,0.01,30)
    center_points.append(center_point)
    Rs.append(R)
    directions.append(direction)
    tree_new=tree
    distance = ODR.distance_point_zhu(dataset_points, center_point, direction)
    dataset_yan = dataset_points[distance<=5*R]
    #分段找回密集点云
    tree_low0 = tree[tree[:, 2] < (min(tree[:, 2]) + 5)]
    tree_low, R_x, direction_low, point_low = dbh.zhu_filter(tree_low0, 0.01, 30)
    daun_0=min(tree_new[:,2])
    daun_1=max(tree_new[:,2])
    tree_new2=tree_new[0:2,:]
    while daun_0 < daun_1:
        if daun_0+3 < daun_1:
            duan_tree0=tree_new[tree_new[:,2]<(daun_0+5)]
            duan_tree=duan_tree0[duan_tree0[:,2]>daun_0-5]
            if duan_tree.shape[0]<3:
                tree_new = np.append(tree_new, duan_tree, axis=0)
                tree_new2 = np.append(tree_new2, duan_tree, axis=0)
                daun_0 = daun_0 + 1
                continue
            tree_duan, R_duan, direction_duan, point_duan = ODR.zhu_filter(duan_tree,0.01,30)#每段参数
            # 找出高程范围内的点
            dataset_up0 = dataset_yan[dataset_yan[:, 2] > daun_0]
            dataset_up = dataset_up0[dataset_up0[:, 2] < (daun_0+1)]
            #找出上面点的ODR距离，和每段ODR相比
            distance_duan = ODR.distance_point_zhu(dataset_up, point_duan, direction_duan)
            tree_duan_new = dataset_up[distance_duan<=R_x]
            if tree_duan_new.size > 3:
                tree_new = np.append(tree_new, tree_duan_new, axis=0)
                tree_new2 = np.append(tree_new2,tree_duan_new, axis=0)
        else:#迭代到最高大于树高时
            duan_tree0 = tree_new[tree_new[:, 2] < daun_1]
            duan_tree = duan_tree0[duan_tree0[:, 2] > daun_0-5]
            if duan_tree.size>4:
                tree_duan, R_duan, direction_duan, point_duan = ODR.zhu_filter(duan_tree,0.01,30)  # 每段参数
                # 找出高程范围内的点
                dataset_up0 = dataset_yan[dataset_yan[:, 2] > daun_0]
                dataset_up = dataset_up0[dataset_up0[:, 2] < daun_1]
                # 找出上面点的ODR距离，和每段ODR相比
                distance_duan = ODR.distance_point_zhu(dataset_up, point_duan, direction_duan)
                tree_duan_new = dataset_up [distance_duan <= R_x]
                if tree_duan_new.size > 3:
                    tree_new = np.append(tree_new, tree_duan_new, axis=0)
                    tree_new2 = np.append(tree_new2, tree_duan_new, axis=0)
        daun_0 = daun_0 + 1
    #tree_new, R, direction, center_point = ODR.zhu_filter(tree_new, 0.01, 30)
    datatree_new.append(tree_new)
del datatree,tree
######################柱状延申_low##########################
print('\n柱状延申_向下')
datatree_low=[]
for i in tqdm(range(0,len(datatree_new))):
    tree=datatree_new[i]
    center_point = center_points[i]
    R = Rs[i]
    direction = directions[i]
    distance = ODR.distance_point_zhu(dataset_points, center_point, direction)
    dataset_yan = dataset_points[distance<=3*R]
    tan=tree.shape[0]-1
    #设置下端检测半径
    tree_low0 = tree[tree[:, 2] < (min(tree[:, 2]) + 3)]
    tree_low, R_x, direction_low, point_low = dbh.zhu_filter(tree_low0, 0.01, 80)
    #顶端的ODR
    while tan<tree.shape[0]:
        tan=tree.shape[0]
        tree_low0=tree[tree[:,2]<(min(tree[:,2])+3)]
        tree_low,R_low,direction_low,point_low=dbh.zhu_filter(tree_low0,0.01,30)
        #找出高程范围内的点
        dataset_low0=dataset_yan[dataset_yan[:,2]>(min(tree[:,2])-0.2)]#0.15
        dataset_low = dataset_low0[dataset_low0[:, 2] < (min(tree[:, 2]))]
        #找出上面点的ODR距离，和顶端ODR相比
        distance_low = ODR.distance_point_zhu(dataset_low , point_low, direction_low)
        tree_yan=dataset_low[distance_low<=R_x]#R_low
        if tree_yan.size > 3:
            tree = np.append(tree, tree_yan, axis=0)
    datatree_low.append(tree)
   # tree_yan = dataset_points[distance <=R]  # 5R范围内的点
del datatree_new,tree

######################柱状延申_up##########################
print('\n柱状延申_向上')
datatree_up=[]
DBH_set=[]
for i in tqdm(range(0,len(datatree_low))):
    tree=datatree_low[i]
    center_point = center_points[i]
    R = Rs[i]
    direction = directions[i]
    distance = ODR.distance_point_zhu(dataset_points, center_point, direction)

    tan=tree.shape[0]-1
    # 设置下端检测半径
    tree_up0=tree[tree[:,2]<(min(tree[:,2])+5)]
    tree_up0 = tree_up0[tree_up0[:, 2] > (min(tree[:, 2]))]
    tree_up1=tree_up0[tree_up0[:,2]>(min(tree[:,2])+2)]

    if tree_up1.shape[0]>3:
        tree_up,R_s0,direction_up,point_up=dbh.zhu_filter(tree_up0,0.01,30)#0~5
        tree_up, R_s1, direction_up, point_up = dbh.zhu_filter(tree_up1, 0.01, 30)#2~5
        if R_s0>1.5*R_s1:
            R_s=R_s1
        else:
            R_s=R_s0
    else:
        tree_up, R_s, direction_up, point_up = dbh.zhu_filter(tree, 0.01, 30)
    tall = tree[:, 2].max() - tree[:, 2].min()
    if R_s<0.1:
        R_s=0.1
    if tall<5:
        tall=5
    DBH_set.append(R_s)
    p_yan = tall * R_s
    dataset_yan = dataset_points[distance < p_yan]  #######################G11:5,G12:4 J1:5其他3
    while tan<tree.shape[0]:
        tan=tree.shape[0]
        tree_up0=tree[tree[:,2]>(max(tree[:,2])-3)]
        if tree_up0.shape[0]>3:
            tree_up,R_up,direction_up,point_up=dbh.zhu_filter(tree_up0,0.01,30)
            #找出高程范围内的点
            dataset_up0=dataset_yan[dataset_yan[:,2]>(max(tree[:,2]))]
            xia=0.1
            dataset_up = dataset_up0[dataset_up0[:, 2] < (max(tree[:, 2])+xia)]
            #找出上面点的ODR距离，和顶端ODR相比
            distance_up = ODR.distance_point_zhu(dataset_up , point_up, direction_up)
            tree_yan=dataset_up[distance_up<=1.5*R_s]
            if tree_yan.size > 3:
                tree = np.append(tree, tree_yan, axis=0)
    datatree_up.append(tree)
   # tree_yan = dataset_points[distance <=R]  # 5R范围内的点
# del datatree_low,tree
# datatree_set(datatree_up,offset,scale,r'data'+lu+'_filtert.las')
######################柱状恢复##########################
print('\n柱状恢复')
datatree_up2=[]
tallset=[]
for i in tqdm(range(0,len(datatree_up))):
    tree=datatree_up[i]
    center_point = center_points[i]
    R_s=DBH_set[i]
    direction = directions[i]
    distance = ODR.distance_point_zhu(dataset_points, center_point, direction)

    tan=tree.shape[0]-1

    tall = tree[:, 2].max() - tree[:, 2].min()
    p_yan = tall * R_s*5
    dataset_yan = dataset_points[distance < p_yan]  #######################G11:5,G12:4 J1:5其他3
    #ODR.hua(tree_up,point_up,direction_up)
    tallset.append(tall)
    tree_up0 = tree[tree[:, 2] < (min(tree[:, 2]) + tall)]
    tree_up, R_up, direction_up, point_up = dbh.zhu_filter(tree_up0, 0.01, 30)
    #顶端的ODR
    while tan<tree.shape[0]:
        tan=tree.shape[0]
        tall_t = tree[:, 2].max() - tree[:, 2].min()
        if tree_up0.shape[0]>3:
            #找出高程范围内的点
            dataset_up0=dataset_yan[dataset_yan[:,2]>(max(tree[:,2]))]
            xia=tall*0.05
            if xia<0.2:
                xia=0.2
            dataset_up = dataset_up0[dataset_up0[:, 2] < (max(tree[:, 2])+xia)]
            #找出上面点的ODR距离，和顶端ODR相比
            distance_up = ODR.distance_point_zhu(dataset_up , point_up, direction_up)
            # if tall_t>tall*1.5:
            #     xia_y=R_s*tall/3
            # else:
            #     xia_y=R_s*(-(0.5*((tall_t-tall)-tall/4))**2+tall/3+(0.5*tall/4)**2)
            # try:
            #     xia_y=min([max(distance_upt)+0.04,xia_y])
            #     xia_y = max([R_s*tall/5, xia_y])
            xia_y = 0.02 * (-(0.5*(tall_t - tall)) ** 2 + (0.5 * tall) ** 2)
            xia_y = max([0.02 * (-(0.5*(tall * 1.5 - tall)) ** 2 + (0.5 * tall) ** 2), xia_y])
            try:
                xia_y=min([max(distance_upt)+0.04,xia_y])
                xia_y = max([0.01 * (-(0.5*(tall * 1.5 - tall)) ** 2 + (0.5 * tall) ** 2), xia_y])
            except:
                xia_y=xia_y
            if xia_y<0.2:
                xia_y=0.2

            tree_yan=dataset_up[distance_up<=xia_y]
            distance_upt=distance_up[distance_up<=xia_y]
            #防止大的间隙
            # if tall_t>tall*1.5:
            #      xia_y=0.2
            if tree_yan.size > 3:
                tree = np.append(tree, tree_yan, axis=0)
    datatree_up2.append(tree)
   # tree_yan = dataset_points[distance <=R]  # 5R范围内的点
del datatree_up,tree

######################过滤柱状##########################
datatree_new=[]
print('\n过滤树干')
for i in tqdm(range(0,len(datatree_up2))):
    tree=datatree_up2[i]
    num_bins = 40
    ##########################筛选单点
    # 计算直方图
    observed_freq, bin_edges = np.histogram(tree[:, 2], bins=num_bins)
    # 找到 observed_freq < 1 的 bin 的索引
    invalid_bins = np.where(observed_freq < 4)[0]
    # 找到这些 bin 对应的范围
    invalid_ranges = [(bin_edges[i], bin_edges[i + 1]) for i in invalid_bins]
    # 筛选出 data 中不在这些范围内的行
    mask = np.ones(len(tree), dtype=bool)  # 初始化掩码，默认保留所有行
    for low, high in invalid_ranges:
        mask &= ~((tree[:, 2] >= low) & (tree[:, 2] <=high))  # 剔除不满足条件的行
    tree= tree[mask]  # 应用掩码，筛选数据
    t= tree[:, 2].max() - tree[:, 2].min()
    tall = tallset[i]
    DBH = DBH_set[i]
    ##########################检查空隙，大于树高的0.2,排除
    tree = tree[np.argsort(tree[:, 2])]
    zs = tree[:, 2]
    b = zs[:-1]
    b = np.insert(b, 0, zs[0])
    c = zs - b
    xian = tall * DBH
    pt = np.where(c > xian)
    if pt[0].size > 0:
        tree = tree[:pt[0][0]]
    tall = tree[:, 2].max() - tree[:, 2].min()
    if tree.shape[0] > 3 and tall > 5:  ##############################
        datatree_new.append(tree)
del datatree_up2
datatree_set(datatree_new,offset,scale,path3)
#datatree_set(datatree_new,offset,scale,r'G:\computer\study\singtree\data\my48\t\result_filter.las')


