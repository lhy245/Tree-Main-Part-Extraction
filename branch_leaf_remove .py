import open3d as o3d
import numpy as np
import laspy
from tqdm import tqdm
import copy


def cylindrical_filter(point_cloud, radius,long, die, min_neighbors, max_var):
    filtered_points = []
    points = np.asarray(point_cloud.points)
    #将点投影到二维平面
    yuan_point=copy.deepcopy(points)
    yuan_point[:,2]=0
    yuan_pcd = o3d.geometry.PointCloud()
    yuan_pcd.points = o3d.utility.Vector3dVector(yuan_point)
    kdtree = o3d.geometry.KDTreeFlann(yuan_pcd)
    for i in tqdm(range(points.shape[0])):
        # 初步筛选在高度和半径范围内的点
        point = points[i]

        tp1 = copy.deepcopy(point)
        tp1[2] = 0
        #进行大圆搜索
        [n1, idx1, _] = kdtree.search_radius_vector_3d(tp1, radius)
        yuan1_point=points[idx1]

        a=yuan1_point[:,2]#无上限圆柱内的点
        c=point[2]
        number1 = a[(a < (c + long)) & (a > (c - long))]
        number = [number1.size]
        for i in range(die):
            number2 = a[(a < (c + long +long*(i + 1))) & (a > (c - long + long*(i + 1)))]
            number3 = a[(a < (c + long - long*(i + 1))) & (a > (c - long -long* (i + 1)))]
            number.append(number2.size)
            number.append(number3.size)

        number=np.array(number)
        max_std =np.std(number)
        mean_nei = np.mean(number)
        pan_d=mean_nei/(max_std+1)
        if  mean_nei >= min_neighbors and pan_d>=max_var:
            filtered_points.append(point)

    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    return filtered_point_cloud
def mid_filter(point_cloud, D1, D2,min_n,long):
    #二维化
    er_points_pcd=copy.deepcopy(point_cloud)
    er_points=np.asarray(er_points_pcd.points)
    er_points[:,2]=0#纵坐标赋值为0
    er_pcd=o3d.geometry.PointCloud()
    er_pcd.points=o3d.utility.Vector3dVector(er_points)

    kdtree = o3d.geometry.KDTreeFlann(er_pcd)
    filtered_points = []
    points = np.asarray(point_cloud.points)

    for i in tqdm(range(points.shape[0])):
        # 初步筛选在高度和半径范围内的点
        point = points[i]
        point_t=copy.deepcopy(point)
        point_t[2]=0
        [n1, idx1, _] = kdtree.search_radius_vector_3d(point_t, D1)
        [n2, idx2, _] = kdtree.search_radius_vector_3d(point_t, D2)
        p1=points[idx1,2]
        p2=points[idx2,2]
        n1=p1[(p1<(point[2]+2*long))&(p1>(point[2]-2*long))].shape[0]
        n2 = p2[(p2 < (point[2] + 2 * long)) & (p2 > (point[2] - 2 * long))].shape[0]
        pan_d=n1/n2
        if pan_d>min_n:
            filtered_points.append(point)

    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    return filtered_point_cloud

def yin_filter(point_cloud, radius,long, die):
    filtered_zvar = []
    points = np.asarray(point_cloud.points)
    # 将点投影到二维平面
    yuan_point = copy.deepcopy(points)
    yuan_point[:, 2] = 0
    yuan_pcd = o3d.geometry.PointCloud()
    yuan_pcd.points = o3d.utility.Vector3dVector(yuan_point)
    kdtree = o3d.geometry.KDTreeFlann(yuan_pcd)
    for i in tqdm(range(points.shape[0])):
        # 初步筛选在高度和半径范围内的点
        point = points[i]

        tp1 = copy.deepcopy(point)
        tp1[2] = 0
        # 进行大圆搜索
        [n1, idx1, _] = kdtree.search_radius_vector_3d(tp1, radius)
        yuan1_point = points[idx1]

        a = yuan1_point[:, 2]  # 无上限圆柱内的点
        c = point[2]
        number1 = a[(a < (c + long)) & (a > (c - long))]
        number = [number1.size]
        for i in range(die):
            number2 = a[(a < (c + long +  long * (i + 1))) & (a > (c - long + long * (i + 1)))]
            number3 = a[(a < (c + long -  long * (i + 1))) & (a > (c - long - long * (i + 1)))]
            number.append(number2.size)
            number.append(number3.size)

        number = np.array(number)
        max_std = np.std(number)
        mean_nei = np.mean(number)
        pan_d = mean_nei / (max_std + 1)
        filtered_zvar.append(pan_d )

    return np.array(filtered_zvar)

def openlas(path):
    las = laspy.read(path)
    scale = las.header.scales
    offset = las.header.offsets
    # srs=las.header.srs
    # 方法2
    coords = np.vstack((las.x, las.y, las.z)).transpose()
    return coords, scale, offset
    del coords
    # 打印两种方法的点
def arrtolas(points,path):
    header = laspy.LasHeader(point_format=3, version="1.2")
    # header.offsets = offset
    # header.scales = scale
    # 创建 LAS 文件对象
    header.add_extra_dim(laspy.ExtraBytesParams(name="Constant", type=np.float32))
    outFile = laspy.LasData(header)

    # 添加 LAS 点记录
    outFile.x = points[:,0]
    outFile.y = points[:,1]
    outFile.z = points[:,2]
    outFile.Constant = points[:, 3]

    outFile.write(path)
####################路径###########################

path1 = input("Input file path: ")
path2 = input("Output file path: ")
points, scale, offset = openlas(path1)
# 使用 open3d 加载点云数据——
pcd0 = o3d.geometry.PointCloud()
pcd0.points = o3d.utility.Vector3dVector(points)
# 示例用法
voxel_size=0.05
pcd = pcd0.voxel_down_sample(voxel_size=voxel_size)
radius=0.1
long=0.2
#柱状去除
filtered_pcd = cylindrical_filter(pcd, radius=radius, long=long,die=2, min_neighbors=2, max_var=4)

d1=radius
d2=radius*2
minn=(d1*d1)/(d2*d2)
filtered_pcd = mid_filter(filtered_pcd,d1,d2,minn,long)
#
filtered_pcd = cylindrical_filter(filtered_pcd, radius=radius, long=long,die=2, min_neighbors=2, max_var=3)
filtered_pcd = mid_filter(filtered_pcd,d1,d2,minn,long)
# iltered_pcd = cylindrical_filter(filtered_pcd, radius=radius,  long=long,die=2, min_neighbors=2, max_var=3)
# filtered_pcd = mid_filter(filtered_pcd,d1,d2,minn,long)
#去噪
quzao_pcd ,ind=filtered_pcd.remove_radius_outlier(2,radius)
#quzao_pcd=filtered_pcd
# o3d.visualization.draw_geometries([pcd])
# o3d.visualization.draw_geometries([quzao_pcd])
#因子标注
yin=yin_filter(quzao_pcd,radius=radius, die=2,long=long)
quzao_point=np.asarray(quzao_pcd.points)

datap=np.append(quzao_point,yin.reshape(-1,1),axis=1)
print(datap.shape)
#倒序排列
datap = datap[np.argsort(datap[:, 3])][::-1]
# per=np.percentile(datap[:,3],30)
# median_index = np.where(datap[:,3] == per)[0][0]
# datap=datap[0:median_index,:]

arrtolas(datap,path2)
