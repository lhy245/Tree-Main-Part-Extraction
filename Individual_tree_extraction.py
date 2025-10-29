import laspy
import numpy as np
import open3d as o3d
from tqdm import tqdm


# 计算两点之间的距离
def two_distance(a, b):
    return (np.linalg.norm(a - b))


# 计算点到所有点距离
def min_distance(a, b):
    a = np.array(a)

    b = np.array(b)

    if a.shape == b.shape:
        a = a[0:2]
        b = b[0:2]
        return (np.linalg.norm(a - b))
    else:
        a = a[:, 0:2]
        b = b[0:2]
        return (np.linalg.norm(a - b, axis=1))


def min_3ddistance(a, b):
    a = np.array(a)

    b = np.array(b)

    if a.shape == b.shape:
        a = a[0:3]
        b = b[0:3]
        return (np.linalg.norm(a - b))
    else:
        a = a[:, 0:3]
        b = b[0:3]
        a_b=a - b
        a_b[:,2]=(0.01**(1/2))*a_b[:,2]
        return (np.linalg.norm(a_b, axis=1))


# 计算聚类中心
def center(a):
    a = np.array(a)
    if a.ndim == 1:
        print('y')
        return a
    else:
        return np.mean(a, axis=0)

import ODR
def sheng_zhang(data_sett,yu1,yu2):
    datatree = []
    data_sheng0=data_sett


    while data_sheng0.size!=0:#迭代，直到没有剩余的点
        #第一棵树
        if data_sheng0.ndim==1:
            continue
        tree=data_sheng0[0].reshape(-1,3)#种子点集合
        np.delete(data_sheng0,0)
        #计算周围的点，减少计算时间
        diatance0=min_distance(data_sheng0,tree[0])
        pt=np.where(diatance0<yu1)
        data_sheng1=data_sheng0[pt]#初始种子点周围的点
        data_sheng0=np.delete(data_sheng0,pt,axis=0)

        print(diatance0.shape[0])
        seed_point=tree[0]#种子点
        new_seed_points=seed_point.reshape(-1,3)#种子点集合
        siz1 = data_sheng1.size-1
        while data_sheng1.size!=siz1:
            siz1 = data_sheng1.size
            if tree.shape[0] > 3:
                direction, point_on_line = ODR.ger_dire(tree, 30)
                distance0 = ODR.distance_point_zhu(tree, point_on_line, direction)
                max_dis = np.max(distance0)
                point1 = tree[np.argmax(distance0)]  # 距离最远的点
                distance2 = ODR.distance_point_zhu(tree, point1, direction)
                point2 = tree[np.argmax(distance2)]  # 距离最远的点
                point_on_line = (point2 + point1 + point_on_line) / 3  # 最终的中心点
                distance3 = ODR.distance_point_zhu(tree, point_on_line, direction)
                R = np.max(distance3)
                if tree[:, 2].max() - tree[:, 2].min() < 50*yu2:  # 高度小于1
                    R_ = R
                    R = R + yu2 * 2
                else:
                    try:
                        if R<1.5*R_:
                            R = R + yu2
                    except:
                        R=R

            else:
                point_on_line=np.mean(tree,axis=0)
                direction=np.array([0,0,1],dtype=np.float64)
                R=yu2*2
            seed_points=new_seed_points
            new_seed_points=np.empty([0,3])
            for seed_point in seed_points:
                #计算种子点周围的点
                diatance1 = min_3ddistance(data_sheng1, seed_point)
                pt0=np.where(diatance1<yu2)[0]
                new_points = data_sheng1[pt0] # 种子点周围的点
                if new_points.shape[0] == 0:  # 周围没有点
                    continue
                # if new_points.ndim == 1:  # 周围只有一个点
                #     continue
                #new_seed_points = [np.array(row) for row in new_seed_points]
                #计算直线周围的点
                diatance2=ODR.distance_point_zhu(new_points,point_on_line,direction)
                pt1=np.where(diatance2<R)[0]
                new_points=new_points[pt1]
                # #限制，只能向上
                # pt1_2=np.where(new_points[:,2]>=max(tree[:,2]-5))
                # new_points = new_points[pt1_2]
                pt2=pt0[pt1]
                if new_points.shape[0] == 0:  # 周围没有点
                    continue

                new_seed_points=np.append(new_seed_points,new_points,axis=0)
                data_sheng1=np.delete(data_sheng1,pt2,axis=0)
            tree=np.append(tree,new_seed_points,axis=0)

        datatree.append(tree)
        data_sheng0=np.append(data_sheng0,data_sheng1.reshape(-1,3),axis=0)#树形成之后剩余点
        print(data_sheng0.shape[0])
    return datatree
def lvzhu(datatree):
    datatree_new = []
    center_points = []
    Rs = []
    directions = []
    print('柱状过滤')
    for i in tqdm(range(0, len(datatree))):
        tree = datatree[i][:, 0:3]
        tree_new, R, direction, center_point = ODR.zhu_filter(tree, 0.01,30)
        center_points.append(center_point)
        Rs.append(R)
        directions.append(direction)
        datatree_new.append(tree_new)
    return datatree_new
def openlas(path):
    las = laspy.read(path)
    print(las.header.point_count)
    scale = las.header.scales
    offset = las.header.offsets
    # 方法2
    coords = np.vstack((las.x, las.y, las.z,las.Constant)).transpose()
    return coords, scale, offset
    del coords
    # 打印两种方法的点
####################路径###########################
path1 = input("Input file path: ")
path2 = input("Output file path: ")
datasett, scale, offset = openlas(path1)#MediumPlot1

datasett = datasett[np.argsort(datasett[:, 2])]
datasett = datasett[np.argsort(datasett[:, 3])][::-1]

pt=datasett.shape[0]
dataset =datasett[:,0:3]



tall=2
min_td=np.percentile(dataset[:,2],40)
max_td=np.percentile(dataset[:,2],80)
def shan(data_s):
    num_bins = 5
    observed_freq, bin_edges = np.histogram(data_s[:, 2], bins=num_bins)

    w0 = observed_freq[observed_freq < 1]

    if w0.size>0:
        ptt=np.where(observed_freq<1)[0][0]
        zhi_s=bin_edges[ptt]
        return data_s[data_s[:,2]<zhi_s]
    else:
        return data_s

def shang(data):
    # 生成示例数据
    if data.shape[0] < 5:  # 空元的数量
        num = 0
    # elif max(data[:,1]) < meann_suan :#排除树干概率较小
    #     num = 0
    elif np.min(data) >min_td:#排除树底高的树
        num = 0
    elif np.max(data)-np.min(data) <tall:
        num=0

    else:
        num=1

    return num

datatree=sheng_zhang(dataset,4,0.2)
datatree=lvzhu(datatree)

'''删除错误树'''
datatree2=[]

datatall=[]

for i in range(len(datatree)):
    data_s = np.array(datatree[i])
    data_s=shan(data_s)
    if data_s.size!=0:

        datatree2.append(data_s.tolist())

        datatall.append(np.max(data_s[:,2])-np.min(data_s[:,2]))
del datatree
datatree=datatree2



x = []
y = []
z = []
clas=0
classes = []
print('yuan',len(datatree))
for i in range(len(datatree)):
    data = np.array(datatree[i])
    num=shang(data[:,2])
    if len(x) != 0:
        if num==1:#排除较低的树
            x = np.append(x, data[:, 0])
            y = np.append(y, data[:, 1])
            z = np.append(z, data[:, 2])
            classes = np.append(classes, np.full(len(datatree[i]), clas))
            clas = clas + 1
    else:
        if num==1:
            x = data[:, 0]
            y = data[:, 1]
            z = data[:, 2]
            classes = np.full(len(datatree[i]), clas)
            clas = clas + 1

# 创建 LAS 文件
print(clas)

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
outFile.trees = classes.astype(np.int32)



outFile.write(path2)

