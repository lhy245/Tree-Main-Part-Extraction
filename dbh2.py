import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import laspy
import math

def hua(points, point_on_line, direction_vector):


    min_z = np.min(points[:, 2])
    max_z = np.max(points[:, 2])
    dz = max_z - min_z
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])
    dy = max_y - min_y
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    dx = max_x - min_x
    points[:,0]=points[:,0]-round(min_x)
    points[:,1]=points[:,1]-round(min_y)
    points[:,2]=points[:,2]-round(min_z)

    point_on_line[0] = point_on_line[ 0] - round(min_x)
    point_on_line[1] = point_on_line[1] - round(min_y)
    point_on_line[2] = point_on_line[2] - round(min_z)
    # 可视化结果
    line_length = 10
    line_points = np.array([point_on_line + direction_vector * t for t in np.linspace(-line_length, line_length, 100)])
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label='Point Cloud',s=0.1)
    ax.plot(line_points[:, 0], line_points[:, 1], line_points[:, 2], c='r', linewidth=2, label='Fitted Line')


    # 设置显示范围
    ax.set_xlim(min(points[:, 0]) - 1, min(points[:, 0]) + dx)  # X轴范围
    ax.set_ylim(min(points[:, 1]) - 1, min(points[:, 1]) + dy)  # Y轴范围
    ax.set_zlim(min(points[:, 2]) - 1, min(points[:, 2]) + dz)  # Z轴范围

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('3D Line Fitting in Point Cloud')
    ax.legend()
    plt.show()


def openlas(path):
    las = laspy.read(path)
    scale = las.header.scales
    offset = las.header.offsets
    # srs=las.header.srs
    # 方法2
    coords = np.vstack((las.x, las.y, las.z)).transpose()
    return coords, scale, offset
    del coords
#进行直线的参数方程拟合
def adjust_direction_to_angle(v,xian):
    # 垂直方向单位向量
    k = np.array([0, 0, 1])
    xian = math.radians(xian)
    # 计算夹角
    v_norm = np.linalg.norm(v)
    cos_theta = v[2] / v_norm
    if cos_theta<0:
        v=-v
        v_norm = np.linalg.norm(v)
        cos_theta = v[2] / v_norm
    theta = np.arccos(cos_theta)


    # 如果夹角小于等于30度，直接返回
    if theta <= xian or theta>=360-xian:
        return v
    else:
        # 计算旋转轴
        r = np.cross(v, k)
        r_norm = np.linalg.norm(r)
        if r_norm == 0:  # 如果方向向量与垂直方向平行，则无需旋转
            return v
        else:
            r = r / r_norm  # 归一化旋转轴

            # 计算旋转角度
            delta_theta = theta - xian

            # 使用 Rodrigues 公式计算旋转后的向量
            v_rotated = (
                    v * np.cos(delta_theta) +
                    np.cross(r, v) * np.sin(delta_theta) +
                    r * np.dot(r, v) * (1 - np.cos(delta_theta))
            )

            return v_rotated
def ger_dire(points,du):
# 生成随机的3D点云

    # 计算点云的均值
    mean = np.mean(points, axis=0)
    # 均心化点云
    centered_points = points - mean
    # 计算协方差矩阵
    cov_matrix = np.cov(centered_points, rowvar=False)
    # 特征值分解
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
    # 最大特征值对应的特征向量（直线的方向向量）
    direction_vector0 = eigenvectors[:, np.argmax(eigenvalues)]

    direction_vector=adjust_direction_to_angle(direction_vector0,du)
    # 拟合直线的参数方程
    point_on_line = mean
    return direction_vector,point_on_line#方向向量



def distance_point_zhu(point_cloud, P, direction):
    P0 = P  # 直线上的一点
    d = direction  # 直线的方向向量
    # 计算方向向量的模长
    d_norm = np.linalg.norm(d)
    # 计算每个点到直线的距离
    distances = np.linalg.norm(np.cross(d, point_cloud - P0), axis=1) / d_norm
    return distances
#计算圆柱的中心点
def find_R_center(point_cloud, P, direction):
    tall=point_cloud[:,2].max()-point_cloud[:,2].min()
    distance1=distance_point_zhu(point_cloud, P, direction)
    mean_dis=np.mean(distance1)
    max_dis=np.max(distance1)
    if max_dis<(2*mean_dis) :
        point1=point_cloud[np.argmax(distance1)]#距离最远的点
        distance2=distance_point_zhu(point_cloud, point1, direction)
        point2=point_cloud[np.argmax(distance2)]#距离最远的点
        point=(point2+point1+P)/3#最终的中心点
        distance3=distance_point_zhu(point_cloud, point, direction)
        R=np.max(distance3)
    else:
        point1 = point_cloud[np.argmax(distance1)]  # 距离最远的点
        distance2 = distance_point_zhu(point_cloud, point1, direction)
        point2 = point_cloud[np.argmax(distance2)]  # 距离最远的点
        point=P
        R=mean_dis*2
    return point,R

#计算圆柱范围内的点
def yuanzhu(point_cloud, P, direction,R):
    P0 =P# 直线上的一点
    d = direction # 直线的方向向量

    # 计算方向向量的模长
    d_norm = np.linalg.norm(d)

    distances = np.linalg.norm(np.cross(d, point_cloud - P0), axis=1) / d_norm

    # 筛选在圆柱范围内的点
    points_in_cylinder = point_cloud[distances <= R]

    # 输出在圆柱范围内的点
    return points_in_cylinder

#进行圆柱拟合
# def zhu_filter(points,xi):
#     direction1, p1= ger_dire(points)#获取方向
#     point,R=find_R_center(points,p1,direction1)
#
#     points_new = yuanzhu(points, point, direction1, R)
#     direction_new, p_new = ger_dire(points_new)  # 获取方向
#     point_new, R_new = find_R_center(points_new, p_new, direction_new)
#     while (abs(R_new - R))>R*xi:
#         R=R_new
#         points_new=yuanzhu(points_new,point_new,direction_new,R_new)
#         direction_new, p_new = ger_dire(points_new)  # 获取方向
#         point_new, R_new = find_R_center(points_new, p_new, direction_new)
#     return points_new,R_new,direction_new,point_new
def zhu_filter(points,xi,du):
    direction1, p1= ger_dire(points,du)#获取方向
    point,R=find_R_center(points,p1,direction1)

    points_new = yuanzhu(points, point, direction1, R)
    if points_new.shape[0] < 3:
        return points_new,R,direction1,point
    else:
        direction_new, p_new = ger_dire(points_new,du)  # 获取方向
        point_new, R_new = find_R_center(points_new, p_new, direction_new)
        while (abs(R_new - R))>R*xi:
            R=R_new
            points_new=yuanzhu(points_new,point_new,direction_new,R_new)
            if points_new.shape[0]<3:
                continue
            direction_new, p_new = ger_dire(points_new,du)  # 获取方向
            point_new, R_new = find_R_center(points_new, p_new, direction_new)
        return points_new,R_new,direction_new,point_new
# points,_,_=openlas('data/t.las')
# points,R,direction,point=zhu_filter(points,0.01,5)
# point_on_line=point
# direction_vector=direction
#
# hua(points, point_on_line, direction_vector)


