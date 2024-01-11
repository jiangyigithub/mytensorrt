# python 3
import numpy as np
import math
import os
import sys
sys.path.append('/home/icv/workspaces/inf_onsite/camera/monocular_3d/deepstream/deepstream-test3/lib/helpers')

# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import operator
from depth_geometry import kdtree

# from scipy import interpolate
# import scipy.interpolate

import time



enabled_interpolation = False

class Item(object):
    def __init__(self, coord , data):
        self.coords = coord
        self.data = data

    def __len__(self):
        return len(self.coords)

    def __getitem__(self, i):
        return self.coords[i]

    def __repr__(self):
        return 'Item({}, {})'.format(self.coords, self.data)


def timestamp(name):
    name_1 = time.time() 
    print( name+":"+str(name_1))

def projection_init(camera_pose):
    
    ###################################################
    ###### To get Zc with intrinsics, extrinsics ######
    ###################################################
    
    #extrinsics
    theta_x = camera_pose[0][0]
    theta_y = camera_pose[0][1]
    theta_z = camera_pose[0][2] 
   
    Tx = camera_pose[1][0] 
    Ty = camera_pose[1][1]
    Tz = camera_pose[1][2] 
    

    global points_set
    global z_set
    global RT_augmented
    global K
    global RT_augmented_inv
    x_set = []
    y_set = []
    z_set = []
   
    points_set = []

    ###################################################
    ###### To get Zc with intrinsics, extrinsics ######
    ###################################################
    # K = np.array([[997.30810007, 0.000000  , 979.18207798],  # houshan south lens old
    #               [0.000000  , 997.04838962, 517.0099155],
    #               [0.000000  , 0.000000  , 1.000000  ]])
    # K = np.array([[1367.8905, 0.000000  , 1143.7368],
    #               [0.000000  , 1373.59794, 722.559216],
    #               [0.000000  , 0.000000  , 1.000000  ]])  # houshan
    
    K = np.array([[1451.25488, 0.000000  , 1120.50216],
                  [0.000000  , 1452.82479, 691.890138],
                  [0.000000  , 0.000000  , 1.000000  ]])  # hongye west
    
    # K = np.array([[1439.96128, 0.000000  , 1120.00946],
    #               [0.000000  , 1442.95208, 690.35747],
    #               [0.000000  , 0.000000  , 1.000000  ]])  #hongye south
    
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(theta_x), -1 * math.sin(theta_x)],
        [0,       math.sin(theta_x), math.cos(theta_x)]
    ])
    Ry = np.array([
        [math.cos(theta_y), 0, math.sin(theta_y)],
        [0, 1, 0],
        [-1*math.sin(theta_y), 0, math.cos(theta_y)]
    ])
    Rz = np.array([
        [math.cos(theta_z), -1 * math.sin(theta_z), 0],
        [math.sin(theta_z), math.cos(theta_z), 0],
        [0, 0, 1]
    ])
    R = Rz @ Ry @ Rx
    

    # print("R:")
    # print(R)

    T = np.array([Tx, Ty, Tz])

    # print("RT:")
    # print(RT)

    RT_augmented = np.eye(4)
    RT_augmented[:3, :3] = R
    RT_augmented[:3, 3] = T

    # print("RT_augmented:")
    # print(RT_augmented)

    RT_augmented_inv = np.linalg.inv(RT_augmented)

    # print("RT_augmented_inv:")
    # print(RT_augmented_inv)


    ################################
    ###### ROI ZC GENERATION  ######
    ################################

    count = 0
    # for Test_x in range(-200, 200, 1):
    #     for Test_y in range(-200, 200, 1): #wuxi
    for Test_x in range(-20, 140, 1):
        for Test_y in range(-20, 140, 1):
    # for Test_x in range(-200001, 200001, 1000):
    #     for Test_y in range(-200001, 200001, 1000):
            # print(Test_y)
            # print(count)

            count +=1

            world = np.array([
                [float(Test_x)],  # /3
                [float(Test_y)],
                [0],
                [1]
            ])

            wtc = RT_augmented_inv @ world

            dm_reduction = np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0]
            ])

            new_reference = dm_reduction @ wtc

            param = K @ new_reference
            # print("param:")
            # print(param)

            uv = param/param[2]
            # print("uv:")
            # print(uv)

            u = uv[0]
            v = uv[1]
            # print(u,v)
            Zc = param[2]
            # print("Zc:")
            # print(Zc)

            x_set = np.append(x_set, u)
            y_set = np.append(y_set, v)

            # points_set.append([np.float(u), np.float(v)])
            points_set.append([float(u), float(v)])
            z_set.append(Zc)
            
    #####################################################################
    ###### KDTree to find nearest k points and choose the right Zc ######
    #####################################################################

    
    global tree
    coord_ap_zc = []
    for i in range(0,count):
        coord_ap_zc.append(Item(points_set[i],z_set[i]))
    # print(coord_ap_zc)
    tree = kdtree.create(coord_ap_zc)
    return True        

def get_Zc( u , v):
    #API: Return Zc  Searching with  U,V

    if(enabled_interpolation):

        points = tree.search_knn([u, v], 4)
        # print(points)

        # point_cor = []
        distance = []
        # label_matrix = []
        Zc_corresponding = []

        for i in range(0, 4):
            # point_cor.append(point[i][0].data)
            distance.append(points[i][1])
            # label = np.where(np.array(points_set) == np.array(point_cor[i]))
            # label_matrix.append(label[0][0])
            Zc_corresponding.append(np.float(points[i][0].data.data))

        # print(point_cor)
        # print(distance)
        # print(Zc_corresponding)

        #######################################################
        ###### interpolate ######
        #######################################################
        molecule_sum = 0
        denominator_sum = 0

        for j in range(0,4):

            molecule = Zc_corresponding[j]/distance[j]

            molecule_sum = molecule_sum + molecule

            denominator = 1/distance[j]
            denominator_sum = denominator_sum + denominator
        
        # print(molecule_sum)
        # print(denominator_sum)

        IDW_average = molecule_sum/denominator_sum
        # print(IDW_average)
        return IDW_average

    else:
        point = tree.search_nn([u,v])
        Zc_corresponding = point[0].data.data
        return Zc_corresponding

def find_point(u,v,h):
    temp = get_Zc(u,v)*((6.6571-h)/6.6571)
    # print(temp)
    return temp




def roty(t):
    """ Rotation about the y-axis. """
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def calc_rotp(norm_vec):
    """Given normal vector, calculate rotation matrix
    """
    a, b, c = float(norm_vec[0]), float(norm_vec[1]), float(norm_vec[2])
    r12, r22, r32 = a, b, c
    r11, r21, r31 = 1, - a / b, 0
    div = a ** 2 + b ** 2
    r13, r23, r33 = (- a * c) / div, (-b * c ) / div, 1
    R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]], dtype=np.float64)
    return R / np.linalg.norm(R, axis=0)

def project_to_image(pts_3d, extri_parameters):
    """ Project 3d points to image plane.

    Usage: pts_2d = projectToImage(pts_3d, extri_parameters)
      input: pts_3d: nx3 matrix
             extri_parameters:      3x4 projection matrix
      output: pts_2d: nx2 matrix

      extri_parameters(3x4) dot pts_3d_extended(4xn) = projected_pts_2d(3xn)
      => normalize projected_pts_2d(2xn)

      <=> pts_3d_extended(nx4) dot extri_parameters'(4x3) = projected_pts_2d(nx3)
        => normalize projected_pts_2d(nx2)
    """
    # pts_3d: (1, 3). extri_parameters: (3, 4)
    # print(pts_3d)
    n = pts_3d.shape[0]
    pts_3d_extend = np.hstack((pts_3d, np.ones((n, 1)))) # (1, 4)

    pts_2d = np.dot(pts_3d_extend, np.transpose(extri_parameters))  # (1, 4) /dot (4, 3) -> (1, 3)
    # normalize image coordinations to pixel coordination.
    pts_2d[:, 0] /= pts_2d[:, 2] 
    pts_2d[:, 1] /= pts_2d[:, 2]
    return pts_2d[:, 0:2]

def obj_enutocam(enu):
    """Convert lidar box3d to cam box3d.
    Args:
        obj (Object3D): lidar box3d (x, y, z, h, w, l, r)
        extrinsic: R, T 
    """
    translation = enu.T
    # print(translation)
    
    # h, w, l = obj.h, obj.w, obj.l
    # x_corners = [l / 2,  l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
    # y_corners = [w / 2, -w / 2, -w / 2,  w / 2, w / 2, -w / 2, -w / 2, w / 2]
    # z_corners = [h / 2,  h / 2,  h / 2,  h / 2, -h/ 2, -h/ 2, -h / 2, -h / 2]
        
    # corners = np.vstack([x_corners, y_corners, z_corners])
    # R_z = rotz(obj.ry)
    # corners_lidar = R_z @ corners + translation # 3x8

    R = RT_augmented_inv[:3, :3]
    T = RT_augmented_inv[:3, 3]
    obj_cam = R @ translation + T # transform to camera coordinate
    # print(R)

    # x_cam = np.mean(corners_cam[0])
    # y_cam = np.mean(corners_cam[1])
    # z_cam = np.mean(corners_cam[2])

    # corners_cam = corners_cam.T
    # ry = math.atan2(corners_cam[0][0] - corners_cam[3][0], 
    #                 corners_cam[0][2] - corners_cam[3][2]) # atan2(z, x) #need to be confirmed

    # obj.x, obj.y, obj.z = x_cam, y_cam, z_cam
    # print(T)
    # obj.ry = ry
    return obj_cam

def obj_3d_center_bottom_pt(vir_object, extrinsic, gplane):
    """ Takes an object and a projection matrix and projects the 3d
        vir_object includes x y h ry z (z,h,ry is the 3d center of object infernecr result)
        3d center (camera coordinate) into the image plane.
        Returns:
            certer_3d: (1,3) array in camera coord.
    """
    # compute rotational matrix around y axis
    R_y = roty(vir_object[3])
    h = vir_object[2]
    enu_ = np.array([vir_object[0],vir_object[1],h/2])
    cam_xyz = obj_enutocam(enu_)
    # center = np.transpose([0, -h/2 , 0])

    # rotate and translate 3d bounding box, (3,3) \dot (3,8) -> (3, 8)
    certer_3d = np.dot(R_y, np.vstack([[0], [0] , [0]]))
    # print(gplane)
    R_p = calc_rotp(-gplane[:3])
    certer_3d = np.dot(R_p, certer_3d)

    # add shift for each axis with locations in camera coordination.
    # print(cam_xyz)
    certer_3d[0] = certer_3d[0] + cam_xyz[0]
    certer_3d[1] = certer_3d[1] + cam_xyz[1]
    certer_3d[2] = certer_3d[2] + cam_xyz[2]
    # print(certer_3d,'      +++++==    ',h)
    # certer_3d = np.array([[-1.93], [2.15], [25.98]])
    # project to 2d
    # print(certer_3d)
    temp_3d = np.transpose(certer_3d)
    # print(temp_3d)
    center_2d = project_to_image(temp_3d, extrinsic)
    # print(extrinsic,'-------------------------extrinsic')
    # print(gplane,'+++++++++++++++++++++++++')
    # print(certer_3d)
    # print(center_2d,'+++++++++++++++++++++++++++++++++++++++++')
    return center_2d, temp_3d

def search_obj_bottom_point(object_inf_result):
    
    for x in range (0, 2000, 1):
        for y in range (0, 2000, 1):
            e = float(x)/10
            n = float(y)/10
            # print(e, n)
            obj_info = [e ,n, object_inf_result[0], object_inf_result[1]]
            # obj_info = [-5.78, 68.46, -1.35, object_inf_result[1]]
            temp_p = np.array([[2.75783972e+03, 0.00000000e+00, 9.92437197e+02, 0.00000000e+00],[0.00000000e+00, 2.89907393e+03, 5.76566446e+02, 0.00000000e+00],[0.00000000e+00, 0, 1, 0.00000000e+00] ])
            temp, temp_center = obj_3d_center_bottom_pt(obj_info, temp_p, np.array([0.0002473687, -0.983721, -0.179703, 7.52674293518]))
            dis_sqr = math.pow((temp[:,0] - object_inf_result[2]),2) + math.pow((temp[:,1] - object_inf_result[3]),2)
            # print(dis_sqr, '    ++++++++++++++++++++++++++++  ')
            # return temp
            if dis_sqr < 10:
                print(dis_sqr, '    ++++++++++++++++++++++++++++ ============================ ',temp, '    ',float(x)/3,'     ',float(y)/3,'     ',temp_center)
                return temp



   
def reverse_porjection(u,v,Zc ):

    #############################################
    ###### reverse projection to get world coordinate ######
    #############################################
    ZC_interpolation = Zc
    UV = np.array([
        [u],
        [v],
        [1]
    ])

    camera_cor = ZC_interpolation*UV

    world_cor = np.linalg.inv(K) @ camera_cor

    world_cor = np.append(world_cor, [[1]],axis= 0 )

    coordinate_vector = RT_augmented @ world_cor

    coordinate = coordinate_vector[0:3]

    # print("coordinate:")

    # print(coordinate)

    return coordinate



