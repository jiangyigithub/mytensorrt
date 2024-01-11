import math

import cv2
import numpy as np


def project_to_image(pts_3d, P):
    """Project 3d points to image plane.

    Usage: pts_2d = projectToImage(pts_3d, P)
      input: pts_3d: nx3 matrix
             P:      3x4 projection matrix
      output: pts_2d: nx2 matrix

      P(3x4) dot pts_3d_extended(4xn) = projected_pts_2d(3xn)
      => normalize projected_pts_2d(2xn)

      <=> pts_3d_extended(nx4) dot P'(4x3) = projected_pts_2d(nx3)
        => normalize projected_pts_2d(nx2)
    """
    # pts_3d: (8, 3). P: (3, 4)
    n = pts_3d.shape[0]
    pts_3d_extend = np.hstack((pts_3d, np.ones((n, 1))))  # (8, 4)

    pts_2d = np.dot(pts_3d_extend,
                    np.transpose(P))  # (8, 4) /dot (4, 3) -> (8, 3)
    # normalize image coordinations to pixel coordination.
    pts_2d[:, 0] /= pts_2d[:, 2]
    pts_2d[:, 1] /= pts_2d[:, 2]
    return pts_2d[:, 0:2]


def calc_rotp(norm_vec):
    """Given normal vector, calculate rotation matrix."""
    a, b, c = float(norm_vec[0]), float(norm_vec[1]), float(norm_vec[2])
    r12, r22, r32 = a, b, c
    r11, r21, r31 = 1, -a / b, 0
    div = a**2 + b**2
    r13, r23, r33 = (-a * c) / div, (-b * c) / div, 1
    R = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]],
                 dtype=np.float64)
    return R / np.linalg.norm(R, axis=0)


# def roty(t):
#     """Rotation about the y-axis."""
#     c = np.cos(t)
#     s = np.sin(t)
#     return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])   # there may be a mistake   https://blog.csdn.net/Aidam_Bo/article/details/84071593?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-84071593-blog-107921623.235^v38^pc_relevant_sort_base3&spm=1001.2101.3001.4242.1&utm_relevant_index=1   
                                                                                    # https://blog.csdn.net/Aidam_Bo/article/details/84071593?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-84071593-blog-107921623.235^v38^pc_relevant_sort_base3&spm=1001.2101.3001.4242.1&utm_relevant_index=1 https://blog.csdn.net/weixin_44369981/article/details/109539155

# def roty(t):
#     """Rotation about the y-axis."""
#     c = np.cos(t)
#     s = np.sin(t)
#     return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])

def roty(t):
    """Rotation about the y-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def compute_box_3d(obj, P, gplane):
    """Takes an object and a projection matrix (P) and projects the 3d bounding
    box (camera coordinate) into the image plane.

    Returns:
        corners_2d: (8,2) array in image coord.
        corners_3d: (8,3) array in camera coord.
    """
    # compute rotational matrix around y axis
    # FIXME: Here the label exported ry is z axis w.r.t x axis (camera coord),
    # FIXME: will fix the labels in next dataset versions
    # R_y = roty(obj.ry -math.pi / 2)  # mistake
    R_y = roty(obj.ry)

    # 3d bounding box dimensions and corners
    l, w, h = obj.length, obj.width, obj.height
    z_corners = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]
    y_corners = [h / 2, h / 2, h / 2, h / 2, -h / 2, -h / 2, -h / 2, -h / 2]
    x_corners = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]

    # rotate and translate 3d bounding box, (3,3) \dot (3,8) -> (3, 8)
    corners_3d = np.dot(R_y, np.vstack([x_corners, y_corners, z_corners]))
    R_p = calc_rotp(-gplane[:3])
    corners_3d = np.dot(R_p, corners_3d)

    # add shift for each axis with locations in camera coordination.
    corners_3d[0, :] = corners_3d[0, :] + obj.x
    corners_3d[1, :] = corners_3d[1, :] + obj.y
    corners_3d[2, :] = corners_3d[2, :] + obj.z

    # project to 2d
    corners_2d = project_to_image(np.transpose(corners_3d), P)
    return corners_2d, np.transpose(corners_3d)


def draw_projected_box3d(image, qs, color=(0, 255, 0), thickness=2):
    """ Draw 3d bounding box in image
        qs: (8,3) array of vertices for the 3d box in following order:
            1 -------- 0
           /|         /|
          2 -------- 3 .
          | |        | |
          . 5 -------- 4
          |/         |/
          6 -------- 7
    """
    qs = qs.astype(np.int32)
    for k in range(0, 4):
        i, j = k, (k + 1) % 4
        # use LINE_AA for opencv3
        cv2.line(image, (qs[i, 0], qs[i, 1]), (qs[j, 0], qs[j, 1]), color,
                 thickness)
        i, j = k + 4, (k + 1) % 4 + 4
        cv2.line(image, (qs[i, 0], qs[i, 1]), (qs[j, 0], qs[j, 1]), color,
                 thickness)

        i, j = k, k + 4
        cv2.line(image, (qs[i, 0], qs[i, 1]), (qs[j, 0], qs[j, 1]), color,
                 thickness)

    return image
