"""some auxiliary functions for KITTI dataset."""
import cv2
import numpy as np


# region  ==Object3D==
def get_objects_from_label(label_file):
    with open(label_file, 'r') as f:
        lines = f.readlines()
    # TODO(xiaofengzhang): change this to use ground plane information
    # file_id = label_file.split('/')
    # print(gp_file)
    # with open( gp_file ) as f:
    #     ground_plane = f.readline()
    # print(ground_plane)

    objects = [Object3d(line) for line in lines]

    return objects


class Object3d(object):
    def __init__(self, line: str, plane: str = '0 0 0 0'):
        """
        Args:
            line (str): a line of data, split by ' '
        cols    cls T O x0 y0 x1 y1 h w l x  y  z  yaw
        index   0   1 2 3  4  5  6  7 8 9 10 11 12 13
        """
        label = line.strip().split(' ')
        self.src = line
        self.cls_type = label[0]
        self.trucation = float(label[1])
        self.occlusion = float(
            label[2]
        )  # 0:fully visible 1:partly occluded 2:largely occluded 3:unknown

        self.box2d = np.array((float(label[3]), float(label[4]), float(
            label[5]), float(label[6])),
                              dtype=np.float32)
        self._h = float(label[7])
        self._w = float(label[8])
        self._l = float(label[9])
        self.pos = np.array(
            (float(label[10]), float(label[11]), float(label[12])),
            dtype=np.float32)
        self.dis_to_cam = np.linalg.norm(self.pos)
        self.ry = float(label[13])
        self.score = -1.0
        self.level_str = None
        self.level = self.get_obj_level()
        self.alpha = self.compute_alpha()

        ground_plane = plane.strip().split(' ')
        self.A = float(ground_plane[0])
        self.B = float(ground_plane[1])
        self.C = float(ground_plane[2])
        self.D = float(ground_plane[3])
        self.norm = np.sqrt(self.A**2 + self.B**2 + self.C**2)

    def compute_alpha(self):

        a = self.ry
        x = self.pos[0]
        z = self.pos[2]
        a += np.arctan2(z, x) + 1.5 * np.pi
        a = a % (2 * np.pi)
        if a > np.pi:
            a -= 2 * np.pi
        return a

    def get_obj_level(self):
        height = float(self.box2d[3]) - float(self.box2d[1]) + 1

        if self.trucation == -1:
            self.level_str = 'DontCare'
            return 0

        if height >= 40 and self.trucation == 0 and self.occlusion <= 0:
            self.level_str = 'Easy'
            return 1  # Easy
        elif height >= 25 and self.trucation <= 0.5 and self.occlusion <= 1:
            self.level_str = 'Moderate'
            return 2  # Moderate
        elif height >= 25 and self.trucation <= 0.8 and self.occlusion <= 2:
            self.level_str = 'Hard'
            return 3  # Hard
        else:
            self.level_str = 'UnKnown'
            return 4

    def generate_corners3d(self):
        """generate corners3d representation for this object.

        :return corners_3d: (8, 3) corners of box3d in camera coord
        """
        _h, _w, _l = self._h, self._w, self._l
        # l = l*self.A/self.norm
        # h = h*self.B/self.norm
        # w = w*self.C/self.norm
        x_corners = [
            _l / 2, _l / 2, -_l / 2, -_l / 2, _l / 2, _l / 2, -_l / 2, -_l / 2
        ]
        y_corners = [
            _h / 2, _h / 2, _h / 2, _h / 2, -_h / 2, -_h / 2, -_h / 2, -_h / 2
        ]
        z_corners = [
            _w / 2, -_w / 2, -_w / 2, _w / 2, _w / 2, -_w / 2, -_w / 2, _w / 2
        ]

        R = np.array([[np.cos(self.ry), 0, np.sin(self.ry)], [0, 1, 0],
                      [-np.sin(self.ry), 0,
                       np.cos(self.ry)]])
        corners3d = np.vstack([x_corners, y_corners, z_corners])  # (3, 8)
        corners3d = np.dot(R, corners3d).T
        corners3d = corners3d + self.pos
        return corners3d

    def to_bev_box2d(self, oblique=True, voxel_size=0.1):
        """
        :param bev_shape: (2) for bev shape (h, w), => (y_max, x_max) in image
        :param voxel_size: float, 0.1m
        :param oblique:
        :return: box2d (4, 2)/ (4) in image coordinate
        """
        if oblique:
            corners3d = self.generate_corners3d()
            xz_corners = corners3d[0:4, [0, 2]]
            box2d = np.zeros((4, 2), dtype=np.int32)
            box2d[:, 0] = ((xz_corners[:, 0] - Object3d.MIN_XZ[0]) /
                           voxel_size).astype(np.int32)
            box2d[:, 1] = Object3d.BEV_SHAPE[0] - 1 - (
                (xz_corners[:, 1] - Object3d.MIN_XZ[1]) / voxel_size).astype(
                    np.int32)
            box2d[:, 0] = np.clip(box2d[:, 0], 0, Object3d.BEV_SHAPE[1])
            box2d[:, 1] = np.clip(box2d[:, 1], 0, Object3d.BEV_SHAPE[0])
        else:
            box2d = np.zeros(4, dtype=np.int32)
            cu = np.floor((self.pos[0] - Object3d.MIN_XZ[0]) /
                          voxel_size).astype(np.int32)
            cv = Object3d.BEV_SHAPE[0] - 1 - (
                (self.pos[2] - Object3d.MIN_XZ[1]) / voxel_size).astype(
                    np.int32)
            half_l, half_w = int(self.l / voxel_size / 2), int(self.w /
                                                               voxel_size / 2)
            box2d[0], box2d[1] = cu - half_l, cv - half_w
            box2d[2], box2d[3] = cu + half_l, cv + half_w

        return box2d

    def to_str(self):
        print_str = \
            '%s %.3f %.3f %.3f ' % (
                self.cls_type,
                self.trucation,
                self.occlusion,
                self.alpha) + \
            'box2d: %s hwl: [%.3f %.3f %.3f] pos: %s ry: %.3f' % (
                self.box2d,
                self._h, self._w, self._l,
                self.pos,
                self.ry)
        return print_str

    def to_kitti_format(self):
        kitti_str = \
            '%s %.2f %d %.2f ' % (
                self.cls_type,
                self.trucation,
                int(self.occlusion),
                self.alpha,) + \
            '%.2f %.2f %.2f %.2f ' % (
                self.box2d[0], self.box2d[1],
                self.box2d[2], self.box2d[3],) + \
            '%.2f %.2f %.2f %.2f %.2f %.2f %.2f' % (
                self._h, self._w, self._l,
                self.pos[0], self.pos[1], self.pos[2],
                self.ry)
        return kitti_str

    def to_dair_format(self):
        dair_str = \
            '%s %.2f %d %.2f ' % (
                self.cls_type,
                self.trucation,
                int(self.occlusion),
                self.alpha,) + \
            '%.2f %.2f %.2f %.2f ' % (
                self.box2d[0], self.box2d[1],
                self.box2d[2], self.box2d[3],) + \
            '%.2f %.2f %.2f %.2f %.2f %.2f %.2f' % (
                self._h, self._w, self._l,
                self.pos[0], self.pos[1], self.pos[2],
                self.ry)
        return dair_str


# endregion

# region ==calibration==


def get_calib_from_file(calib_file):
    with open(calib_file) as f:
        lines = f.readlines()

    obj = lines[2].strip().split(' ')[1:]
    P2 = np.array(obj, dtype=np.float32)
    obj = lines[3].strip().split(' ')[1:]
    P3 = np.array(obj, dtype=np.float32)
    obj = lines[4].strip().split(' ')[1:]
    R0 = np.array(obj, dtype=np.float32)
    obj = lines[5].strip().split(' ')[1:]
    Tr_velo_to_cam = np.array(obj, dtype=np.float32)

    return {
        'P2': P2.reshape(3, 4),
        'P3': P3.reshape(3, 4),
        'R0': R0.reshape(3, 3),
        'Tr_velo2cam': Tr_velo_to_cam.reshape(3, 4)
    }


def get_dair_calib_from_file(calib_file: str) -> dict:
    """get calib parameters from calib file.

    Args:
        calib_file (str): path to calib file
            root/[split]/calib/xxxxxx.txt

    Returns:
        dict: Dictionary including P2 and R0
        The challenge 5 only have value for P2
        R0 is identity matrix
    """
    with open(calib_file) as f:
        lines = f.readlines()
    # print(lines)
    obj = lines[0].strip().split(' ')[1:]
    P2 = np.array(obj, dtype=np.float32)

    R0 = np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float32)
    # Not used just a place holder
    Tr_velo_to_cam = np.array([
        -3.20550189e-02, -9.97451892e-01, 2.05512490e-02, -2.19044456e+00,
        -2.24093014e-01, 2.98604149e-03, -8.75680012e-01, 5.63608626e+00,
        9.73745544e-01, -4.16783500e-02, -2.02337505e-01, 1.41636648e+00
    ],
                              dtype=np.float32)
    return {
        'P2': P2.reshape(3, 4)[:, :3],
        'R0': R0.reshape(3, 3),
        'Tr_velo2cam': Tr_velo_to_cam.reshape(3, 4)
    }


class DAIRCalibration(object):
    def __init__(self, calib_file):
        if isinstance(calib_file, str):
            calib = get_dair_calib_from_file(calib_file)
        else:
            calib = calib_file

        self.P2 = calib['P2']  # 3 x 4
        self.R0 = calib['R0']  # 3 x 3
        self.V2C = calib['Tr_velo2cam']  # 3 x 4
        self.C2V = self.inverse_rigid_trans(self.V2C)

        # Camera intrinsics and extrinsics
        self.cu = self.P2[0, 2]
        self.cv = self.P2[1, 2]
        self.fu = self.P2[0, 0]
        self.fv = self.P2[1, 1]
        self.tx = 0.0
        self.ty = 0.0

    def cart_to_hom(self, pts):
        """
        :param pts: (N, 3 or 2)
        :return pts_hom: (N, 4 or 3)
        """
        pts_hom = np.hstack((pts, np.ones((pts.shape[0], 1),
                                          dtype=np.float32)))
        return pts_hom

    def lidar_to_rect(self, pts_lidar):
        """
        :param pts_lidar: (N, 3)
        :return pts_rect: (N, 3)
        """
        pts_lidar_hom = self.cart_to_hom(pts_lidar)
        pts_rect = np.dot(pts_lidar_hom, np.dot(self.V2C.T, self.R0.T))
        # pts_rect = reduce(np.dot, (pts_lidar_hom, self.V2C.T, self.R0.T))
        return pts_rect

    def rect_to_lidar(self, pts_rect):
        pts_ref = np.transpose(
            np.dot(np.linalg.inv(self.R0), np.transpose(pts_rect)))
        pts_ref = self.cart_to_hom(pts_ref)  # nx4
        return np.dot(pts_ref, np.transpose(self.C2V))

    def rect_to_img(self, pts_rect):
        """
        :param pts_rect: (N, 3) N points  and vector3: x, y, z
        P2: 3x3 or 3x4 matrix
        {[fx 0 cx]
         [0 fy cy]
         [0 0  1]
        :return pts_img: (N, 2)
        """
        Zcs = pts_rect[:, 2]
        n = len(Zcs)
        uv1 = np.dot(pts_rect, self.P2.T)
        if n == 8:
            for i in range(n):
                uv1[i] /= Zcs[i]
        else:
            uv1[0] /= Zcs
        uv = uv1[:, 0:2]
        return uv

    def lidar_to_img(self, pts_lidar):
        """
        :param pts_lidar: (N, 3)
        :return pts_img: (N, 2)
        """
        pts_rect = self.lidar_to_rect(pts_lidar)
        pts_img, pts_depth = self.rect_to_img(pts_rect)
        return pts_img, pts_depth

    def img_to_rect(self, u, v, depth_rect):
        """
        :param u: (N)
        :param v: (N)
        :param depth_rect: (N)
        :return:
        """
        x = ((u - self.cu) * depth_rect) / self.fu + self.tx
        y = ((v - self.cv) * depth_rect) / self.fv + self.ty
        pts_rect = np.concatenate(
            (x.reshape(-1, 1), y.reshape(-1, 1), depth_rect.reshape(-1, 1)),
            axis=1)
        return pts_rect

    def depthmap_to_rect(self, depth_map):
        """
        :param depth_map: (H, W), depth_map
        :return:
        """
        x_range = np.arange(0, depth_map.shape[1])
        y_range = np.arange(0, depth_map.shape[0])
        x_idxs, y_idxs = np.meshgrid(x_range, y_range)
        x_idxs, y_idxs = x_idxs.reshape(-1), y_idxs.reshape(-1)
        depth = depth_map[y_idxs, x_idxs]
        pts_rect = self.img_to_rect(x_idxs, y_idxs, depth)
        return pts_rect, x_idxs, y_idxs

    def corners3d_to_img_boxes(self, corners3d):
        """
        :param corners3d: (N, 8, 3) corners in rect coordinate
        :return: boxes: (None, 4) [x1, y1, x2, y2] in rgb coordinate
        :return: boxes_corner: (None, 8) [xi, yi] in rgb coordinate
        """
        sample_num = corners3d.shape[0]
        corners3d_hom = np.concatenate((corners3d, np.ones(
            (sample_num, 8, 1))),
                                       axis=2)  # (N, 8, 4)

        img_pts = np.matmul(corners3d_hom, self.P2.T)  # (N, 8, 3)

        x, y = img_pts[:, :, 0] / img_pts[:, :, 2], img_pts[:, :,
                                                            1] / img_pts[:, :,
                                                                         2]
        x1, y1 = np.min(x, axis=1), np.min(y, axis=1)
        x2, y2 = np.max(x, axis=1), np.max(y, axis=1)

        boxes = np.concatenate((x1.reshape(-1, 1), y1.reshape(
            -1, 1), x2.reshape(-1, 1), y2.reshape(-1, 1)),
                               axis=1)
        boxes_corner = np.concatenate(
            (x.reshape(-1, 8, 1), y.reshape(-1, 8, 1)), axis=2)

        return boxes, boxes_corner

    def camera_dis_to_rect(self, u, v, d):
        """Can only process valid u, v, d, which means u, v can not beyond the
        image shape, reprojection error 0.02.

        :param u: (N)
        :param v: (N)
        :param d: (N), the distance between camera and 3d points,
            d^2 = x^2 + y^2 + z^2
        :return:
        """
        assert self.fu == self.fv, '%.8f != %.8f' % (self.fu, self.fv)
        fd = np.sqrt((u - self.cu)**2 + (v - self.cv)**2 + self.fu**2)
        x = ((u - self.cu) * d) / fd + self.tx
        y = ((v - self.cv) * d) / fd + self.ty
        z = np.sqrt(d**2 - x**2 - y**2)
        pts_rect = np.concatenate(
            (x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)), axis=1)
        return pts_rect

    def inverse_rigid_trans(self, Tr):
        """Inverse a rigid body transform matrix (3x4 as [R|t])

        [R'|-R't; 0|1]
        """
        inv_Tr = np.zeros_like(Tr)  # 3x4
        inv_Tr[0:3, 0:3] = np.transpose(Tr[0:3, 0:3])
        inv_Tr[0:3, 3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
        return inv_Tr

    def alpha2ry(self, alpha, u):
        """
        Get rotation_y by alpha + theta - 180
        alpha : Observation angle of object, ranging [-pi..pi]
        x : Object center x to the camera center (x-W/2), in pixels
        rotation_y : Rotation ry around Y-axis in camera coordinates [-pi..pi]
        """
        ry = alpha + np.arctan2(u - self.cu, self.fu)

        if ry > np.pi:
            ry -= 2 * np.pi
        if ry < -np.pi:
            ry += 2 * np.pi

        return ry

    def ry2alpha(self, ry, u):

        alpha = ry - np.arctan2(u - self.cu, self.fu)

        if alpha > np.pi:
            alpha -= 2 * np.pi
        if alpha < -np.pi:
            alpha += 2 * np.pi

        return alpha


class Calibration(object):
    def __init__(self, calib_file):
        if isinstance(calib_file, str):
            calib = get_calib_from_file(calib_file)
        else:
            calib = calib_file

        self.P2 = calib['P2']  # 3 x 4
        self.R0 = calib['R0']  # 3 x 3
        self.V2C = calib['Tr_velo2cam']  # 3 x 4
        self.C2V = self.inverse_rigid_trans(self.V2C)

        # Camera intrinsics and extrinsics
        self.cu = self.P2[0, 2]
        self.cv = self.P2[1, 2]
        self.fu = self.P2[0, 0]
        self.fv = self.P2[1, 1]
        self.tx = self.P2[0, 3] / (-self.fu)
        self.ty = self.P2[1, 3] / (-self.fv)

    def cart_to_hom(self, pts):
        """
        :param pts: (N, 3 or 2)
        :return pts_hom: (N, 4 or 3)
        """
        pts_hom = np.hstack((pts, np.ones((pts.shape[0], 1),
                                          dtype=np.float32)))
        return pts_hom

    def lidar_to_rect(self, pts_lidar):
        """
        :param pts_lidar: (N, 3)
        :return pts_rect: (N, 3)
        """
        pts_lidar_hom = self.cart_to_hom(pts_lidar)
        pts_rect = np.dot(pts_lidar_hom, np.dot(self.V2C.T, self.R0.T))
        # pts_rect = reduce(np.dot, (pts_lidar_hom, self.V2C.T, self.R0.T))
        return pts_rect

    def rect_to_lidar(self, pts_rect):
        pts_ref = np.transpose(
            np.dot(np.linalg.inv(self.R0), np.transpose(pts_rect)))
        pts_ref = self.cart_to_hom(pts_ref)  # nx4
        return np.dot(pts_ref, np.transpose(self.C2V))

    def rect_to_img(self, pts_rect):
        """
        :param pts_rect: (N, 3)
        :return pts_img: (N, 2)
        """
        # pts_rect_hom = self.cart_to_hom(pts_rect)
        print(f' pts rect : {pts_rect.shape}')
        pts_2d_hom = np.dot(pts_rect, self.P2.T)
        print(f' 2d homo : {pts_2d_hom.shape}')

        pts_img = (pts_2d_hom.T / pts_2d_hom[:, 2]).T  # (N, 2)
        print(f' pts img : {pts_img.shape}')

        pts_rect_depth = pts_2d_hom[:, 2] - self.P2.T[
            3, 2]  # depth in rect camera coord
        return pts_img, pts_rect_depth

    def dair_rect_to_img(self, pts_rect):
        """
        :param pts_rect: (N, 3)
        :return pts_img: (N, 2)
        """
        # pts_rect_hom = self.cart_to_hom(pts_rect)
        print(f' pts rect : {pts_rect.shape}')
        pts_2d_hom = np.dot(pts_rect, self.P2.T)
        print(f' 2d homo : {pts_2d_hom.shape}')

        pts_img = (pts_2d_hom.T / pts_2d_hom[:, 2]).T  # (N, 2)
        print(f' pts img : {pts_img.shape}')

        pts_rect_depth = pts_2d_hom[:, 2]  # depth in rect camera coord
        return pts_img, pts_rect_depth

    def lidar_to_img(self, pts_lidar):
        """
        :param pts_lidar: (N, 3)
        :return pts_img: (N, 2)
        """
        pts_rect = self.lidar_to_rect(pts_lidar)
        pts_img, pts_depth = self.rect_to_img(pts_rect)
        return pts_img, pts_depth

    def img_to_rect(self, u, v, depth_rect):
        """
        :param u: (N)
        :param v: (N)
        :param depth_rect: (N)
        :return:
        """
        x = ((u - self.cu) * depth_rect) / self.fu + self.tx
        y = ((v - self.cv) * depth_rect) / self.fv + self.ty
        pts_rect = np.concatenate(
            (x.reshape(-1, 1), y.reshape(-1, 1), depth_rect.reshape(-1, 1)),
            axis=1)
        return pts_rect

    def depthmap_to_rect(self, depth_map):
        """
        :param depth_map: (H, W), depth_map
        :return:
        """
        x_range = np.arange(0, depth_map.shape[1])
        y_range = np.arange(0, depth_map.shape[0])
        x_idxs, y_idxs = np.meshgrid(x_range, y_range)
        x_idxs, y_idxs = x_idxs.reshape(-1), y_idxs.reshape(-1)
        depth = depth_map[y_idxs, x_idxs]
        pts_rect = self.img_to_rect(x_idxs, y_idxs, depth)
        return pts_rect, x_idxs, y_idxs

    def corners3d_to_img_boxes(self, corners3d):
        """
        :param corners3d: (N, 8, 3) corners in rect coordinate
        :return: boxes: (None, 4) [x1, y1, x2, y2] in rgb coordinate
        :return: boxes_corner: (None, 8) [xi, yi] in rgb coordinate
        """
        sample_num = corners3d.shape[0]
        corners3d_hom = np.concatenate((corners3d, np.ones(
            (sample_num, 8, 1))),
                                       axis=2)  # (N, 8, 4)

        img_pts = np.matmul(corners3d_hom, self.P2.T)  # (N, 8, 3)

        x, y = img_pts[:, :, 0] / img_pts[:, :, 2], img_pts[:, :,
                                                            1] / img_pts[:, :,
                                                                         2]
        x1, y1 = np.min(x, axis=1), np.min(y, axis=1)
        x2, y2 = np.max(x, axis=1), np.max(y, axis=1)

        boxes = np.concatenate((x1.reshape(-1, 1), y1.reshape(
            -1, 1), x2.reshape(-1, 1), y2.reshape(-1, 1)),
                               axis=1)
        boxes_corner = np.concatenate(
            (x.reshape(-1, 8, 1), y.reshape(-1, 8, 1)), axis=2)

        return boxes, boxes_corner

    def camera_dis_to_rect(self, u, v, d):
        """Can only process valid u, v, d, which means u, v can not beyond the
        image shape, reprojection error 0.02.

        :param u: (N)
        :param v: (N)
        :param d: (N), the distance between camera and 3d points,
            d^2 = x^2 + y^2 + z^2
        :return:
        """
        assert self.fu == self.fv, '%.8f != %.8f' % (self.fu, self.fv)
        fd = np.sqrt((u - self.cu)**2 + (v - self.cv)**2 + self.fu**2)
        x = ((u - self.cu) * d) / fd + self.tx
        y = ((v - self.cv) * d) / fd + self.ty
        z = np.sqrt(d**2 - x**2 - y**2)
        pts_rect = np.concatenate(
            (x.reshape(-1, 1), y.reshape(-1, 1), z.reshape(-1, 1)), axis=1)
        return pts_rect

    def inverse_rigid_trans(self, Tr):
        """Inverse a rigid body transform matrix (3x4 as [R|t])

        [R'|-R't; 0|1]
        """
        inv_Tr = np.zeros_like(Tr)  # 3x4
        inv_Tr[0:3, 0:3] = np.transpose(Tr[0:3, 0:3])
        inv_Tr[0:3, 3] = np.dot(-np.transpose(Tr[0:3, 0:3]), Tr[0:3, 3])
        return inv_Tr

    def alpha2ry(self, alpha, u):
        """
        Get rotation_y by alpha + theta - 180
        alpha : Observation angle of object, ranging [-pi..pi]
        x : Object center x to the camera center (x-W/2), in pixels
        rotation_y : Rotation ry around Y-axis in camera coordinates [-pi..pi]
        """
        ry = alpha + np.arctan2(u - self.cu, self.fu)

        if ry > np.pi:
            ry -= 2 * np.pi
        if ry < -np.pi:
            ry += 2 * np.pi

        return ry

    def ry2alpha(self, ry, u):
        alpha = ry - np.arctan2(u - self.cu, self.fu)

        if alpha > np.pi:
            alpha -= 2 * np.pi
        if alpha < -np.pi:
            alpha += 2 * np.pi

        return alpha


# endregion


# region ==affine trainsform==
def get_dir(src_point, rot_rad):
    sn, cs = np.sin(rot_rad), np.cos(rot_rad)

    src_result = [0, 0]
    src_result[0] = src_point[0] * cs - src_point[1] * sn
    src_result[1] = src_point[0] * sn + src_point[1] * cs

    return src_result


def get_3rd_point(a, b):
    direct = a - b
    return b + np.array([-direct[1], direct[0]], dtype=np.float32)


def get_affine_transform(center,
                         scale,
                         rot,
                         output_size,
                         shift=np.array([0, 0], dtype=np.float32),
                         inv=0):
    if not isinstance(scale, np.ndarray) and not isinstance(scale, list):
        scale = np.array([scale, scale], dtype=np.float32)

    scale_tmp = scale
    src_w = scale_tmp[0]
    dst_w = output_size[0]
    dst_h = output_size[1]

    rot_rad = np.pi * rot / 180
    src_dir = get_dir([0, src_w * -0.5], rot_rad)
    dst_dir = np.array([0, dst_w * -0.5], np.float32)

    src = np.zeros((3, 2), dtype=np.float32)
    dst = np.zeros((3, 2), dtype=np.float32)
    src[0, :] = center + scale_tmp * shift
    src[1, :] = center + src_dir + scale_tmp * shift
    dst[0, :] = [dst_w * 0.5, dst_h * 0.5]
    dst[1, :] = np.array([dst_w * 0.5, dst_h * 0.5], np.float32) + dst_dir

    src[2:, :] = get_3rd_point(src[0, :], src[1, :])
    dst[2:, :] = get_3rd_point(dst[0, :], dst[1, :])

    if inv:
        trans = cv2.getAffineTransform(np.float32(src), np.float32(dst))
        trans_inv = cv2.getAffineTransform(np.float32(dst), np.float32(src))
        return trans, trans_inv
    else:
        trans = cv2.getAffineTransform(np.float32(src), np.float32(dst))
    return trans


def affine_transform(pt, t):
    new_pt = np.array([pt[0], pt[1], 1.], dtype=np.float32).T
    new_pt = np.dot(t, new_pt)
    return new_pt[:2]
