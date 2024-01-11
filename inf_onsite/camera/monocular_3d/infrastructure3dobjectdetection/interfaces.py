import logging

import numpy as np

VALID_CLS = [
    'car', 'van', 'truck', 'trailer', 'bus', 'pedestrian', 'cyclist',
    'motorcyclist', 'barrow', 'tricyclist'
]


class Object3D(object):
    """3d object label if len(label_file_line) > 14: it is kitti label if
    len(label_file_line) == 14: it is challenge5 label."""
    def __init__(self, label_file_line, format='challenge5'):
        data = label_file_line.split(' ')
        self.format = format
        assert self.format in ['challenge5', 'kitti']

        if self.format == 'kitti':
            self.load_kitti(data)
        elif self.format == 'challenge5':
            self.load_challenge5(data)
        else:
            logging.error('Not supported data format {}'.format(self.format))

    def load_kitti(self, data):
        data[1:] = [float(x) for x in data[1:]]
        # extract label, truncation, occlusion
        self.type = data[0].lower()  # 'car', 'pedestrian', ...
        self.truncation = data[1]  # truncated pixel ratio [0..1]
        self.occlusion = int(
            data[2]
        )  # 0=visible, 1=partly occluded, 2=fully occluded, 3=unknown
        self.alpha = data[3]  # object observation angle [-pi..pi]

        # extract 2d bounding box in 0-based coordinates
        self.xmin = data[4]  # left
        self.ymin = data[5]  # top
        self.xmax = data[6]  # right
        self.ymax = data[7]  # bottom
        self.box2d = np.array([self.xmin, self.ymin, self.xmax, self.ymax])

        # extract 3d bounding box information
        self.height = data[8]  # box height (in meters)
        self.width = data[9]  # box width
        self.length = data[10]  # box length
        self.x = data[11]
        self.y = data[12]
        self.z = data[13]  # location (x,y,z) in camera coord
        # yaw angle (around Y-axis in camera coordinates) [-pi..pi]
        self.ry = data[14]
        self.check_sum = self.height + self.width + self.length

    def load_challenge5(self, data):
        data[1:] = [float(x) for x in data[1:]]
        # extract label, truncation, occlusion
        self.type = data[0].lower()  # 'Car', 'Pedestrian', ...
        self.truncation = data[1]  # truncated pixel ratio [0..1]
        self.occlusion = int(data[2])

        # extract 2d bounding box in 0-based coordinates
        self.xmin = data[3]  # left
        self.ymin = data[4]  # top
        self.xmax = data[5]  # right
        self.ymax = data[6]  # bottom
        self.box2d = np.array([self.xmin, self.ymin, self.xmax, self.ymax])
        # challenge 5 format
        # labels (x,y,z,h,w,l) are in camera coordinate
        self.height = data[7]
        self.width = data[8]
        self.length = data[9]
        self.x = data[10]
        self.y = data[11]
        self.z = data[12]
        # yaw angle (around Y-axis in camera coordinates) [-pi..pi]
        self.ry = data[13]
        self.check_sum = self.height + self.width + self.length

    def __repr__(self, DEBUG=False):
        """Export challenge 5 format.

        Returns:
            Export challenge 5 format
        """
        if DEBUG:
            print('Type:(%s), Truncation(%f), occlusion(%f)' %
                  (self.type, self.truncation, self.occlusion))
            print('2D bbox: (%f, %f, %f, %f)' %
                  (self.xmin, self.ymin, self.xmax, self.ymax))
            print('3D obj (h,w,l): (%f, %f, %f)' %
                  (self.height, self.width, self.length))
            print('3D obj location (x, y, z):(%f, %f, %f), ry:%f' %
                  (self.x, self.y, self.z, self.ry))

        return '{0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13}'\
            .format(self.type, self.truncation, self.occlusion,
                    round(self.xmin, 2), round(self.ymin, 2),
                    round(self.xmax, 2), round(self.ymax, 2),
                    round(self.height, 2), round(self.width, 2),
                    round(self.length, 2),
                    round(self.x, 2), round(self.y, 2), round(self.z, 2),
                    round(self.ry, 2))


class Calibration(object):
    """Calibration file image2 coord:

     ----> x-axis (u)
    |
    |
    v y-axis (v)

    lidar coord:
    front x, left y, up z

    rect/ref camera coord:
    right x, down y, front z

    Ref (KITTI paper): http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    """
    def __init__(self, calib_filepath):
        calibs = self.read_calib_file(calib_filepath)
        # Projection matrix from rect camera coord to image2 coord
        self.P = calibs['P2']
        self.P = np.reshape(self.P, [3, 4])
        self.c_u = self.P[0, 2]
        self.c_v = self.P[1, 2]
        self.f_u = self.P[0, 0]
        self.f_v = self.P[1, 1]
        self.b_x = self.P[0, 3] / (-self.f_u)  # relative
        self.b_y = self.P[1, 3] / (-self.f_v)

    def read_calib_file(self, filepath):
        """Read in a calibration file and parse into a dictionary.

        Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
        """
        data = {}
        with open(filepath, 'r') as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0:
                    continue
                key, value = line.split(':', 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass

        return data
