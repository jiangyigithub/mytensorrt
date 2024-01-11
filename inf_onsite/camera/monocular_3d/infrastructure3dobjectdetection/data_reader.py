import os

import cv2
import numpy as np

# from torch.utils.data import Dataset
from interfaces import Calibration, Object3D


def parse_label(label_filename: str, format: str) -> list:
    """Parse AI challenge 5 label.

    Args:
        label_filename (str): label file path
        format (str): dataset format, e.g. kitti, challenge5

    Returns:
        list: list of Object3D objects
    """
    lines = [line.rstrip() for line in open(label_filename)]
    objects = [Object3D(line, format=format) for line in lines]
    return objects


class ICVReader(object):
    """fileio for read rope3d dataset."""
    def __init__(self, root_path, split, format='challenge5'):
        self.root_path = root_path
        assert split in ['training', 'validation',
                         'test'], 'Invalid split: {}'.format(split)
        self.split = split
        self.format = format

    def load_img(self, filename):
        filepath = os.path.join(self.root_path, self.split, 'images',
                                filename + '.jpg')
        return cv2.imread(filepath)

    def load_label(self, filename, label_tag):
        filename = os.path.join(self.root_path, self.split, label_tag,
                                filename + '.txt')
        return parse_label(filename, self.format)

    def load_gplane(self, filename):
        filename = os.path.join(self.root_path, self.split, 'groundplane',
                                filename + '.txt')
        handler = open(filename, 'r')
        lines = [line.strip().split(' ') for line in handler.readlines()]
        coef = list(map(lambda x: float(x), lines[0]))
        return np.array(coef, dtype=np.float32)

    def load_calib(self, filename):
        filename = os.path.join(self.root_path, self.split, 'calib',
                                filename + '.txt')
        return Calibration(filename)


# class ICVDataset(Dataset):
#     def __init__(self, root_path, split='training'):
#         super(ICVDataset, self).__init__()
#         self.root_path = root_path
#         self.split = split
#         self.data_reader = ICVReader(root_path, split)
#         self.sample_set = self.load_set()

#     def load_set(self):
#         label_dir = os.path.join(self.root_path, self.split, 'labels')
#         filenames = [f[:-len('.txt')] for f in os.listdir(label_dir)]
#         return filenames

#     def __len__(self):
#         return len(self.sample_set)

#     def __getitem__(self, index):
#         filename = self.sample_set[index]
#         data = {
#             'image': self.data_reader.load_img(filename),
#             'calib': self.data_reader.load_calib(filename),
#             'gplane': self.data_reader.load_gplane(filename),
#             'labels': self.data_reader.load_label(filename),
#             'name': filename,
#             'idx': index
#         }
#         return data
