import os

import numpy as np
import torch.utils.data as data
from lib.datasets.dair.utils import (DAIRCalibration, affine_transform,
                                     get_affine_transform,
                                     get_objects_from_label)
from lib.datasets.utils import (angle2class, draw_umich_gaussian,
                                gaussian_radius)
from PIL import Image


class DAIR_Dataset_MonoCon(data.Dataset):
    def __init__(self, split, cfg):
        # basic configuration
        self.root_dir = cfg.get('root_dir',
                                '/home/server/dataset/challenge5_dataset')
        self.split = split
        self.num_classes = 10
        self.max_objs = 50
        self.class_name = cfg.get('writelist', [
            'car', 'van', 'truck', 'trailer', 'bus', 'pedestrian', 'cyclist',
            'motorcyclist', 'barrow', 'tricyclist'
        ])
        self.cls2id = {
            'car': 0,
            'van': 1,
            'truck': 2,
            'trailer': 3,
            'bus': 4,
            'pedestrian': 5,
            'cyclist': 6,
            'motorcyclist': 7,
            'barrow': 8,
            'tricyclist': 9
        }  # {'pedestrian': 0, 'car': 1, 'cyclist': 2}
        self.resolution = np.array([1920, 1080])  # W * H
        self.use_3d_center = cfg.get('use_3d_center', True)
        self.writelist = cfg.get('writelist', ['car'])
        # anno: use src annotations as GT, proj: use projected 2d bboxes as GT
        self.bbox2d_type = cfg.get('bbox2d_type', 'anno')
        assert self.bbox2d_type in ['anno', 'proj']
        self.meanshape = cfg.get('meanshape', False)
        self.class_merging = cfg.get('class_merging', False)
        self.use_dontcare = cfg.get('use_dontcare', False)

        if self.class_merging:
            self.writelist.extend([
                'van', 'truck', 'bus', 'tricyclist', 'motorcyclist', 'barrow'
            ])
        if self.use_dontcare:
            self.writelist.extend(['DontCare'])

        # data split loading
        assert self.split in ['train', 'val', 'trainval', 'test']

        # path configuration
        if self.split == 'train':
            self.data_dir = os.path.join(self.root_dir, 'training')
        elif self.split == 'val':
            self.data_dir = os.path.join(self.root_dir, 'validation')
        elif self.split == 'test':
            self.data_dir = os.path.join(self.root_dir, 'test')

        self.image_dir = os.path.join(self.data_dir, 'images')
        self.depth_dir = os.path.join(self.data_dir, 'depth')
        self.calib_dir = os.path.join(self.data_dir, 'calib')
        self.label_dir = os.path.join(self.data_dir, 'labels')

        self.idx_list = sorted(os.listdir(self.image_dir))

        self.idx_list = [x.strip('.jpg') for x in self.idx_list]
        # data augmentation configuration
        self.data_augmentation = True if split in ['train', 'trainval'
                                                   ] else False
        self.random_flip = cfg.get('random_flip', 0.5)
        self.random_crop = cfg.get('random_crop', 0.5)
        self.scale = cfg.get('scale', 0.4)
        self.shift = cfg.get('shift', 0.1)

        # statistics
        # self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        # self.std  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.mean = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.std = np.array([1.0, 1.0, 1.0], dtype=np.float32)

        # 3 class E(h) E(w) E(l)
        self.cls_mean_size = np.array([[1.76255119, 0.66068622, 0.84422524],
                                       [1.52563191, 1.62856739, 3.52588311],
                                       [1.73698127, 0.59706367, 1.76282397]],
                                      dtype=np.float32)  # H*W*L
        if not self.meanshape:
            self.cls_mean_size = np.zeros_like(self.cls_mean_size,
                                               dtype=np.float32)

        # others
        self.downsample = 4
        self.num_kpt = 8
        self.vector_regression_level = 1

    def get_image(self, idx):
        img_file = os.path.join(self.image_dir, '%06d.jpg' % idx)
        assert os.path.exists(img_file)
        return Image.open(img_file)

    def get_label(self, idx):
        label_file = os.path.join(self.label_dir, '%06d.txt' % idx)
        assert os.path.exists(label_file)
        return get_objects_from_label(label_file)

    def get_calib(self, idx):
        calib_file = os.path.join(self.calib_dir, '%06d.txt' % idx)
        assert os.path.exists(calib_file)
        return DAIRCalibration(calib_file)

    def __len__(self):
        return self.idx_list.__len__()

    def __getitem__(self, item):
        """_summary_
        Returns:
        if split is test:
            (img, src, info):
            img:np.array in 0~1,
            src:np.array in 0~255,
            info:dict { image_id:int
                        image_size:(np.array(w,h))
                        down_sample_ratio
                        }
        else:
            (inputs,targets,info): inputs:np.array
        """
        #  region ==get inputs==
        index = int(self.idx_list[item])  # index mapping, get real data id
        # image loading
        img = self.get_image(index)
        src = np.array(img)
        img_size = np.array(img.size)
        features_size = self.resolution // self.downsample  # W * H
        # print(features_size)

        # data augmentation for image
        center = np.array(img_size) / 2
        aug_scale, crop_size = 1.0, img_size
        random_crop_flag, random_flip_flag = False, False
        # print("000000000000000000000000000000000000------------------1111111111111111111111")

        if self.data_augmentation:
            print("000000000000000000000000000000000000------------------")
            if np.random.random() < self.random_flip:
                random_flip_flag = True
                img = img.transpose(Image.FLIP_LEFT_RIGHT)

            if np.random.random() < self.random_crop:
                random_crop_flag = True
                aug_scale = np.clip(np.random.randn() * self.scale + 1,
                                    1 - self.scale, 1 + self.scale)
                crop_size = img_size * aug_scale
                center[0] += img_size[0] * np.clip(
                    np.random.randn() * self.shift, -2 * self.shift,
                    2 * self.shift)
                center[1] += img_size[1] * np.clip(
                    np.random.randn() * self.shift, -2 * self.shift,
                    2 * self.shift)

        # add affine transformation for 2d images.
        trans, trans_inv = get_affine_transform(center,
                                                crop_size,
                                                0,
                                                self.resolution,
                                                inv=1)
        img = img.transform(tuple(self.resolution.tolist()),
                            method=Image.AFFINE,
                            data=tuple(trans_inv.reshape(-1).tolist()),
                            resample=Image.BILINEAR)
        # image encoding
        img = np.array(img).astype(np.float32) / 255.0
        img = (img - self.mean) / self.std
        img = img.transpose(2, 0, 1)  # C * H * W

        info = {
            'img_id': index,
            'img_size': img_size,
            'bbox_downsample_ratio': img_size / features_size
        }

        if self.split == 'test':
            return img, src, info  # img / placeholder(fake label) / info
        # endregion
        # region ==get labels==
        objects = self.get_label(index)
        calib = self.get_calib(index)
        # computed 3d projected box
        if self.bbox2d_type == 'proj':
            for object in objects:
                object.box2d_proj = np.array(calib.corners3d_to_img_boxes(
                    object.generate_corners3d()[None, :])[0][0],
                                             dtype=np.float32)
                object.box2d = object.box2d_proj.copy()

        for object in objects:
            if object.cls_type not in self.writelist:
                continue
            center_3d = object.pos
            center_3d = center_3d.reshape(-1, 3)  # shape adjustment (N, 3)
            object.proj_center_2d = calib.rect_to_img(
                center_3d)  # project 3D center to image plane

            object.corners_3d = np.array(object.generate_corners3d())
            object.valid_corners_mask = np.zeros((8, 1))

            in_front = np.argwhere(object.corners_3d[:, 2] > 0).flatten()
            object.valid_corners_mask[in_front, :] = 1

            corners_3d = object.corners_3d  # (8, 3) array

            object.corners_2d = calib.rect_to_img(
                corners_3d)  # project 3D center to image plane

            # mark points inside the image as visible, the rest is invisible
            for kpt_idx in range(object.corners_2d.shape[0]):
                kptx, kpty = object.corners_2d[kpt_idx]
                is_kpt_in_image = (0 <= kptx <= 1920) and (0 <= kpty <= 1080)
                if is_kpt_in_image:
                    object.valid_corners_mask[kpt_idx] = 2

        # data augmentation for labels
        if random_flip_flag:
            for object in objects:
                [x1, _, x2, _] = object.box2d
                object.box2d[0], object.box2d[
                    2] = img_size[0] - x2, img_size[0] - x1
                object.alpha = np.pi - object.alpha
                object.ry = np.pi - object.ry
                if object.alpha > np.pi:
                    object.alpha -= 2 * np.pi  # check range
                if object.alpha < -np.pi:
                    object.alpha += 2 * np.pi
                if object.ry > np.pi:
                    object.ry -= 2 * np.pi
                if object.ry < -np.pi:
                    object.ry += 2 * np.pi

        # labels encoding
        heatmap = np.zeros(
            (self.num_classes, features_size[1], features_size[0]),
            dtype=np.float32)  # C * H * W
        size_2d = np.zeros((self.max_objs, 2), dtype=np.float32)
        offset_2d = np.zeros((self.max_objs, 2), dtype=np.float32)
        depth = np.zeros((self.max_objs, 1), dtype=np.float32)
        heading_bin = np.zeros((self.max_objs, 1), dtype=np.int64)
        heading_res = np.zeros((self.max_objs, 1), dtype=np.float32)
        src_size_3d = np.zeros((self.max_objs, 3), dtype=np.float32)
        size_3d = np.zeros((self.max_objs, 3), dtype=np.float32)
        offset_3d = np.zeros((self.max_objs, 2), dtype=np.float32)
        indices = np.zeros((self.max_objs), dtype=np.int64)
        mask_2d = np.zeros((self.max_objs), dtype=np.uint8)
        mask_3d = np.zeros((self.max_objs), dtype=np.uint8)
        object_num = len(
            objects) if len(objects) < self.max_objs else self.max_objs

        # 2D-3D kpt heatmap and offset
        center2kpt_offset_target = np.zeros(
            (self.max_objs, self.num_kpt * 2))  # num kpt =8 const
        kpt_heatmap_target = np.zeros(
            (self.num_kpt, features_size[1], features_size[0]),
            dtype=np.float32)
        kpt_heatmap_offset_target = np.zeros((self.max_objs, self.num_kpt * 2))

        mask_center2kpt_offset = np.zeros((self.max_objs, self.num_kpt * 2))
        mask_kpt_heatmap_offset = np.zeros((self.max_objs, self.num_kpt * 2))

        indices_kpt = np.zeros((self.max_objs, self.num_kpt), dtype=np.int64)

        # process object in each image
        for i in range(object_num):
            if objects[i].cls_type not in self.writelist:
                print(objects[i].cls_type, 'not in write list')
                continue

            # filter inappropriate samples
            if objects[i].level_str == 'UnKnown':  # or objects[i].pos[-1] < 2:
                continue

            # process 2d bbox &     get 2d center
            bbox_2d = objects[i].box2d.copy()

            # add affine transformation for 2d boxes.
            bbox_2d[:2] = affine_transform(bbox_2d[:2], trans)
            bbox_2d[2:] = affine_transform(bbox_2d[2:], trans)
            # modify the 2d bbox according to pre-compute downsample ratio
            bbox_2d[:] /= self.downsample

            # process 3d bbox & get 3d center
            center_2d = np.array([(bbox_2d[0] + bbox_2d[2]) / 2,
                                  (bbox_2d[1] + bbox_2d[3]) / 2],
                                 dtype=np.float32)  # W * H

            center_3d = objects[i].proj_center_2d
            center_3d = center_3d[0]  # shape adjustment

            if random_flip_flag:  # random flip for center3d
                center_3d[0] = img_size[0] - center_3d[0]
            center_3d = affine_transform(center_3d.reshape(-1), trans)

            center_3d /= self.downsample

            corners_2d = objects[
                i].corners_2d  # project 3D center to image plane
            if random_flip_flag:  # random flip for center3d
                corners_2d[:, 0] = img_size[0] - corners_2d[:, 0]

            corners_2d_copy = []
            for corner_2d in corners_2d:
                corners_2d_copy.append(affine_transform(corner_2d, trans))

            corners_2d = np.array(corners_2d_copy)
            corners_2d /= self.downsample

            # generate the center of gaussian heatmap
            # [optional: 3d center or 2d center]
            center_heatmap = center_3d.astype(
                np.int32) if self.use_3d_center else center_2d.astype(np.int32)
            if center_heatmap[0] < 0 or center_heatmap[0] >= features_size[0]:
                continue
            if center_heatmap[1] < 0 or center_heatmap[1] >= features_size[1]:
                continue
            # generate the radius of gaussian heatmap
            w, h = bbox_2d[2] - bbox_2d[0], bbox_2d[3] - bbox_2d[1]
            radius = gaussian_radius((w, h))
            radius = max(0, int(radius))

            cls_id = self.cls2id[objects[i].cls_type]  # P Car Cyc
            draw_umich_gaussian(heatmap[cls_id], center_heatmap, radius)

            # encoding 2d/3d offset & 2d size
            indices[
                i] = center_heatmap[1] * features_size[0] + center_heatmap[0]
            offset_2d[i] = center_2d - center_heatmap
            size_2d[i] = 1. * w, 1. * h

            # encoding depth
            depth[i] = objects[i].pos[-1] * aug_scale

            # encoding heading angle
            heading_angle = objects[i].alpha
            heading_bin[i], heading_res[i] = angle2class(heading_angle)

            # encoding 3d offset & size_3d
            offset_3d[i] = center_3d - center_heatmap
            src_size_3d[i] = np.array(
                [objects[i]._h, objects[i]._w, objects[i]._l],
                dtype=np.float32)
            # if use mean size of objects
            # mean_size = self.cls_mean_size[self.cls2id[objects[i].cls_type]]
            mean_size = 0
            size_3d[i] = src_size_3d[i] - mean_size

            mask_2d[i] = 1
            mask_3d[i] = 0 if random_crop_flag else 1

            for k in range(self.num_kpt):
                kpt = corners_2d[k]
                kptx_int, kpty_int = kpt.astype(np.int32)
                kptx, kpty = kpt
                vis_level = objects[i].valid_corners_mask[k]
                if vis_level < self.vector_regression_level:
                    continue

                center2kpt_offset_target[i, k * 2] = kptx - center_heatmap[0]
                center2kpt_offset_target[i,
                                         k * 2 + 1] = kpty - center_heatmap[1]
                mask_center2kpt_offset[i, k * 2:k * 2 + 2] = 1

                is_kpt_inside_image = (0 <= kptx_int < features_size[0]) and (
                    0 <= kpty_int < features_size[1])
                if not is_kpt_inside_image:
                    continue

                draw_umich_gaussian(kpt_heatmap_target[k],
                                    [kptx_int, kpty_int], radius)

                kpt_index = kpty_int * features_size[0] + kptx_int
                indices_kpt[i, k] = kpt_index

                kpt_heatmap_offset_target[i, k * 2] = kptx - kptx_int
                kpt_heatmap_offset_target[i, k * 2 + 1] = kpty - kpty_int
                mask_kpt_heatmap_offset[i, k * 2:k * 2 + 2] = 1

        inputs = img
        targets = {
            'heatmap': heatmap,  #
            'offset_2d': offset_2d,  #
            'size_2d': size_2d,  #
            'offset_3d': offset_3d,
            'size_3d': size_3d,  #
            'depth': depth,  #
            'heading_bin': heading_bin,  #
            'heading_res': heading_res,  #
            'center2kpt_offset_target':
            center2kpt_offset_target,  # #### monocon
            'mask_center2kpt_offset': mask_center2kpt_offset,
            'kpt_heatmap_target': kpt_heatmap_target,
            'kpt_heatmap_offset_target': kpt_heatmap_offset_target,
            'mask_kpt_heatmap_offset': mask_kpt_heatmap_offset,
            'mask_2d': mask_2d,
            'mask_3d': mask_3d,
            'indices': indices,
            'src_size_3d': src_size_3d,
            'indices_kpt': indices_kpt
        }

        info = {
            'img_id': index,
            'img_size': img_size,
            'bbox_downsample_ratio': img_size / features_size
        }
        return inputs, targets, info
