import argparse
import logging
import os
import random

import cv2

import projection
from data_reader import ICVReader


def make_dir_if_not_exist(dir_path: str) -> None:
    """Create directory if not exists.

    Args:
        dir_path (str): the directory you want to create
    """
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)
        logging.info('Created dir: {}'.format(dir_path))


def visualize_3dbox(img, objects, calib, gplane):
    for obj in objects:
        box3d_pts_2d, _ = projection.compute_box_3d(obj, calib.P, gplane)
        img = projection.draw_projected_box3d(img,
                                              box3d_pts_2d,
                                              color=(0, 255, 255))

    return img


def visualize(args):
    data_dir = args.data_dir
    reader = ICVReader(data_dir, 'training', 'challenge5')
    output_dir = args.output_dir
    make_dir_if_not_exist(output_dir)

    filenames = [
        f[:-len('.txt')]
        for f in os.listdir(os.path.join(data_dir, 'training', args.label_tag))
    ]

    for filename in random.sample(filenames, args.num_samples):
        img = reader.load_img(filename)
        label = reader.load_label(filename, args.label_tag)
        calib = reader.load_calib(filename)
        gp = reader.load_gplane(filename)
        img = visualize_3dbox(img, label, calib, gp)
        save_p = os.path.join(output_dir, '{}_3d.jpg'.format(filename))
        cv2.imwrite(save_p, img)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Visualize challenge5 3D boxes',
                                     add_help=False)
    parser.add_argument('--data_dir', '-d', default='./data', type=str)
    parser.add_argument('--num_samples', '-n', default=8933, type=int)
    parser.add_argument('--output_dir', '-o', default='./output', type=str)
    parser.add_argument('--label_tag', '-t', default='labels', type=str)
    args = parser.parse_args()

    visualize(args)
