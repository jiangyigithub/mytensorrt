# Based on nuScenes dev-kit written by Oscar Beijbom, 2019.

import os

# import numpy as np
from common.data_classes import EvalBoxes
from detection.data_classes import DetectionBox
from tqdm import tqdm


def load_prediction(result_path: str, box_cls) -> EvalBoxes:
    """Loads object predictions from file.

    :param result_path: Path to the .json result file provided by the user.
    :param box_cls: Type of box to load, e.g. DetectionBox.
    :param verbose: Whether to print messages to stdout.
    :return: The deserialized results and meta data.
    """

    pred_2d_box_list = os.listdir(result_path)
    content = dict()
    for name in tqdm(pred_2d_box_list):
        pred_2d_array = []
        data_list = []
        # Try read predictions
        with open(os.path.join(result_path, name), 'r') as file_to_read:
            for line in file_to_read.readlines():
                line_list = line.strip().split(' ')
                pred_2d_array.append(line_list)

        for i, line in enumerate(pred_2d_array):
            # extracting values from annotation
            box_cls_res = line[0]
            box_2d_xmin = float(line[3])
            box_2d_ymin = float(line[4])
            box_2d_xmax = float(line[5])
            box_2d_ymax = float(line[6])
            box_3d_size = [float(line[7]), float(line[8]), float(line[9])]
            box_3d_xyz = [float(line[10]), float(line[11]), float(line[12])]
            box_3d_yaw = float(line[13])
            box_conf = float(line[14])

            data = dict(
                sample_token=name,
                translation=box_3d_xyz,
                classification=box_cls_res,
                bbox_2d=[box_2d_xmin, box_2d_ymin, box_2d_xmax, box_2d_ymax],
                size=box_3d_size,
                rotation=box_3d_yaw,
                detection_score=box_conf)
            data_list.append(data)

        content[name] = data_list
    all_results = EvalBoxes.deserialize(content, box_cls)
    return all_results


def load_prediction_selection(result_path: str, box_cls,
                              choice: str) -> EvalBoxes:
    """Loads object predictions from file.

    :param result_path: Path to the .json result file provided by the user.
    :param box_cls: Type of box to load, e.g. DetectionBox.
    :param verbose: Whether to print messages to stdout.
    :return: The deserialized results and meta data.
    """

    pred_2d_box_list = []
    configuration_name = choice
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(this_dir, '..', 'detection', 'configs',
                            '%s.txt' % configuration_name)
    assert os.path.exists(
        cfg_path), 'Requested unknown configuration {}'.format(
            configuration_name)
    with open(cfg_path, 'r') as cfg_read:
        # Try read predictions
        pred_2d_box_list = cfg_read.read().splitlines()

    content = dict()
    for name in tqdm(pred_2d_box_list):
        pred_2d_array = []
        data_list = []
        # Try read predictions
        with open(os.path.join(result_path, name), 'r') as file_to_read:
            for line in file_to_read.readlines():
                line_list = line.strip().split(' ')
                pred_2d_array.append(line_list)

        for i, line in enumerate(pred_2d_array):
            # extracting values from annotation
            box_cls_res = line[0]
            box_2d_xmin = float(line[3])
            box_2d_ymin = float(line[4])
            box_2d_xmax = float(line[5])
            box_2d_ymax = float(line[6])
            box_3d_size = [float(line[7]), float(line[8]), float(line[9])]
            box_3d_xyz = [float(line[10]), float(line[11]), float(line[12])]
            box_3d_yaw = float(line[13])
            box_conf = float(line[14])

            data = dict(
                sample_token=name,
                translation=box_3d_xyz,
                classification=box_cls_res,
                bbox_2d=[box_2d_xmin, box_2d_ymin, box_2d_xmax, box_2d_ymax],
                size=box_3d_size,
                rotation=box_3d_yaw,
                detection_score=box_conf)
            data_list.append(data)

        content[name] = data_list
    all_results = EvalBoxes.deserialize(content, box_cls)
    return all_results


def load_prediction_selection_zip(pred_zip, box_cls, choice: str) -> EvalBoxes:
    """Loads object predictions from file.

    :param result_path: Path to the .json result file provided by the user.
    :param box_cls: Type of box to load, e.g. DetectionBox.
    :param verbose: Whether to print messages to stdout.
    :return: The deserialized results and meta data.
    """

    pred_2d_box_list = []
    configuration_name = choice
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(this_dir, '..', 'detection', 'configs',
                            '%s.txt' % configuration_name)
    assert os.path.exists(
        cfg_path), 'Requested unknown configuration {}'.format(
            configuration_name)
    with open(cfg_path, 'r') as cfg_read:
        # Try read predictions
        pred_2d_box_list = cfg_read.read().splitlines()

    content = dict()
    for name in tqdm(pred_2d_box_list):
        pred_2d_array = []
        data_list = []
        with pred_zip.open(name) as file_to_read:
            # Try read predictions
            # print(file_to_read.readlines())
            for line in file_to_read.readlines():
                # print(line.strip())
                # print(type(line.strip().split(" ")))
                line_list = line.strip().split(' '.encode(encoding='utf-8'))
                pred_2d_array.append(line_list)

        for i, line in enumerate(pred_2d_array):
            # extracting values from annotation
            box_cls_res = str(line[0], encoding='utf-8')
            box_2d_xmin = float(line[3])
            box_2d_ymin = float(line[4])
            box_2d_xmax = float(line[5])
            box_2d_ymax = float(line[6])
            box_3d_size = [float(line[7]), float(line[8]), float(line[9])]
            box_3d_xyz = [float(line[10]), float(line[11]), float(line[12])]
            box_3d_yaw = float(line[13])
            box_conf = float(line[14])

            data = dict(
                sample_token=name,
                translation=box_3d_xyz,
                classification=box_cls_res,
                bbox_2d=[box_2d_xmin, box_2d_ymin, box_2d_xmax, box_2d_ymax],
                size=box_3d_size,
                rotation=box_3d_yaw,
                detection_score=box_conf)
            data_list.append(data)

        content[name] = data_list
    all_results = EvalBoxes.deserialize(content, box_cls)
    return all_results


def load_gt(result_path: str, box_cls) -> EvalBoxes:
    """Loads object predictions from file.

    :param result_path: Path to the .json result file provided by the user.
    :param box_cls: Type of box to load, e.g. DetectionBox.
    :param verbose: Whether to print messages to stdout.
    :return: The deserialized results and meta data.
    """

    pred_2d_box_list = os.listdir(result_path)
    content = dict()
    for name in tqdm(pred_2d_box_list):
        # hash_token = hashlib.md5( (str(name)).encode("utf8") ).hexdigest()
        pred_2d_array = []
        data_list = []

        with open(os.path.join(result_path, name), 'r') as file_to_read:
            # Try read predictions
            for line in file_to_read.readlines():
                line_list = line.strip().split(' ')
                pred_2d_array.append(line_list)

        for i, line in enumerate(pred_2d_array):
            # extracting values from annotation
            box_cls_res = line[0]
            box_2d_xmin = float(line[3])
            box_2d_ymin = float(line[4])
            box_2d_xmax = float(line[5])
            box_2d_ymax = float(line[6])
            box_3d_size = [float(line[7]), float(line[8]), float(line[9])]
            box_3d_xyz = [float(line[10]), float(line[11]), float(line[12])]
            box_3d_yaw = float(line[13])
            data = dict(
                sample_token=name,
                translation=box_3d_xyz,
                classification=box_cls_res,
                bbox_2d=[box_2d_xmin, box_2d_ymin, box_2d_xmax, box_2d_ymax],
                size=box_3d_size,
                rotation=box_3d_yaw)
            data_list.append(data)
        content[name] = data_list
    all_results = EvalBoxes.deserialize(content, box_cls)
    return all_results


def load_gt_selection(result_path: str, box_cls, choice: str) -> EvalBoxes:
    """Loads object predictions from file.

    :param result_path: Path to the .json result file provided by the user.
    :param box_cls: Type of box to load, e.g. DetectionBox.
    :param choice: file name of the.
    :return: The deserialized results and meta data.
    """

    pred_2d_box_list = []
    configuration_name = choice
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(this_dir, '..', 'detection', 'configs',
                            '%s.txt' % configuration_name)
    assert os.path.exists(
        cfg_path), 'Requested unknown configuration {}'.format(
            configuration_name)
    with open(cfg_path, 'r') as cfg_read:
        pred_2d_box_list = cfg_read.read().splitlines()

    # pred_2d_box_list = os.listdir(result_path)
    content = dict()
    for name in tqdm(pred_2d_box_list):
        # hash_token = hashlib.md5( (str(name)).encode("utf8") ).hexdigest()
        pred_2d_array = []
        data_list = []

        with open(os.path.join(result_path, name), 'r') as file_to_read:
            # Try read predictions
            for line in file_to_read.readlines():
                line_list = line.strip().split(' ')
                pred_2d_array.append(line_list)

        for i, line in enumerate(pred_2d_array):
            # extracting values from annotation
            box_cls_res = line[0]
            box_2d_xmin = float(line[3])
            box_2d_ymin = float(line[4])
            box_2d_xmax = float(line[5])
            box_2d_ymax = float(line[6])
            box_3d_size = [float(line[7]), float(line[8]), float(line[9])]
            box_3d_xyz = [float(line[10]), float(line[11]), float(line[12])]
            box_3d_yaw = float(line[13])
            data = dict(
                sample_token=name,
                translation=box_3d_xyz,
                classification=box_cls_res,
                bbox_2d=[box_2d_xmin, box_2d_ymin, box_2d_xmax, box_2d_ymax],
                size=box_3d_size,
                rotation=box_3d_yaw)
            data_list.append(data)
        content[name] = data_list
    all_results = EvalBoxes.deserialize(content, box_cls)
    return all_results


def load_gt_selection_zip(pred_zip, box_cls, choice: str) -> EvalBoxes:
    """Loads object predictions from file.

    :param result_path: Path to the .json result file provided by the user.
    :param box_cls: Type of box to load, e.g. DetectionBox.
    :param verbose: Whether to print messages to stdout.
    :return: The deserialized results and meta data.
    """

    pred_2d_box_list = []
    configuration_name = choice
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(this_dir, '..', 'detection', 'configs',
                            '%s.txt' % configuration_name)
    assert os.path.exists(
        cfg_path), 'Requested unknown configuration {}'.format(
            configuration_name)
    with open(cfg_path, 'r') as cfg_read:
        pred_2d_box_list = cfg_read.read().splitlines()

    content = dict()
    for name in tqdm(pred_2d_box_list):
        # hash_token = hashlib.md5( (str(name)).encode("utf8") ).hexdigest()
        pred_2d_array = []
        data_list = []

        with pred_zip.open(name) as file_to_read:
            # print(file_to_read.readlines())
            for line in file_to_read.readlines():
                # print(line.strip())
                # print(type(line.strip().split(" ")))
                line_list = line.strip().split(' '.encode(encoding='utf-8'))
                pred_2d_array.append(line_list)
        for i, line in enumerate(pred_2d_array):
            # extracting values from annotation
            box_cls_res = str(line[0], encoding='utf-8')
            box_2d_xmin = float(line[3])
            box_2d_ymin = float(line[4])
            box_2d_xmax = float(line[5])
            box_2d_ymax = float(line[6])
            box_3d_size = [float(line[7]), float(line[8]), float(line[9])]
            box_3d_xyz = [float(line[10]), float(line[11]), float(line[12])]
            box_3d_yaw = float(line[13])
            data = dict(
                sample_token=name,
                translation=box_3d_xyz,
                classification=box_cls_res,
                bbox_2d=[box_2d_xmin, box_2d_ymin, box_2d_xmax, box_2d_ymax],
                size=box_3d_size,
                rotation=box_3d_yaw)
            data_list.append(data)
        content[name] = data_list
    all_results = EvalBoxes.deserialize(content, box_cls)
    return all_results


def add_center_dist(eval_boxes: EvalBoxes):
    """Adds the cylindrical (xy) center distance from ego vehicle to each box.

    :param nusc: The NuScenes instance.
    :param eval_boxes: A set of boxes, either GT or predictions.
    :return: eval_boxes augmented with center distances.
    """
    for sample_token in eval_boxes.sample_tokens:
        for box in eval_boxes[sample_token]:
            # Both boxes and ego pose are given in global coord system,
            # so distance can be calculated directly.
            # Note that the z component of the ego pose is 0.
            ego_translation = box.translation[0]
            if isinstance(box, DetectionBox):
                #  or isinstance(box, TrackingBox):
                box.ego_translation = ego_translation
            else:
                raise NotImplementedError

    return eval_boxes
