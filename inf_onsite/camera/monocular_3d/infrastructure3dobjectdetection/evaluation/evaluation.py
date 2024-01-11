import argparse
import os
# import zipfile
from typing import Tuple

import numpy as np
from common.config import config_factory
from common.data_classes import EvalBox, EvalBoxes
from common.loaders import (add_center_dist, load_gt_selection,
                            load_gt_selection_zip, load_prediction_selection,
                            load_prediction_selection_zip)
from detection.algo import accumulate, calc_ap, calc_tp
from detection.constants import TP_METRICS
from detection.data_classes import (DetectionBox, DetectionMetricDataList,
                                    DetectionMetrics)
from detection.render import (class_pr_curve, class_tp_curve, dist_pr_curve,
                              summary_plot)


class GraderException(Exception):
    pass


def read_local_gt_sections(section: str):
    """Load a file list from repo.

    section: name of file list
    """
    name_list = []
    configuration_name = section
    this_dir = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(this_dir, 'detection', 'configs',
                            '%s.txt' % configuration_name)
    assert os.path.exists(
        cfg_path), 'Requested unknown configuration {}'.format(
            configuration_name)
    with open(cfg_path, 'r') as cfg_read:
        name_list = cfg_read.read().splitlines()
    cfg_read.close()
    return name_list


def load_files(pred_file_root_path: str, gt_pub_root_path: str,
               section: str) -> Tuple[EvalBoxes, EvalBoxes, EvalBoxes]:
    """
    Load the files into objects
    pred_file_root_path: the root path in string of 3d object preditions
    gt_pub_root_path:
    -> The root path in string of annotated ground truth in PUBLIC region
    section: (string)  "public" or "private"
    """
    user_pre_bboxes = load_prediction_selection(pred_file_root_path,
                                                DetectionBox, section)
    pub_gt_boxes = load_gt_selection(gt_pub_root_path, DetectionBox, section)
    # pri_gt_boxes = load_gt(gt_pri_root_path,DetectionBox)
    return user_pre_bboxes, pub_gt_boxes
    # return user_pre_bboxes, pub_gt_boxes, pri_gt_boxes


def evaluate(pre_bboxes: EvalBox, gt_boxes: EvalBox, args):
    """Evaluation on nuScense format data and plot pdf report."""
    print(gt_boxes)
    cfg_ = config_factory('detection_cvpr_2019')
    # add center distance
    pre_bboxes = add_center_dist(pre_bboxes)
    gt_boxes = add_center_dist(gt_boxes)

    # -----------------------------------
    # Step 1: Accumulate metric data for all classes and distance thresholds.
    # -----------------------------------
    metric_data_list = DetectionMetricDataList()
    for class_name in cfg_.class_names:
        for dist_th in cfg_.dist_ths:
            md = accumulate(gt_boxes, pre_bboxes, class_name,
                            cfg_.dist_fcn_callable, dist_th)
            metric_data_list.set(class_name, dist_th, md)

    # # -----------------------------------
    # # Step 2: Calculate metrics from the data.
    # # -----------------------------------
    print('Calculating metrics...')
    metrics = DetectionMetrics(cfg_)
    for class_name in cfg_.class_names:
        # Compute APs.
        for dist_th in cfg_.dist_ths:
            metric_data = metric_data_list[(class_name, dist_th)]
            ap = calc_ap(metric_data, cfg_.min_recall, cfg_.min_precision)
            metrics.add_label_ap(class_name, dist_th, ap)

        # Compute TP metrics.
        for metric_name in TP_METRICS:
            metric_data = metric_data_list[(class_name, cfg_.dist_th_tp)]
            if class_name in ['traffic_cone'] and metric_name in [
                    'attr_err', 'vel_err', 'orient_err'
            ]:
                tp = np.nan
            elif class_name in ['barrier'
                                ] and metric_name in ['attr_err', 'vel_err']:
                tp = np.nan
            else:
                tp = calc_tp(metric_data, cfg_.min_recall, metric_name)
            metrics.add_label_tp(class_name, metric_name, tp)

    md_list = metric_data_list

    if args.enable_pdf_report is not None:
        print('Rendering PR and TP curves')

        def savepath(name):
            return os.path.join(args.enable_pdf_report, name + '.pdf')

        summary_plot(md_list,
                     metrics,
                     min_precision=cfg_.min_precision,
                     min_recall=cfg_.min_recall,
                     dist_th_tp=cfg_.dist_th_tp,
                     savepath=savepath('summary'))

        for detection_name in cfg_.class_names:
            class_pr_curve(md_list,
                           metrics,
                           detection_name,
                           cfg_.min_precision,
                           cfg_.min_recall,
                           savepath=savepath(detection_name + '_pr'))

            class_tp_curve(md_list,
                           metrics,
                           detection_name,
                           cfg_.min_recall,
                           cfg_.dist_th_tp,
                           savepath=savepath(detection_name + '_tp'))

        for dist_th in cfg_.dist_ths:
            dist_pr_curve(md_list,
                          metrics,
                          dist_th,
                          cfg_.min_precision,
                          cfg_.min_recall,
                          savepath=savepath('dist_pr_' + str(dist_th)))

    metrics_summary = metrics.serialize()
    # Print high-level metrics.
    print('mAP: %.4f' % (metrics_summary['mean_ap']))
    err_name_mapping = {
        'trans_err': 'mATE',
        'scale_err': 'mASE',
        # 'orient_err': 'mAOE',
        # 'vel_err': 'mAVE',
        # 'attr_err': 'mAAE'
    }
    for tp_name, tp_val in metrics_summary['tp_errors'].items():
        print('%s: %.4f' % (err_name_mapping[tp_name], tp_val))
    print('NDS: %.4f' % (metrics_summary['nd_score']))
    # print('Eval time: %.1fs' % metrics_summary['eval_time'])

    # Print per-class metrics.
    print()
    print('Per-class results:')
    print('Object Class\tAP\tATE\tASE')  # \tAOE\tAVE\tAAE')
    class_aps = metrics_summary['mean_dist_aps']
    class_tps = metrics_summary['label_tp_errors']
    for class_name in class_aps.keys():
        print(
            '%12s\t%.3f\t%.3f\t%.3f'  # \t%.3f\t%.3f\t%.3f'
            % (
                class_name,
                class_aps[class_name],
                class_tps[class_name]['trans_err'],
                class_tps[class_name]['scale_err'],
                # class_tps[class_name]['orient_err'],
                # class_tps[class_name]['vel_err'],
                # class_tps[class_name]['attr_err']
            ))
    return (class_aps['car'] + class_aps['pedestrian'] + class_aps['cyclist'] +
            class_aps['motorcyclist']) / 4


def sub_list_checker(zip):
    """
    Checking if the submission files are exactly the same
    with the required test list
    zip: zip object read from the submission
    """
    user_name_list = read_local_gt_sections('user')
    full_list = user_name_list
    full_list.sort()
    zip.namelist().sort()
    if zip.namelist() == full_list:
        print('checking passed!')
        return True
    else:
        return False


def i3od_checker(args):
    """Checking if the submission files are exactly the same with the required
    test list."""
    user_name_list = read_local_gt_sections('user')
    full_list = user_name_list
    full_list.sort()
    pred_list = os.listdir(args.pred_file_path)
    pred_list.sort()
    if pred_list == full_list:
        print('Congratulation~ checking passed, \
            Please wrap the files into one single zip for submission')
        return True
    else:
        print('Checking NOT passed!, \
            Please check if your pred files are incomplete or redundant')
        return False


def i3od_grader_zip(
        submission_filepath,
        gt_filepath  # , key_col, label_col, prediction_col
):
    import zipfile

    with zipfile.ZipFile(gt_filepath, 'r') as gt_zip, zipfile.ZipFile(
            submission_filepath, 'r') as pred_zip:

        if sub_list_checker(pred_zip) is False:
            # raise error()
            raise GraderException(
                "Error in submission '{}'has missing or additional files,\
                please make sure your submission follows the README file rules"
                .format(submission_filepath))
        user_pre_bboxes = load_prediction_selection_zip(
            pred_zip, DetectionBox, 'user')
        gt_bboxes = load_gt_selection_zip(gt_zip, DetectionBox, 'user')
        score = evaluate(user_pre_bboxes, gt_bboxes, args)
        print('Final Score for submission (mAP):')
        print(score)

    return score


def i3od_grader(args):
    """Evaluation wrapper."""
    user_pre_bboxes = load_prediction_selection(args.pred_file_path,
                                                DetectionBox, 'user')
    gt_bboxes = load_gt_selection(args.gt_file_path, DetectionBox, 'user')
    score = evaluate(user_pre_bboxes, gt_bboxes, args)
    print('Final Score for submission (mAP):')
    print(score)

    return score


def main(args):

    if args.checking_submission is False:
        i3od_grader(args)
    else:
        i3od_checker(args)


if __name__ == '__main__':

    parser = argparse.ArgumentParser('Evaluation: mAP of 3D boxes',
                                     add_help=True)
    parser.add_argument('--pred_file_path',
                        '-p',
                        type=str,
                        help='global path to the prediciton folder')
    parser.add_argument('--gt_file_path',
                        '-g',
                        type=str,
                        help='global path to the ground truth folder')
    parser.add_argument('--enable_pdf_report',
                        '-d',
                        type=str,
                        help='global path for saving pdf reports, \
                            only valid when enable_pdf_report is True')
    parser.add_argument('--checking_submission',
                        '-c',
                        default=False,
                        action=argparse.BooleanOptionalAction,
                        help='global path for saving pdf reports, \
                            only valid when enable_pdf_report is True')

    args = parser.parse_args()
    main(args)
