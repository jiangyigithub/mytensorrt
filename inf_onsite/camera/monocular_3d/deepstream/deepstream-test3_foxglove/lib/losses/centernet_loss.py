import math

import torch
import torch.nn.functional as F
from lib.helpers.decode_helper import _transpose_and_gather_feat
from lib.losses.dim_aware_loss import dim_aware_l1_loss
from lib.losses.focal_loss import focal_loss_cornernet
from lib.losses.uncertainty_loss import laplacian_aleatoric_uncertainty_loss


def judge_nan(value):
    if math.isnan(value):
        return 1e-5
    else:
        return value


def compute_centernet3d_loss(input, target):
    stats_dict = {}

    seg_loss = compute_segmentation_loss(input, target)
    offset2d_loss = compute_offset2d_loss(input, target)
    size2d_loss = compute_size2d_loss(input, target)
    offset3d_loss = compute_offset3d_loss(input, target)
    depth_loss = compute_depth_loss(input, target)
    size3d_loss = compute_size3d_loss(input, target)
    heading_loss = compute_heading_loss(input, target)
    center2kpt_offset_loss = compute_center2kpt_offset_loss(input, target)
    kpt_heatmap_loss = compute_kpt_heatmap_loss(input, target)
    kpt_heatmap_offset_loss = compute_kpt_heatmap_offset_loss(input, target)

    # statistics
    stats_dict['seg'] = judge_nan(seg_loss.item())
    stats_dict['offset2d'] = judge_nan(offset2d_loss.item())
    stats_dict['size2d'] = judge_nan(size2d_loss.item())
    stats_dict['offset3d'] = judge_nan(offset3d_loss.item())
    stats_dict['depth'] = judge_nan(depth_loss.item())
    stats_dict['size3d'] = judge_nan(size3d_loss.item())
    stats_dict['heading'] = judge_nan(heading_loss.item())
    stats_dict['center2kpt_offset'] = judge_nan(center2kpt_offset_loss.item())
    stats_dict['kpt_heatmap'] = judge_nan(kpt_heatmap_loss.item())
    stats_dict['kpt_heatmap_offset'] = judge_nan(
        kpt_heatmap_offset_loss.item())

    total_loss = seg_loss + offset2d_loss + size2d_loss + offset3d_loss + \
        depth_loss + size3d_loss + heading_loss + center2kpt_offset_loss + \
        kpt_heatmap_loss + kpt_heatmap_offset_loss
    return total_loss, stats_dict


def compute_segmentation_loss(input, target):

    # ce = nn.CrossEntropyLoss()
    input['heatmap'] = torch.clamp(input['heatmap'].sigmoid_(),
                                   min=1e-4,
                                   max=1 - 1e-4)
    loss = focal_loss_cornernet(input['heatmap'], target['heatmap'])
    # loss = ce(input['heatmap'], target['heatmap'])
    return loss


def compute_offset2d_loss(input, target):
    # compute offset2d loss

    offset2d_input = extract_input_from_tensor(input['offset_2d'],
                                               target['indices'],
                                               target['mask_2d'])
    offset2d_target = extract_target_from_tensor(target['offset_2d'],
                                                 target['mask_2d'])
    offset2d_loss = F.l1_loss(offset2d_input,
                              offset2d_target,
                              reduction='mean')
    return offset2d_loss


def compute_size2d_loss(input, target):
    # compute size2d loss
    size2d_input = extract_input_from_tensor(input['size_2d'],
                                             target['indices'],
                                             target['mask_2d'])
    size2d_target = extract_target_from_tensor(target['size_2d'],
                                               target['mask_2d'])
    size2d_loss = F.l1_loss(size2d_input, size2d_target, reduction='mean')
    return size2d_loss


def compute_offset3d_loss(input, target):
    offset3d_input = extract_input_from_tensor(input['offset_3d'],
                                               target['indices'],
                                               target['mask_3d'])
    offset3d_target = extract_target_from_tensor(target['offset_3d'],
                                                 target['mask_3d'])
    offset3d_loss = F.l1_loss(offset3d_input,
                              offset3d_target,
                              reduction='mean')
    return offset3d_loss


def compute_size3d_loss(input, target):
    size3d_input = extract_input_from_tensor(input['size_3d'],
                                             target['indices'],
                                             target['mask_3d'])
    size3d_target = extract_target_from_tensor(target['size_3d'],
                                               target['mask_3d'])
    size3d_loss = dim_aware_l1_loss(size3d_input, size3d_target, size3d_target)
    return size3d_loss


def compute_depth_loss(input, target):
    depth_input = extract_input_from_tensor(input['depth'], target['indices'],
                                            target['mask_3d'])
    depth_input, depth_log_variance = depth_input[:, 0:1], depth_input[:, 1:2]
    depth_input = 1. / (depth_input.sigmoid() + 1e-6) - 1.
    depth_target = extract_target_from_tensor(target['depth'],
                                              target['mask_3d'])
    depth_loss = laplacian_aleatoric_uncertainty_loss(depth_input,
                                                      depth_target,
                                                      depth_log_variance)
    return depth_loss


def compute_heading_loss(input, target):
    heading_input = _transpose_and_gather_feat(
        input['heading'], target['indices'])  # B * C * H * W ---> B * K * C
    heading_input = heading_input.view(-1, 24)
    heading_target_cls = target['heading_bin'].view(-1)
    heading_target_res = target['heading_res'].view(-1)
    mask = target['mask_2d'].view(-1)

    # classification loss
    heading_input_cls = heading_input[:, 0:12]
    heading_input_cls, heading_target_cls = heading_input_cls[
        mask > 0], heading_target_cls[mask > 0]
    if mask.sum() > 0:
        cls_loss = F.cross_entropy(heading_input_cls,
                                   heading_target_cls,
                                   reduction='mean')
    else:
        cls_loss = 0.0

    # regression loss
    heading_input_res = heading_input[:, 12:24]
    heading_input_res, heading_target_res = heading_input_res[
        mask > 0], heading_target_res[mask > 0]
    cls_onehot = torch.zeros(heading_target_cls.shape[0],
                             12).cuda().scatter_(dim=1,
                                                 index=heading_target_cls.view(
                                                     -1, 1),
                                                 value=1)
    heading_input_res = torch.sum(heading_input_res * cls_onehot, 1)
    reg_loss = F.l1_loss(heading_input_res,
                         heading_target_res,
                         reduction='mean')
    return cls_loss + reg_loss


def compute_center2kpt_offset_loss(input, target):
    center2kpt_offset_input = extract_input_from_tensor(
        input['center2kpt_offset'], target['indices'], target['mask_3d'])
    center2kpt_offset_target = extract_target_from_tensor(
        target['center2kpt_offset_target'], target['mask_3d'])
    mask_center2kpt_offset = extract_target_from_tensor(
        target['mask_center2kpt_offset'], target['mask_3d'])
    center2kpt_offset_input *= mask_center2kpt_offset
    center2kpt_offset_loss = F.l1_loss(center2kpt_offset_input,
                                       center2kpt_offset_target,
                                       reduction='mean')
    return center2kpt_offset_loss


def compute_kpt_heatmap_loss(input, target):
    input['kpt_heatmap'] = torch.clamp(input['kpt_heatmap'].sigmoid_(),
                                       min=1e-4,
                                       max=1 - 1e-4)
    loss = focal_loss_cornernet(input['kpt_heatmap'],
                                target['kpt_heatmap_target'])
    return loss


def compute_kpt_heatmap_offset_loss(input, target):

    batch_size = input['kpt_heatmap_offset'].shape[0]
    indices_kpt = target['indices_kpt']
    indices_kpt = indices_kpt.reshape(batch_size, -1)

    kpt_heatmap_offset_input = _transpose_and_gather_feat(
        input['kpt_heatmap_offset'], indices_kpt)  # [8, 2, 96, 312]
    kpt_heatmap_offset_input = kpt_heatmap_offset_input.reshape(
        batch_size, 50, 16)
    kpt_heatmap_offset_input = kpt_heatmap_offset_input[target['mask_2d'] > 0]
    kpt_heatmap_offset_target = target['kpt_heatmap_offset_target']
    kpt_heatmap_offset_target = kpt_heatmap_offset_target[target['mask_2d'] >
                                                          0]  # [8, 30, 18]
    mask_kpt_heatmap_offset = extract_target_from_tensor(
        target['mask_kpt_heatmap_offset'], target['mask_2d'])
    kpt_heatmap_offset_input *= mask_kpt_heatmap_offset

    kpt_heatmap_offset_loss = F.l1_loss(kpt_heatmap_offset_input,
                                        kpt_heatmap_offset_target,
                                        reduction='mean')
    return kpt_heatmap_offset_loss


# endregion
# region ==auxiliary functions==


def extract_input_from_tensor(input, ind, mask):
    input = _transpose_and_gather_feat(input, ind)  # B*C*H*W --> B*K*C
    return input[mask > 0]  # B*K*C --> M * C


def extract_target_from_tensor(target, mask):
    return target[mask > 0]


# endregion
