import numpy as np
import torch
import torch.nn as nn
from lib.datasets.utils import class2angle
from lib.helpers.depth_geometry import projection, kdtree


def decode_detections(dets, info, calibs, cls_mean_size, threshold):
    '''
    NOTE: THIS IS A NUMPY FUNCTION
    input: dets, numpy array, shape in [batch x max_dets x dim]
    input: img_info, dict, necessary information of input images
    input: calibs, corresponding calibs for the input batch
    output: [(cls) (0) (0) (x0 y0 x1 y1) (hwl)]
    '''

    depth_geometry_open = False
    # print(calibs)
    # print(dets.shape)

    results = {}
    for i in range(dets.shape[0]):  # batch
        preds = []
        for j in range(dets.shape[1]):  # max_dets
            cls_id = int(dets[i, j, 0])
            score = dets[i, j, 1]
            if score < threshold:
                continue

            # 2d bboxs decoding
            x = dets[i, j, 2] * info['bbox_downsample_ratio'][i][0]
            y = dets[i, j, 3] * info['bbox_downsample_ratio'][i][1]
            w = dets[i, j, 4] * info['bbox_downsample_ratio'][i][0]
            h = dets[i, j, 5] * info['bbox_downsample_ratio'][i][1]
            bbox = [x - w / 2, y - h / 2, x + w / 2, y + h / 2]

            # 3d bboxs decoding
            # depth decoding
            depth = dets[i, j, 6]

            # dimensions decoding
            dimensions = dets[i, j, 31:34]
            # dimensions += cls_mean_size[int(cls_id)]

            # positions decoding
            x3d = dets[i, j, 34] * info['bbox_downsample_ratio'][i][0]
            y3d = dets[i, j, 35] * info['bbox_downsample_ratio'][i][1]
            # print(x3d,"     ",y3d)

            # heading angle decoding
            alpha = get_heading_angle(dets[i, j, 7:31])
            ry = calibs[i].alpha2ry(alpha, x3d)

            # depth calculated via geometry relationships
            if depth_geometry_open :
                # calculate depth
                # obj_info = [h, ry, x3d, y3d]
                # x3d_b, y3d_b = projection.search_obj_bottom_point(obj_info)
                # depth = projection.get_Zc(x3d_b,y3d_b)
                # print(x3d,y3d, dimensions)
                depth = projection.find_point(x3d,y3d,dimensions[0]/2-0.4)
            locations = calibs[i].img_to_rect(x3d, y3d, depth).reshape(-1)

            # print(score,"  00000 ")
            score = score * dets[i, j, -1]

            # 0: cls 1:t=0 2:o=0 3-6:bbox 7-9:hwl 10-12:xyz 13:ry 14:score
            preds.append([cls_id, 0, 0] + bbox + dimensions.tolist() +
                         locations.tolist() + [ry, score])
        results[info['img_id'][i]] = preds
    return results


def extract_dets_from_outputs(outputs, K=50):
    # get src outputs
    heatmap = outputs['heatmap']
    heading = outputs['heading']
    depth = outputs['depth'][:, 0:1, :, :]
    sigma = outputs['depth'][:, 1:2, :, :]
    size_3d = outputs['size_3d']
    offset_3d = outputs['offset_3d']
    size_2d = outputs['size_2d']
    offset_2d = outputs['offset_2d']

    heatmap = torch.clamp(heatmap.sigmoid_(), min=1e-4, max=1 - 1e-4)
    depth = 1. / (depth.sigmoid() + 1e-6) - 1.
    sigma = torch.exp(-sigma)

    batch, channel, height, width = heatmap.size()  # get shape

    # perform nms on heatmaps
    heatmap = _nms(heatmap)
    scores, inds, cls_ids, xs, ys = _topk(heatmap, K=K)

    offset_2d = _transpose_and_gather_feat(offset_2d, inds)
    offset_2d = offset_2d.view(batch, K, 2)
    xs2d = xs.view(batch, K, 1) + offset_2d[:, :, 0:1]
    ys2d = ys.view(batch, K, 1) + offset_2d[:, :, 1:2]

    offset_3d = _transpose_and_gather_feat(offset_3d, inds)
    offset_3d = offset_3d.view(batch, K, 2)
    xs3d = xs.view(batch, K, 1) + offset_3d[:, :, 0:1]
    ys3d = ys.view(batch, K, 1) + offset_3d[:, :, 1:2]

    heading = _transpose_and_gather_feat(heading, inds)
    heading = heading.view(batch, K, 24)
    depth = _transpose_and_gather_feat(depth, inds)
    depth = depth.view(batch, K, 1)
    sigma = _transpose_and_gather_feat(sigma, inds)
    sigma = sigma.view(batch, K, 1)
    size_3d = _transpose_and_gather_feat(size_3d, inds)
    size_3d = size_3d.view(batch, K, 3)
    cls_ids = cls_ids.view(batch, K, 1).float()
    scores = scores.view(batch, K, 1)

    # check shape
    xs2d = xs2d.view(batch, K, 1)
    ys2d = ys2d.view(batch, K, 1)
    xs3d = xs3d.view(batch, K, 1)
    ys3d = ys3d.view(batch, K, 1)

    size_2d = _transpose_and_gather_feat(size_2d, inds)
    size_2d = size_2d.view(batch, K, 2)

    detections = torch.cat([
        cls_ids, scores, xs2d, ys2d, size_2d, depth, heading, size_3d, xs3d,
        ys3d, sigma
    ],
                           dim=2)
    return detections


# region auxiliary functions


def _nms(heatmap, kernel=3):
    padding = (kernel - 1) // 2
    heatmapmax = nn.functional.max_pool2d(heatmap, (kernel, kernel),
                                          stride=1,
                                          padding=padding)
    keep = (heatmapmax == heatmap).float()
    return heatmap * keep


def _topk(heatmap, K=50):
    batch, cat, height, width = heatmap.size()

    # batch * cls_ids * 50
    topk_scores, topk_inds = torch.topk(heatmap.view(batch, cat, -1), K)

    topk_inds = topk_inds % (height * width)
    topk_ys = torch.div(topk_inds, width, rounding_mode='trunc').float()
    topk_xs = (topk_inds % width).int().float()

    # batch * cls_ids * 50
    topk_score, topk_ind = torch.topk(topk_scores.view(batch, -1), K)
    topk_cls_ids = torch.div(topk_ind, K, rounding_mode='trunc')
    topk_inds = _gather_feat(topk_inds.view(batch, -1, 1),
                             topk_ind).view(batch, K)
    topk_ys = _gather_feat(topk_ys.view(batch, -1, 1), topk_ind).view(batch, K)
    topk_xs = _gather_feat(topk_xs.view(batch, -1, 1), topk_ind).view(batch, K)

    return topk_score, topk_inds, topk_cls_ids, topk_xs, topk_ys


def _gather_feat(feat, ind, mask=None):
    '''
    Args:
        feat: tensor shaped in B * (H*W) * C
        ind:  tensor shaped in B * K (default: 50)
        mask: tensor shaped in B * K (default: 50)

    Returns: tensor shaped in B * K or B * sum(mask)
    '''
    dim = feat.size(2)  # get channel dim
    ind = ind.unsqueeze(2).expand(
        ind.size(0), ind.size(1),
        dim)  # B*len(ind) --> B*len(ind)*1 --> B*len(ind)*C
    feat = feat.gather(1, ind)  # B*(HW)*C ---> B*K*C
    if mask is not None:
        mask = mask.unsqueeze(2).expand_as(feat)  # B*50 ---> B*K*1 --> B*K*C
        feat = feat[mask]
        feat = feat.view(-1, dim)
    return feat


def _transpose_and_gather_feat(feat, ind):
    '''
    Args:
        feat: feature maps shaped in B * C * H * W
        ind: indices tensor shaped in B * K
    Returns:
    '''
    feat = feat.permute(0, 2, 3,
                        1).contiguous()  # B * C * H * W ---> B * H * W * C
    feat = feat.view(feat.size(0), -1,
                     feat.size(3))  # B * H * W * C ---> B * (H*W) * C
    feat = _gather_feat(feat, ind)  # B * len(ind) * C
    return feat


def get_heading_angle(heading):
    heading_bin, heading_res = heading[0:12], heading[12:24]
    cls = np.argmax(heading_bin)
    res = heading_res[cls]
    return class2angle(cls, res, to_label_format=True)


# endregion
