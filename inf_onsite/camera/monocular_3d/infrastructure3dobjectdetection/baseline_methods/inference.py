import argparse
import datetime
import os

import numpy as np
import torch
import yaml
from lib.datasets.dair.dair_dataset import DAIR_Dataset_MonoCon
from lib.helpers.decode_helper import (decode_detections,
                                       extract_dets_from_outputs)
from lib.helpers.model_helper import build_model
from lib.helpers.save_helper import load_checkpoint
from lib.helpers.utils_helper import create_logger, set_random_seed
from torch.utils.data import DataLoader
from tqdm import tqdm
from lib.helpers.depth_geometry import projection, kdtree
import time

parser = argparse.ArgumentParser(
    description='End-to-End Monocular 3D Object Detection')
parser.add_argument('--config',
                    dest='config',
                    help='settings of detection in yaml format')
args = parser.parse_args()

# init datasets and dataloaders


def main():
    def my_worker_init_fn(worker_id):
        np.random.seed(np.random.get_state()[1][0] + worker_id)

    cfg = yaml.load(open(args.config, 'r'), Loader=yaml.Loader)

    camera_pose_info =[[-1.740657, 0.030855, 0.029865],[20.169886, -14.505331, 6.657059]]
    projection.projection_init(camera_pose_info)

    set_random_seed(cfg.get('random_seed', 444))
    log_file = 'train.log.%s' % datetime.datetime.now().strftime(
        '%Y%m%d_%H%M%S')
    test_set = DAIR_Dataset_MonoCon(split='test', cfg=cfg['dataset'])
    test_loader = DataLoader(dataset=test_set,
                             batch_size=1,
                             num_workers=1,
                             worker_init_fn=my_worker_init_fn,
                             shuffle=False,
                             pin_memory=False,
                             drop_last=False)

    log_file = 'test.log.%s' % datetime.datetime.now().strftime(
        '%Y%m%d_%H%M%S')
    logger = create_logger(log_file)
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    # device = torch.device('cpu')
    model = build_model(cfg['model']).to(device).eval()
    print(cfg['tester']['checkpoint'])
    load_checkpoint(model=model,
                    optimizer=None,
                    filename=cfg['tester']['checkpoint'],
                    map_location=device,
                    logger=logger)
    torch.set_grad_enabled(False)

    # TODO: check id tpye
    # DAIR cls ID
    id2cls = {
        0: 'car',
        1: 'van',
        2: 'truck',
        3: 'trailer',
        4: 'bus',
        5: 'pedestrian',
        6: 'cyclist',
        7: 'motorcyclist',
        8: 'barrow',
        9: 'tricyclist'
    }
    output_dir = cfg['tester']['save_dir']
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    batch_idx = 0

    # main loop for infrance
    # print(test_loader)
    for (inputs, img, info) in tqdm(test_loader):
        # print(inputs.shape)
        # print(inputs)
        inputs = inputs.to(device)
        # print(inputs)
        t1 = time.time()
        outputs = model(inputs)
        t2 = time.time()
        # print(t2 - t1)
        # print(outputs.keys())
        # for key,values in outputs.items():
        #     if key =='heatmap':
        #         print(key, values.shape)

        dets = extract_dets_from_outputs(outputs=outputs,
                                         K=test_loader.dataset.max_objs)
        # print(dets)
        dets = dets.detach().cpu().numpy()
        
        calibs = [
            test_loader.dataset.get_calib(index) for index in info['img_id']
        ]
        info = {key: val.detach().cpu().numpy() for key, val in info.items()}
        cls_mean_size = test_loader.dataset.cls_mean_size

        dets = decode_detections(dets=dets,
                                 info=info,
                                 calibs=calibs,
                                 cls_mean_size=cls_mean_size,
                                 threshold=cfg['tester']['threshold'])

        obj = dets[info['img_id'][0]]
        output_file_name = os.path.join(output_dir,
                                        '%06d.txt' % info['img_id'][0])
        with open(output_file_name, 'w') as f:
            for o in obj:
                cls = id2cls[o[0]]
                x0, y0, x1, y1 = o[3:7]
                _h, _w, _l = o[7:10]
                x, y, z = o[10:13]
                ry, s = o[13:15]
                res = f'{cls} {0} {0} ' + \
                    f'{x0} {y0} {x1} {y1} ' + \
                    f'{_h} {_w} {_l} {x} {y} {z} {ry} {s}\n'
                f.write(res)
        batch_idx += 1
    logger.info(f'==> Infrance Finished, results saved to : \n {output_dir}')


if __name__ == '__main__':
    main()
