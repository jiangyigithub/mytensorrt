import torch
from lib.helpers.model_helper import build_model, build_model_deploy
import yaml
import argparse
import numpy
import time
import tensorrt as trt
import common
import os
from tqdm import tqdm

from lib.helpers.decode_helper import (decode_detections,
                                       extract_dets_from_outputs)
from lib.helpers.utils_helper import create_logger, set_random_seed
from lib.datasets.dair.dair_dataset import DAIR_Dataset_MonoCon
from torch.utils.data import DataLoader

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
runtime = trt.Runtime(TRT_LOGGER)

parser = argparse.ArgumentParser(
    description='End-to-End Monocular 3D Object Detection')
parser.add_argument('--config',
                    dest='config',
                    help='settings of detection in yaml format')
args = parser.parse_args()

def my_worker_init_fn(worker_id):
    numpy.random.seed(numpy.random.get_state()[1][0] + worker_id)
cfg = yaml.load(open(args.config, 'r'), Loader=yaml.Loader)
set_random_seed(cfg.get('random_seed', 444))
test_set = DAIR_Dataset_MonoCon(split='test', cfg=cfg['dataset'])
test_loader = DataLoader(dataset=test_set,
                            batch_size=1,
                            num_workers=1,
                            worker_init_fn=my_worker_init_fn,
                            shuffle=False,
                            pin_memory=False,
                            drop_last=False)
torch.set_grad_enabled(False)
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


input_names = ["input"]
output_names = ["output"]

with open('/home/icv/Edward/inf_sense/test/trtfiles/docker_3d_det_engine.trt', 'rb') as f:
    engine = runtime.deserialize_cuda_engine(f.read())
    print("Completed creating Engine")
context = engine.create_execution_context()
context.set_binding_shape(0, (1, 3, 1080, 1920))
inputs_, outputs, bindings, stream = common.allocate_buffers(engine,1)
for (inputs, img, info) in tqdm(test_loader):
    x = inputs
    # print((x).shape)
    # print(x)
    inputX = torch.tensor(numpy.squeeze(x))
    # print(inputX)
    inputX = inputX.unsqueeze(0)
    inputs_[0].host = x.numpy()
    # print((inputs_[0].host).shape)
    t1 = time.time()
    trt_outputs = common.do_inference(context, bindings=bindings, inputs=inputs_, outputs=outputs, stream=stream)
    t2 = time.time()
    print(t2-t1)

# print((trt_outputs))
trt_outputs_0 = torch.tensor(trt_outputs[0].reshape((1,10,270,480)))
# print(trt_outputs_0)
trt_outputs_1 = torch.tensor(trt_outputs[1].reshape((1,2,270,480)))
trt_outputs_2 = torch.tensor(trt_outputs[2].reshape((1,2,270,480)))
trt_outputs_3 = torch.tensor(trt_outputs[3].reshape((1,2,270,480)))
trt_outputs_4 = torch.tensor(trt_outputs[4].reshape((1,2,270,480)))
trt_outputs_5 = torch.tensor(trt_outputs[5].reshape((1,3,270,480)))
# print(trt_outputs_5)
trt_outputs_6 = torch.tensor(trt_outputs[6].reshape((1,24,270,480)))
trt_outputs_7 = torch.tensor(trt_outputs[7].reshape((1,16,270,480)))
trt_outputs_8 = torch.tensor(trt_outputs[8].reshape((1,8,270,480)))
trt_outputs_9 = torch.tensor(trt_outputs[9].reshape((1,2,270,480)))
keys = ['heatmap', 'offset_2d', 'size_2d', 'depth', 'offset_3d', 'size_3d', 'heading', 'center2kpt_offset', 'kpt_heatmap', 'kpt_heatmap_offset']
# trt values
values = [trt_outputs_0, trt_outputs_1, trt_outputs_2, trt_outputs_3, trt_outputs_4, trt_outputs_5, trt_outputs_6, trt_outputs_7, trt_outputs_8, trt_outputs_9]
trt_outputs_final = dict(zip(keys,values))

dets = extract_dets_from_outputs(outputs=trt_outputs_final,
                                         K=test_loader.dataset.max_objs)
# print("(dets).shape:  ",(dets))
dets = dets.detach().cpu().numpy()
# print("(dets).shape:  ",(dets)[0].shape)
calibs = [
    test_loader.dataset.get_calib(index) for index in info['img_id']
]
# print(len(calibs))
info = {key: val.detach().cpu().numpy() for key, val in info.items()}
cls_mean_size = test_loader.dataset.cls_mean_size

dets = decode_detections(dets=dets,
                            info=info,
                            calibs=calibs,
                            cls_mean_size=cls_mean_size,
                            threshold=cfg['tester']['threshold'])
# print(dets)

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
