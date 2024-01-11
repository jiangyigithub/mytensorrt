import torch
import torchvision.models as models
import onnx
import onnxruntime
from lib.helpers.model_helper import build_model, build_model_deploy
import yaml
import argparse
import numpy
import cv2
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
batch_size = 1
input_shape = torch.randn(batch_size, 3, 1080, 1920).cuda()
output_shape = (batch_size, 1000)

onnx_session = onnxruntime.InferenceSession("/home/icv/code/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx",
                                           providers=[
                                               ("CUDAExecutionProvider", { 
                                                   "device_id": 0,
                                                   "arena_extend_strategy": "kNextPowerOfTwo",
                                                   "gpu_mem_limit": 4 * 1024 * 1024 * 1024,
                                                   "cudnn_conv_algo_search": "EXHAUSTIVE",
                                                   "do_copy_in_default_stream": True
                                                   # "cudnn_conv_use_max_workspace": "1" 
                                               }),
                                               "CPUExecutionProvider"   
                                           ])

with open('/home/icv/code/workspaces/inf_onsite/test/trtfiles/3d_det_engine_all_orin_fp16.trt', 'rb') as f:
    engine = runtime.deserialize_cuda_engine(f.read())
    print("Completed creating Engine")
context = engine.create_execution_context()
context.set_binding_shape(0, (1, 3, 1080, 1920))
# Allocate buffers for input and output
inputs_, outputs, bindings, stream = common.allocate_buffers(engine,1)
#(n,c,w,h)
# imagePath = '/home/icv/Edward/inf_sense/test/pic_files/1655783979119666666.jpg'
# t1 = time.time()
onnx_result=[]
for (inputs, img, info) in tqdm(test_loader):
    onnx_result=[]
    input_name = onnx_session.get_inputs()[0].name
    x = inputs  # .cuda()?
    print('*********/',inputs.dtype)
    inputs_n = inputs.numpy()
    y = numpy.around(inputs_n, 4)
    print(inputs_n.dtype)
    for i in range(10):
        output_name = onnx_session.get_outputs()[i].name
        t_t1 = time.time()
        onnx_output = onnx_session.run([output_name], {input_name: y})
        t_t2 = time.time()
        print('*********onnx time*********',t_t2-t_t1)
        onnx_output_ = numpy.squeeze(onnx_output,1)
        onnx_result.append(torch.tensor(onnx_output_))
        # print(numpy.array(onnx_output_).shape)


    # x = input_shape.cpu().numpy()
    t1 = time.time()

    inputX = torch.tensor(numpy.squeeze(x))
    print(inputX.shape)
    inputX = inputX.unsqueeze(0)
    # inputs[0].host = inputX.numpy()
    inputs_[0].host = x.numpy()
    print(type(x.numpy()))
    # print(type(inputs[0].host ))
    trt_outputs = common.do_inference(context, bindings=bindings, inputs=inputs_, outputs=outputs, stream=stream)

    t2 = time.time()
    print('*********trt time*********',t2-t1)
    # for i in range(10):
    #     print(numpy.array(trt_outputs)[i].shape)
        # print(trt_outputs[i])

    # print((trt_outputs))
    # print(f"ONNX output: {list(numpy.array(onnx_output))}")
    # print(numpy.array(onnx_output).shape)
    # print(numpy.array(trt_outputs)[0].shape)
    trt_outputs_0 = torch.tensor(trt_outputs[0].reshape((1,10,270,480)))
    trt_outputs_1 = torch.tensor(trt_outputs[1].reshape((1,2,270,480)))
    trt_outputs_2 = torch.tensor(trt_outputs[2].reshape((1,2,270,480)))
    trt_outputs_3 = torch.tensor(trt_outputs[3].reshape((1,2,270,480)))
    trt_outputs_4 = torch.tensor(trt_outputs[4].reshape((1,2,270,480)))
    trt_outputs_5 = torch.tensor(trt_outputs[5].reshape((1,3,270,480)))
    trt_outputs_6 = torch.tensor(trt_outputs[6].reshape((1,24,270,480)))
    trt_outputs_7 = torch.tensor(trt_outputs[7].reshape((1,16,270,480)))
    trt_outputs_8 = torch.tensor(trt_outputs[8].reshape((1,8,270,480)))
    trt_outputs_9 = torch.tensor(trt_outputs[9].reshape((1,2,270,480)))
    # print(trt_outputs_0.shape)
    keys = ['heatmap', 'offset_2d', 'size_2d', 'depth', 'offset_3d', 'size_3d', 'heading', 'center2kpt_offset', 'kpt_heatmap', 'kpt_heatmap_offset']
    # trt values
    values = [trt_outputs_0, trt_outputs_1, trt_outputs_2, trt_outputs_3, trt_outputs_4, trt_outputs_5, trt_outputs_6, trt_outputs_7, trt_outputs_8, trt_outputs_9]
    # onnx values
    # values = onnx_result   # onnx
    trt_outputs_final = dict(zip(keys,values))
    # for key,value in trt_outputs_final.items():
    #             if key =='heatmap':
    #                 print(key, value)

    dets = extract_dets_from_outputs(outputs=trt_outputs_final,
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
    print('output_dir',output_dir)
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
