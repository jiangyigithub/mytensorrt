import torch
import torchvision.models as models
import onnx
import onnxruntime
from lib.helpers.model_helper import build_model, build_model_deploy
from lib.helpers.save_helper import load_checkpoint
import datetime
import yaml
import argparse
import numpy
from lib.helpers.utils_helper import create_logger, set_random_seed
from lib.datasets.dair.dair_dataset import DAIR_Dataset_MonoCon
from torch.utils.data import DataLoader
from lib.helpers.decode_helper import (decode_detections,
                                       extract_dets_from_outputs)
from tqdm import tqdm
import os


parser = argparse.ArgumentParser(
    description='End-to-End Monocular 3D Object Detection')
parser.add_argument('--config',
                    dest='config',
                    help='settings of detection in yaml format')
args = parser.parse_args()
cfg = yaml.load(open(args.config, 'r'), Loader=yaml.Loader)
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


# model = models.resnet18(pretrained=True)
log_file = 'test.log.%s' % datetime.datetime.now().strftime(
        '%Y%m%d_%H%M%S')
logger = create_logger(log_file)
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
model = build_model(cfg['model']).to('cuda').eval()
load_checkpoint(model=model,
                    optimizer=None,
                    filename=cfg['tester']['checkpoint'],
                    map_location=device,
                    logger=logger)
model.eval()
input_names = ["input"]
# output_names = ["output"] #heatmap; offset_2d; size_2d; depth; offset_3d; size_3d; heading; center2kpt_offset; kpt_heatmap; kpt_heatmap_offset
output_names = ['heatmap','offset_2d','size_2d','depth','offset_3d','size_3d','heading','center2kpt_offset','kpt_heatmap','kpt_heatmap_offset']
batch_size = 1
input_shape = torch.randn(batch_size, 3, 1080, 1920).cuda()
output_shape = (batch_size, 1000)
torch.onnx.export(model,  
                  input_shape,  
                  "/home/icv/Edward/inf_sense/test/data/3d_det_200.onnx",  
                  input_names=input_names,
                  output_names=output_names,  
                  dynamic_axes={input_names[0]: {0: "batch_size"}, output_names[0]: {0: "batch_size"}}  
                  )
onnx_model = onnx.load("/home/icv/Edward/inf_sense/test/data/3d_det_200.onnx")
onnx_model_graph = onnx_model.graph
onnx_session = onnxruntime.InferenceSession("/home/icv/Edward/inf_sense/test/data/3d_det_200.onnx",
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

# print(input_shape.dtype)
# img analysis
for (inputs, img, info) in tqdm(test_loader):
    # print(inputs.dtype)
    inputs = inputs.cuda()
    x = numpy.around(inputs.cpu().numpy(),4)
    input_name = onnx_session.get_inputs()[0].name
    output_name = onnx_session.get_outputs()[1].name
    onnx_output = onnx_session.run([output_name], {input_name: x})
    pt_output = model(inputs)
    print(f"PyTorch output: {pt_output['offset_2d'][0, :2]}")
    print(f"ONNX output: {numpy.array(onnx_output)[0, :2]}")
# end img

# randn
# x = numpy.around(input_shape.cpu().numpy(),4)
# input_name = onnx_session.get_inputs()[0].name
# output_name = onnx_session.get_outputs()[1].name
# onnx_output = onnx_session.run([output_name], {input_name: x})
# pt_output = model(input_shape)
# end randn
# print(x)
# print(f"PyTorch output: {pt_output['offset_2d'][0, :2]}")
# print(f"ONNX output: {numpy.array(onnx_output)[0, :2]}")
# print(numpy.array(onnx_output).shape)
mean_error_per = numpy.divide(numpy.array(onnx_output) - pt_output['offset_2d'].cpu().detach().numpy(), pt_output['offset_2d'].cpu().detach().numpy())
print('mean_error_percentage: ',100*mean_error_per.mean(),'%')
# for key,values in pt_output.items():
#             # if key =='heatmap':
#             print(key, values.shape)

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

onnx_result=[]
for (inputs, img, info) in tqdm(test_loader):
    onnx_result=[]
    input_name = onnx_session.get_inputs()[0].name
    y = numpy.around(inputs.numpy(), 4)
    for i in range(10):
        output_name = onnx_session.get_outputs()[i].name
        onnx_output = onnx_session.run([output_name], {input_name: y})
        onnx_output_ = numpy.squeeze(onnx_output,1)
        onnx_result.append(torch.tensor(onnx_output_))
        # print(numpy.array(onnx_output_).shape)
    inputs = inputs.cuda()
    pt_output = model(inputs)

for (inputs, img, info) in tqdm(test_loader):
    # print(inputs.dtype)
    inputs = inputs.cuda()
    pt_output = model(inputs)

keys = ['heatmap', 'offset_2d', 'size_2d', 'depth', 'offset_3d', 'size_3d', 'heading', 'center2kpt_offset', 'kpt_heatmap', 'kpt_heatmap_offset']
# onnx values
values = onnx_result   # onnx
outputs_final = dict(zip(keys,values))
# pth values
# outputs_final = pt_output
# for key,value in trt_outputs_final.items():
#             if key =='heatmap':
#                 print(key, value.shape)

dets = extract_dets_from_outputs(outputs=outputs_final,
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
            f'{round(x0,3)} {round(y0,3)} {round(x1,3)} {round(y1,3)} ' + \
            f'{round(_h,3)} {round(_w,3)} {round(_l,3)} {round(x,3)} {round(y,3)} {round(z,3)} {round(ry,3)} {round(s,3)}\n'
        f.write(res)
