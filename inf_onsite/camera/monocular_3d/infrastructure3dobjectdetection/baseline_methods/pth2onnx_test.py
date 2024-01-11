import torch
import torch.onnx
import onnx
import os
from lib.helpers.model_helper import build_model, build_model_deploy
from lib.helpers.save_helper import load_checkpoint
import argparse
import yaml
import onnxruntime
import numpy as np
import cv2

from torch.utils.data import DataLoader
from lib.datasets.dair.dair_dataset import DAIR_Dataset_MonoCon
from tqdm import tqdm



parser = argparse.ArgumentParser(
    description='End-to-End Monocular 3D Object Detection')
parser.add_argument('--config',
                    dest='config',
                    help='settings of detection in yaml format')
args = parser.parse_args()

def pth_to_onnx(input, checkpoint, onnx_path, input_names=['input'], output_names=['output'], device='cuda'):
    if not onnx_path.endswith('.onnx'):
        print('Warning! The onnx model name is not correct,\
              please give a name that ends with \'.onnx\'!')
        return 0
    cfg = yaml.load(open(args.config, 'r'), Loader=yaml.Loader)
    # model = build_model_deploy().to(device).eval()
    # model.load_state_dict(torch.load(checkpoint), strict=False)
    model = build_model(cfg['model']).to(device).eval()
    
    # torch.onnx.export(model, input, onnx_path, verbose=True, input_names=input_names, output_names=output_names)
    torch.onnx.export(model, input, onnx_path, input_names=input_names, output_names=output_names)
    print("Exporting .pth model to onnx model has been successful!")

def onnx_check(onnx_model_path):
    onnx_model = onnx.load(onnx_model_path)
    onnx.checker.check_model(onnx_model)
    print(onnx.helper.printable_graph(onnx_model.graph))

def onnx_inference(model_path):
    session = onnxruntime.InferenceSession(model_path,
                                           providers=[
                                               ("CUDAExecutionProvider", { 
                                                   "device_id": 0,
                                                   "arena_extend_strategy": "kNextPowerOfTwo",
                                                   "gpu_mem_limit": 4 * 1024 * 1024 * 1024,
                                                   "cudnn_conv_algo_search": "EXHAUSTIVE",
                                                   "do_copy_in_default_stream": True,
                                                   # "cudnn_conv_use_max_workspace": "1" 
                                               }),
                                               "CPUExecutionProvider"   
                                           ])
    # session = onnxruntime.InferenceSession(model_path)
    image = cv2.imread("/home/icv/Edward/inf_sense/test/pic_files/1655783979119666666.jpg")
    image = cv2.resize(image, (1920,1080))
    image = image.transpose(2,0,1)
    image = image[np.newaxis,:].astype(np.float32)
    # print(image.shape)
    # print(image)
    # data = np.random.randn(1, 3, 1080, 1920).astype(np.float32)
    # print(data)
    if True:
         def my_worker_init_fn(worker_id):
            np.random.seed(np.random.get_state()[1][0] + worker_id)
        # with open("/home/icv/Edward/inf_sense/src/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/config/config.yaml","r") as f:
        #     cfg=yaml.load(f,Loader=yaml.Loader)
         cfg = yaml.load(open(args.config, 'r'), Loader=yaml.Loader)
         test_set = DAIR_Dataset_MonoCon(split='test', cfg=cfg['dataset'])
         test_loader = DataLoader(dataset=test_set,
                                 batch_size=1,
                                 num_workers=1,
                                 worker_init_fn=my_worker_init_fn,
                                 shuffle=False,
                                 pin_memory=False,
                                 drop_last=False)
         for (inputs, img, info) in tqdm(test_loader):
            #  print(inputs.shape)
             inputs = inputs.numpy()
             print(inputs.shape)
            #  print(inputs)
             input_name = session.get_inputs()[0].name
            #  print(input_name)
             output_name = session.get_outputs()[0].name
             outputs = session.run([output_name], {input_name: inputs})
             print(np.array(outputs).shape)
             print(outputs)


 
    input_name = session.get_inputs()[0].name
    # print(input_name)
    output_name = session.get_outputs()[0].name
    # print("input name: {}".format(input_name))
    # print(output_name)
    # outputs = session.run([output_name], {input_name: data})
    # outputs = session.run([output_name], {input_name: image})
    outputs = session.run([output_name], {input_name: image})
    # print((outputs))
    # print(np.array(outputs).shape)

   


if __name__ == '__main__':
    os.environ['CUDA_VISIBLE_DEVICES']='0'
    checkpoints_path = '/home/icv/Edward/inf_sense/test/checkpoints/'
    onnx_path = '/home/icv/Edward/inf_sense/test/onnxfiles/'
    checkpoint = checkpoints_path+'checkpoint_epoch_140.pth'
    onnx_path = onnx_path+ '3d_det_140.onnx'
    input = torch.randn(1, 3, 1080, 1920).cuda()
    device = torch.device("cuda" if torch.cuda.is_available() else 'cpu')
    pth_to_onnx(input, checkpoint, onnx_path)
    onnx_check(onnx_path)
    # onnx_inference(onnx_path)

