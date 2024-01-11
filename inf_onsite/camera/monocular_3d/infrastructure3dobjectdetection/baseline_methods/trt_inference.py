import torch
import cv2
import tensorrt as trt
import numpy as np
 
def trt_version():
   return trt.__version__
 
def torch_device_from_trt(device):
   if device == trt.TensorLocation.DEVICE:
       return torch.device("cuda")
   elif device == trt.TensorLocation.HOST:
       return torch.device("cpu")
   else:
       return TypeError("%s is not supported by torch" % device)
 
def torch_dtype_from_trt(dtype):
   if dtype == trt.int8:
       return torch.int8
   elif trt_version() >= '7.0' and dtype == trt.bool:
       return torch.bool
   elif dtype == trt.int32:
       return torch.int32
   elif dtype == trt.float16:
       return torch.float16
   elif dtype == trt.float32:
       return torch.float32
   else:
       raise TypeError("%s is not supported by torch" % dtype)
 
class TRTModule(torch.nn.Module):
        
    def __init__(self, engine=None, input_names=None, output_names=None):
        super(TRTModule, self).__init__()
        self.engine = engine
        if self.engine is not None:
            self.context = self.engine.create_execution_context()
    
        self.input_names = input_names
        self.output_names = output_names
 
    def forward(self, *inputs):
        batch_size = inputs[0].shape[0]
        bindings = [None] * (len(self.input_names) + len(self.output_names))
        outputs = [None] * len(self.output_names)
        for i, output_name in enumerate(self.output_names):
            idx = self.engine.get_binding_index(output_name) 
            dtype = torch_dtype_from_trt(self.engine.get_binding_dtype(idx)) 
            shape = tuple(self.engine.get_binding_shape(idx)) 
            device = torch_device_from_trt(self.engine.get_location(idx))
            output = torch.empty(size=shape, dtype=dtype, device=device)
            outputs[i] = output
            bindings[idx] = output.data_ptr() 
    
        for i, input_name in enumerate(self.input_names):
            idx = self.engine.get_binding_index(input_name)
            bindings[idx] = inputs[0].contiguous().data_ptr()
    
        self.context.execute_async_v2(bindings, torch.cuda.current_stream().cuda_stream)
    
        outputs = tuple(outputs)
        if len(outputs) == 1:
            outputs = outputs[0] 
            
        return outputs
 
if __name__ == "__main__":
    
    logger = trt.Logger(trt.Logger.INFO)
    with open("/home/icv/code/workspaces/inf_onsite/test/trtfiles/3d_det_engine_all_orin_fp16.trt", "rb") as f, trt.Runtime(logger) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())
    
    for idx in range(engine.num_bindings):
        is_input = engine.binding_is_input(idx)
        name = engine.get_binding_name(idx)
        op_type = engine.get_binding_dtype(idx)
        shape = engine.get_binding_shape(idx)
        print('input id:', idx, ' is input: ', is_input, ' binding name:', name, ' shape:', shape, 'type: ', op_type)
    
    trt_model = TRTModule(engine, ["input"], ["output"])
    
    image = cv2.imread("/home/icv/code/workspaces/inf_onsite/test/picfiles/1655783979119666666.jpg")
    
    image = cv2.resize(image, (1080,1920))
    image = image.transpose(2,0,1)
    img_input = image.astype(np.float32)
    img_input = torch.from_numpy(img_input)
    img_input = img_input.unsqueeze(0)
    img_input = img_input.to('cuda')
    
    result_trt = trt_model(img_input)
    print(result_trt)