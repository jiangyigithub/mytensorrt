import torch
import torchvision.models as models
import onnx
import onnxruntime
model = models.resnet18(pretrained=True)
model.eval()
input_names = ["input"]
output_names = ["output"]
batch_size = 1
input_shape = (batch_size, 3, 224, 224)
output_shape = (batch_size, 1000)
torch.onnx.export(model,  
                  torch.randn(input_shape), 
                  "/home/icv/Edward/inf_sense/test/data/resnet18.onnx",  
                  input_names=input_names, 
                  output_names=output_names, 
                  dynamic_axes={input_names[0]: {0: "batch_size"}, output_names[0]: {0: "batch_size"}}  
                  )
onnx_model = onnx.load("/home/icv/Edward/inf_sense/test/data/resnet18.onnx")
onnx_model_graph = onnx_model.graph
# onnx_session = onnxruntime.InferenceSession(onnx_model.SerializeToString())
onnx_session = onnxruntime.InferenceSession("/home/icv/Edward/inf_sense/test/data/resnet18.onnx",
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
x = torch.randn(input_shape).numpy()
onnx_output = onnx_session.run(output_names, {input_names[0]: x})[0]
print(f"PyTorch output: {model(torch.from_numpy(x)).detach().numpy()[0, :5]}")
print(f"ONNX output: {onnx_output[0, :5]}")