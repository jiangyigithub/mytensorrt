```python
import torch
import torch.utils.data
import torchvision
from absl import logging
from pytorch_quantization import nn as quant_nn
from pytorch_quantization import quant_modules

logging.set_verbosity(logging.FATAL)

quant_modules.initialize()

model = torchvision.models.resnet18().cuda()

# 在这里直接进行模型训练就行
# 应该是以fp32模式训练好一个模型，再使用极低的学习率去做训练中量化微调

def export_onnx(model, onnx_filename, batch_onnx):
    model.eval()
    quant_nn.TensorQuantizer.use_fb_fake_quant = True
    opset_version = 13
    print("creating onnx file ... ", onnx_filename)
    dummy_input = torch.randn(batch_onnx, 3, 224, 224, device="cuda")
    torch.onnx.export(model, dummy_input, onnx_filename, verbose=False, opset_version=opset_version, do_constant_folding=True)
    return True
```

