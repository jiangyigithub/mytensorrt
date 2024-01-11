```python
import torch
import torch.profiler
import torch.utils.data
import torchvision
from torch2trt import torch2trt
from tqdm import tqdm

SAMPLES = [torch.zeros(1, 3, 224, 224) for _ in range(1024)]
MODEL = torchvision.models.resnet18().cuda().eval()
FP16_MODE = True

# pytorch 执行网络的推理, 体验一下速度
for sample in tqdm(SAMPLES, desc="Torch Executing"):
    MODEL.forward(sample.cuda())

# torch2trt 包裹 torch 模型, 会在原始模型上进行图优化, 但只能用于推理了, 得到的模型依然是 torch.nn.Module 模型
model_trt = torch2trt(MODEL, [sample.cuda()], fp16_mode=FP16_MODE)
for sample in tqdm(SAMPLES, desc="Trt Executing"):
    model_trt.forward(sample.cuda())
print(isinstance(model_trt, torch.nn.Module))

# 记录 model_trt 执行推理的相关信息: GPU利用率能高达80%
with torch.profiler.profile(
    activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA],
    schedule=torch.profiler.schedule(wait=2, warmup=1, active=7),
    on_trace_ready=torch.profiler.tensorboard_trace_handler("torch2trt_log")
) as p:
    for _ in range(100):
        model_trt.forward(SAMPLES[0].cuda())
        p.step()

# 记录 model_torch 执行推理的相关信息: GPU利用率仅有20%
with torch.profiler.profile(
    activities=[torch.profiler.ProfilerActivity.CPU, torch.profiler.ProfilerActivity.CUDA],
    schedule=torch.profiler.schedule(wait=2, warmup=1, active=7),
    on_trace_ready=torch.profiler.tensorboard_trace_handler("torch2trt_log")
) as p:
    for _ in range(100):
        MODEL.forward(SAMPLES[0].cuda())
        p.step()
```

