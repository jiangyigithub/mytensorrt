## repo --> https://github.com/jiangyigithub/mytensorrt

## 硬盘分区，linux文件系统
https://phoenixnap.com/kb/linux-format-disk
1. get device name
sudo fdisk -l
Disk /dev/nvme0n1: 1.84 TiB, 2000398934016 bytes, 3907029168 sectors
-->
/dev/nvme0n1

2. formatting Disk Partition in Linux
sudo mkfs -t ext4 /dev/nvme0n1

3. mounting the disk partition
sudo mount -t auto /dev/nvme0n1 /home/icv/code/

4. verify the file system 
lsblk -f

5. change the permission of a file or directory
sudo chmod 777 /home/icv/code/

6. auto mount after rebooting(need get UUID)
sudo cp /etc/fstab /etc/fstab.bak
lsblk -f
sudo echo UUID=75bb84f1-4e3e-4554-9edd-eaad26966605 /home/icv/code/ ext4 defaults 0 2 >> /etc/fstab

## docker environment + ROS2
### pull docker image and create docker container
sudo docker pull nvcr.io/nvidia/deepstream:6.3-triton-multiarch
sudo docker images
sudo docker run -it -d -v /home/icv/workspaces/inf_onsite_docker/:/home/icv/workspaces/inf_onsite_docker/ --name inf-docker nvcr.io/nvidia/deepstream:6.3-triton-multiarch
sudo docker ps -a
docker start inf-docker
sudo docker exec -it inf-docker /bin/bash

### operate docker container
sudo docker stop 65415e0d4cbf
docker rm 65415e0d4cbf

### operate docker image
sudo docker images
sudo docker rmi adb2e0a9b1ee

### docker base and docker file
https://catalog.ngc.nvidia.com/orgs/nvidia/containers/deepstream-l4t/tags
https://github.com/NVIDIA-AI-IOT/deepstream_dockers/blob/main/jetson/ubuntu_base_devel/Dockerfile
https://github.com/NVIDIA-AI-IOT/ros2_deepstream/blob/main/docker/dockerfile.ros.eloquent.deepstream

### docker file example 
https://sourcecode.socialcoding.bosch.com/projects/RIX3/repos/unifiedvisibilitysw/browse
```Dockerfile
FROM zombbie/cuda11.1-cudnn8-ubuntu20.04:v1.0


ENV ACCEPT_EULA=Y

RUN apt-get update &&\
    apt-get install -y unzip &&\
    apt-get install -y lsb-core &&\
    apt-get install -y curl &&\
    apt-get autoclean &&\
    rm -rf /var/lib/apt/lists/*

# install ros
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
RUN sh -c '. /etc/lsb-release && echo "deb http:/mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &&\
    apt-get -y update &&\
    apt-get -y --no-install-recommends install ros-noetic-desktop &&\
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc &&\
    apt-get autoclean &&\
    rm -rf /var/lib/apt/lists/*

RUN apt-get -y update &&\ 
    pip3 install rosdepc &&\
    # rosdepc init && rosdepc update &&\
    apt-get -y install  python3-rosinstall python3-rosinstall-generator python3-wstool build-essential &&\
    apt-get install -y ros-noetic-octomap-ros &&\
    apt-get install -y ros-noetic-grid-map &&\
    apt-get autoclean &&\
    rm -rf /var/lib/apt/lists/*

# setting libtorch
WORKDIR /catkin_ws/src/unifiedvisibilitysw

RUN wget -q https://download.pytorch.org/libtorch/cu111/libtorch-cxx11-abi-shared-with-deps-1.9.1%2Bcu111.zip && \
    unzip libtorch-cxx11-abi-shared-with-deps-1.9.1+cu111.zip && \
    rm libtorch-cxx11-abi-shared-with-deps-1.9.1+cu111.zip

RUN mv ./libtorch /usr/local/

ENV LD_LIBRARY_PATH=/usr/local/libtorch:$LD_LIBRARY_PATH \
     PATH=/usr/local/libtorch:$PATH

COPY ./ ./

RUN chmod +x entrypoints.sh

ENTRYPOINT ["/catkin_ws/src/unifiedvisibilitysw/entrypoints.sh"]
```

```bash  entrypoints.sh
#!/bin/bash

cd /catkin_ws/

source ~/.bashrc
source /opt/ros/noetic/setup.bash

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release

source /catkin_ws/devel/setup.bash

# 运行 catkin_ws 中的 launch 节点（示例命令，需要替换为实际的启动命令）
roslaunch univiz  univiz.launch
```

## ROS2 env
http://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
sudo update-locale LANG=C.UTF-8
sudo sh -c 'echo "deb [arch=arm64,armhf] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
Or, you can use the new keyserver method:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update && sudo apt install -y build-essential cmake git python3-colcon-common-extensions python3-pip

sudo apt install -y \
  python3-argcomplete \
  python3-rosdep \
  python3-vcstool \
  python3-yaml \
  python3-pyparsing \
  libpython3-dev

sudo apt update
sudo apt install -y ros-foxy-desktop
sudo rosdep init
rosdep update

source /opt/ros/foxy/setup.bash

You may want to add this line to your shell configuration file (e.g., ~/.bashrc or ~/.zshrc) to automatically source it when you open a new terminal.


### linux scp to copy file
sudo chmod 777 /home/jan5szh/workspaces/inf_onsite/
sudo scp -r /home/jan5szh/workspaces/inf_onsite icv@192.168.99.130:/home/icv/workspaces/inf_onsite

## NVIDIA CUDA CUDNN TENSORRT
https://www.bilibili.com/video/BV1CN411N7Dd/?spm_id_from=333.337.search-card.all.click&vd_source=0d7a659e0c3fd86bc699b9150fa1cbbb
https://pan.quark.cn/s/773e64b5427e#/list/share/68ac90b4824b48b68dd93db8fc59a01f-nvidia*101devops
https://www.youtube.com/watch?v=LUxyNyCl4ro

### CUDA
export PATH=/usr/local/cuda-11/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-11

source /etc/profile
nvcc -V

### CUDNN
sudo cp /usr/include/cudnn.h /usr/local/cuda-11/include
sudo cp /usr/lib/aarch64-linux-gnu/libcudnn* /usr/local/cuda-11/lib64/
sudo chmod 777 /usr/local/cuda-11/include/cudnn.h /usr/local/cuda-11/lib64/libcudnn*

cd /usr/local/cuda-11/lib64
ll libcudnn*
sudo ldconfig

sudo cp -r /usr/src/cudnn_samples_v8/ /tmp/
cd /tmp/cudnn_samples_v8/conv_sample/
sudo make clean
sudo make
./conv_sample

### cuda runtime
g++ -o cudnn_softmax -I /usr/local/cuda/include     cudnn_softmax.cpp     -L /usr/local/cuda/lib64 -lcudnn -lcudart
`c_cpp_properties.json`
    "includePath": [
        "${workspaceFolder}/**",
        "/usr/local/cuda/include/**"
    ],

### TensorRT
sudo ln -s /usr/src/tensorrt/bin/trtexec /usr/bin/trtexec
sudo chmod -R 777 /usr/src/tensorrt
cd /usr/src/tensorrt/samples/sampleINT8API/
sudo make
cd ../../bin/
./sample_int8_api

### conda
https://github.com/conda-forge/miniforge
https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh


### trtexec
sudo trtexec --onnx=/home/icv/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx --saveEngine=/home/icv/workspaces/inf_onsite/test/trtfiles/3d_det_engine_all_orin_fp16.trt --fp16
sudo chmod 777 /home/icv/workspaces/inf_onsite/test/trtfiles/3d_det_engine_all_orin.trt
/home/icv/workspaces/inf_onsite/camera/monocular_3d/deepstream/deepstream-test3/dstest3_pgie_config.txt
model-engine-file = /home/icv-ai/workspaces/inf_sense/test/trtfiles/3d_det_engine_all.trt

### clean cmake build
rm -rf CMakeCache.txt CMakeFiles/

### Tensorrt源码安装和DEB文件安装
https://zhuanlan.zhihu.com/p/379287312
DEB安装--> https://developer.nvidia.com/nvidia-tensorrt-8x-download
源码安装-->https://github.com/NVIDIA/TensorRT/tree/release/8.5


### VPN todesk
osd-vpn-connect -x
todesk --> https://zhuanlan.zhihu.com/p/632933198

### `tensorrt --> import tensorrt as trt` tensorrt的python调用(issue:C++调用可以，但python不行)
tensorrt python --> do not activate conda

### `pycuda` python CUDA
https://medium.com/dropout-analytics/pycuda-on-jetson-nano-7990decab299
https://pypi.org/project/pycuda/#files
https://zhuanlan.zhihu.com/p/344027329
pip install pytools
tar zxvf pycuda-2019.1.2.tar.gz    
cd pycuda-2019.1.2/  
python3 configure.py --cuda-root=/usr/local/cuda-11
sudo python3 setup.py install

### `cuda-python` python CUDA
https://pypi.org/project/cuda-python/#files
https://forums.developer.nvidia.com/t/cuda-python-vs-pycuda/216751

### 需要的库汇总和版本对应
cuda cudnn tensorrt onnx onnxruntime pytorch torchvision
JetPack 是 NVIDIA 提供的一套软件开发工具包，主要用于支持 NVIDIA 的 GPU 平台上的深度学习和计算机视觉开发。JetPack 中包含了一系列的软件和工具，具体部分(https://developer.nvidia.com/embedded/jetpack)：
CUDA Toolkit: 用于 GPU 加速计算的核心工具包，包括 CUDA 库、编译器、驱动等。
cuDNN: NVIDIA 提供的深度学习库，用于加速深度神经网络的训练和推理。
TensorRT: 用于高性能深度学习推理的库，支持优化和加速深度学习模型。
NVIDIA VisionWorks: 一个计算机视觉库，提供了一系列优化的图像处理和计算机视觉算法。
NVIDIA Nsight Systems 和 Nsight Graphics: 用于系统和图形分析的工具，可用于调试和优化 CUDA 和图形应用程序。
NVIDIA NvMedia 和 NvAVParsers: 用于多媒体处理的库，支持音频和视频处理。
OpenCV: 一个开源计算机视觉库，用于图像处理和计算机视觉应用。
Linux For Tegra (L4T): 一个基于 Ubuntu 的 Linux 发行版，为 NVIDIA GPU 提供支持，并包含了一些必要的驱动程序和工具。
Jetson.GPIO: 用于 Jetson 系列开发板的 GPIO 控制库。
Jetson Multimedia API: 提供了用于图像和视频处理的 API。

### `torchvision` 安装
基于JetPack离线安装torch和编译安装torchvision（arm架构） -->https://zhuanlan.zhihu.com/p/661173952
source code compile into a wheel file

### copy python package into conda environment
cp -r /usr/lib/python3.8/dist-packages/* /home/icv/miniforge3/envs/trt/lib/python3.8/site-packages/
cp -r /home/icv/.local/lib/python3.8/site-packages/* cp -r /usr/lib/python3.8/dist-packages/
`conda config --set auto_activate_base false`

### install CUDA CUDNN Tensorrt without jetpack
https://zhuanlan.zhihu.com/p/471726587


### pip 源设为国内
pip -i source_local
阿里云镜像：-i https://mirrors.aliyun.com/pypi/simple/

### IPC IP
/media/icv-ai/17aed8c1-fa27-4be1-bda0-25bb0361f3ec/home/icv/Edward/04_tools/ros2_ws/src
192.168.99.158

### deepstream DEB 文件安装
`DeepStream` 版本对应:DeepStream SDK 6.3需要JetPack 5.1
https://catalog.ngc.nvidia.com/orgs/nvidia/resources/deepstream/files?version=6.3

### pyds wheel安装  --> pybind
https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/tag/v1.1.8

### 修改内部头文件
/home/icv/workspaces/nvdsinfer_wxy.h
line:158
/**
 * Holds information about one parsed object from a detector's output.wxy
 */
typedef struct
{
  unsigned int classId;
  float Scores;
  float Xs2d;
  float Ys2d;
  float Xs3d;
  float Ys3d;
  float Size_2d[2];
  float Size_3d[3];
  float Depth;
  float Heading[24];
  float Sigma;
} NvDsInferObjectDetectionInfo_4decode;

### pip python package安装位置
pip show torch

### 修改cmake查找 torch库
# set(Torch_DIR /home/icv/Edward/deepstream/common/libtorch/share/cmake/Torch) 
set(Torch_DIR /home/icv/.local/lib/python3.8/site-packages/torch/share/cmake/Torch) 

### 内部库路经
`depth_geometry`
sys.path.append('/home/icv/workspaces/inf_onsite/camera/monocular_3d/deepstream/deepstream-test3/lib/helpers')

### launch deepstream app
source /home/icv/workspaces/ros2_ws/install/perception_kit_msgs/share/perception_kit_msgs/local_setup.bash
source /home/icv/workspaces/ros2_ws/install/local_setup.bash 
source /home/icv/workspaces/ros2_ws/install/adma_msgs/share/adma_msgs/local_setup.bash 
cd /home/icv/workspaces/inf_onsite/camera/monocular_3d/deepstream/deepstream-test3
python3.8 ds_3d_det_ros_main.py -i  file:////home/icv/workspaces/wheel/test_video.mp4

### convert onnx to trt
sudo trtexec --onnx=/home/icv/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx --saveEngine=/home/icv/workspaces/inf_onsite/test/trtfiles/3d_det_engine_all_orin_fp16.trt --fp16
0:51:16] [I] === Trace details ===
[12/28/2023-10:51:16] [I] Trace averages of 10 runs:
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 40.2957 ms - Host latency: 42.369 ms (enqueue 40.1544 ms)
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 42.2684 ms - Host latency: 44.3462 ms (enqueue 42.5364 ms)
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 39.5355 ms - Host latency: 41.6029 ms (enqueue 39.448 ms)
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 40.277 ms - Host latency: 42.3513 ms (enqueue 40.1736 ms)
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 40.8254 ms - Host latency: 42.9138 ms (enqueue 40.6084 ms)
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 40.255 ms - Host latency: 42.3322 ms (enqueue 40.2723 ms)
[12/28/2023-10:51:16] [I] Average on 10 runs - GPU latency: 41.3408 ms - Host latency: 43.4249 ms (enqueue 41.2328 ms)
[12/28/2023-10:51:16] [I] 
[12/28/2023-10:51:16] [I] === Performance summary ===
[12/28/2023-10:51:16] [I] Throughput: 24.4584 qps
[12/28/2023-10:51:16] [I] Latency: min = 39.4309 ms, max = 48.812 ms, mean = 42.6834 ms, median = 42.4526 ms, percentile(90%) = 45.2621 ms, percentile(95%) = 47.5834 ms, percentile(99%) = 48.812 ms
[12/28/2023-10:51:16] [I] Enqueue Time: min = 37.2709 ms, max = 49.4214 ms, mean = 40.5248 ms, median = 40.3695 ms, percentile(90%) = 44.2222 ms, percentile(95%) = 45.3137 ms, percentile(99%) = 49.4214 ms
[12/28/2023-10:51:16] [I] H2D Latency: min = 0.69458 ms, max = 0.765869 ms, mean = 0.743482 ms, median = 0.743408 ms, percentile(90%) = 0.750488 ms, percentile(95%) = 0.754883 ms, percentile(99%) = 0.765869 ms
[12/28/2023-10:51:16] [I] GPU Compute Time: min = 37.3687 ms, max = 46.7526 ms, mean = 40.6098 ms, median = 40.3651 ms, percentile(90%) = 43.1836 ms, percentile(95%) = 45.4991 ms, percentile(99%) = 46.7526 ms
[12/28/2023-10:51:16] [I] D2H Latency: min = 1.08813 ms, max = 1.42346 ms, mean = 1.33012 ms, median = 1.32031 ms, percentile(90%) = 1.37573 ms, percentile(95%) = 1.38623 ms, percentile(99%) = 1.42346 ms
[12/28/2023-10:51:16] [I] Total Host Walltime: 3.10731 s
[12/28/2023-10:51:16] [I] Total GPU Compute Time: 3.08634 s
[12/28/2023-10:51:16] [W] * Throughput may be bound by Enqueue Time rather than GPU Compute and the GPU may be under-utilized.
[12/28/2023-10:51:16] [W]   If not already in use, --useCudaGraph (utilize CUDA graphs where possible) may increase the throughput.
[12/28/2023-10:51:16] [W] * GPU compute time is unstable, with coefficient of variance = 5.58526%.
[12/28/2023-10:51:16] [W]   If not already in use, locking GPU clock frequency or adding --useSpinWait may improve the stability.
[12/28/2023-10:51:16] [I] Explanations of the performance metrics are printed in the verbose logs.
[12/28/2023-10:51:16] [I] 

### onnx result vs trt result
/home/icv/workspaces/inf_onsite/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/trt_onnx_compa.py

### pipeline example use Yolo
https://github.com/marcoslucianops/DeepStream-Yolo

### debug with log
script -c"python3.8 ds_3d_det_ros_main.py -i  file:////home/icv/workspaces/wheel/test_vedio.mp4" ds.log

### nv buffer to ros image
tiler_sink_pad.add_probe(Gst.PadProbeType.BUFFER, self.tiler_sink_pad_buffer_probe, 0)
`cv2_to_compressed_imgmsg`
Convert an OpenCV :cpp:type:`cv::Mat` type to a ROS sensor_msgs::CompressedImage message.



### rtsp input
drop frame
rtsp://service:Icv%24%241234@192.168.0.1

`IPv4`
Address 192.168.0.2 
Netmask 255.255.255.0
Web http://192.168.0.1/Settings.html#page_installer_menu


### torch2trt performance tools
https://github.com/NVIDIA-AI-IOT/torch2trt/releases
pip install absl-py
pip install onnx_graphsurgeon
pip install polygraphy
sudo apt install ./nsight-systems-2023.4.1_2023.4.1.97-1_arm64.deb
https://zhuanlan.zhihu.com/p/652696571


### latency
export GST_DEBUG="GST_TRACER:7"
export GST_TRACERS="latency(flags=element+pipeline)"
export GST_DEBUG_FILE=latency.log
python3.8 ds_3d_det_ros_main.py -i  file:////home/icv/workspaces/wheel/test_video.mp4

### image input
1920*1080

### model performance
/usr/bin/python /home/icv/code/workspaces/inf_onsite/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/trt_onnx_compa.py --config /home/icv/code/workspaces/inf_onsite/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/config/config.yaml

### numpy  version issue
AttributeError: module 'numpy' has no attribute 'bool'.
pip install numpy==1.23.2 -i https://mirrors.aliyun.com/pypi/simple/

### profile为dynamic shape所需要的
When working with networks that have dynamic or shape inputs, you need to define optimization profiles 
https://zhuanlan.zhihu.com/p/652918258

```c++
nvinfer1::IOptimizationProfile* profile = builder ->createOptimizationProfile();
profile->setDimensions("input", nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims32 {4, {1, 1, 1,1}});
profile->setDimensions("input", nvinfer1::OptProfileSelector::kOPT, nvinfer1::Dims32 {4, {3, 4, 5,6}});
profile->setDimensions("input", nvinfer1::OptProfileSelector::kMAX, nvinfer1::Dims32 {4, {6, 8, 10,12}});
config->addOptimizationProfile(profile);
```

### model input
name: input
tensor: float32[batch_size,3,1080,1920]

### pecision set in C++ tensorrt
https://github.com/cyrusbehr/tensorrt-cpp-api/blob/main/src/engine.cpp
config->setFlag(BuilderFlag::kFP16);

### new docker container
sudo docker run --gpus all --net=host -d -it -v /home/icv/workspaces/:/home/icv/workspaces/ --shm-size=64g --name inf-docker nvcr.io/nvidia/deepstream:6.4-triton-multiarch
sudo docker exec -it inf-docker /bin/bash

sudo docker run --gpus all --net=host -d -it -v /home/icv/workspaces/:/home/icv/workspaces/ --shm-size=64g --name rt-docker nvcr.io/nvidia/pytorch:21.08-py3
sudo docker exec -it rt-docker /bin/bash

nvidia-smi

sudo trtexec --onnx=/home/icv/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx --saveEngine=/home/icv/workspaces/inf_onsite/test/trtfiles/3d_det_engine_all_3090_fp16.trt --fp16

### install python-tensorrt
ERROR: Could not build wheels for tensorrt, which is required to install `pyproject.toml-based` projects
pip install nvidia-pyindex
pip install nvidia-tensorrt
https://zhuanlan.zhihu.com/p/607601799?utm_id=0
https://developer.download.nvidia.cn/compute/redist/nvidia-tensorrt/nvidia_tensorrt-8.4.3.1-cp310-none-linux_x86_64.whl

### engine version and tensorrt env
https://www.codenong.com/js3c2fb7b45cc7/
    context = engine.create_execution_context()
AttributeError: 'NoneType' object has no attribute 'create_execution_context'

### engine
41M Jan 11 09:25 3d_det_engine_all_3090_fp16.trt
41M Jan 11 09:23 your_model_fp16.trt
