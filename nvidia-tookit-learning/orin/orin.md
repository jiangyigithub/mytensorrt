# 一、安装Nvidia-jetpack和基础开发环境

## 1、修改deb source

```bash
$ cat /etc/apt/sources.list.d/nvidia-l4t-apt-source.list

# SPDX-FileCopyrightText: Copyright (c) 2019-2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.

deb https://repo.download.nvidia.com/jetson/common r34.0 main   # 改成34.1
deb https://repo.download.nvidia.com/jetson/t234 r34.0 main		# 改成34.1
```

## 2、安装jetpack

```bash
sudo apt update
sudo apt dist-upgrade
sudo reboot
sudo apt install nvidia-jetpack
```

## 3、验证jetpack安装

```bash
sudo apt show nvidia-jetpack  
# jetpack中已经安装了 docker 和 nvidia-docker, 存在 triton / deepStream 使用需求时可直接通过docker启动
```

## 4、预装必要软件

```bash
sudo apt-get -y install cmake libopenblas-base libopenmpi-dev autoconf bc build-essential g++-8 gcc-8 clang-8 lld-8 gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev libxslt-dev locales moreutils openssl python-openssl rsync scons python3-pip libopenblas-dev libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
```

# 二、安装GPU监测工具

```bash
sudo -H pip install jetson-stats
sudo jtop
```

jtop窗口怎么看：[jetson-stats 4.2.1 (rnext.it)](https://rnext.it/jetson_stats/)

# 三、整理cudatookit、cudnn、trtexec工具

## 1、cudatookit

```bash
ln -s /usr/local/cuda-11 /usr/local/cuda
sudo vim /etc/profile
# 加入以下
export PATH=/usr/local/cuda-11/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11/lib64$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-11

# 验证cudatookit
source /etc/profile
nvcc -V
```

## 2、cudnn

```bash
sudo cp /usr/include/cudnn.h /usr/local/cuda-11/include
sudo cp /usr/lib/aarch64-linux-gnu/libcudnn* /usr/local/cuda-11/lib64/
sudo chmod 777 /usr/local/cuda-11/include/cudnn.h /usr/local/cuda-11/lib64/libcudnn*

# 验证和更新链接库
cd /usr/local/cuda-11/lib64
ll libcudnn*  # 有东西
sudo ldconfig

# 测试cudnn
sudo cp -r /usr/src/cudnn_samples_v8/ /tmp/
cd /tmp/cudnn_samples_v8/conv_sample
sudo make clean
sudo make
./conv_sample
```

## 3、trtexec

```bash
ln -s /usr/src/tensorrt/bin/trtexec /usr/bin/trtexec
```

# 验证tensorrt正确安装

```bash
sudo chmod -R 777 /usr/src/tensorrt
cd /usr/src/tensorrt/samples/sampleINT8/
sudo make
cd ../../bin/
./sampleINT8
```

# 四、安装pytorch开发环境

去[Releases · conda-forge/miniforge (github.com)](https://github.com/conda-forge/miniforge/releases)下载miniforge(aarch64版轻量级conda)用于环境管理。

```bash
bash Miniforge-pypy3-22.11.1-4-Linux-aarch64.sh
# 在选项中请选择默认执行conda init
# 添加你喜欢的镜像
# 这里使用国科大镜像源
conda config --prepend channels https://mirrors.ustc.edu.cn/anaconda/pkgs/main/
conda config --prepend channels https://mirrors.ustc.edu.cn/anaconda/pkgs/free/
conda config --set show_channel_urls yes
```

安装pytorch：在[Jetson Zoo - eLinux.org](https://elinux.org/Jetson_Zoo)上下载whl文件

```bash
conda create -n trt python=3.8
conda activate trt
pip install Cython numpy 'pillow<7'
pip install torch-1.11.0-cp38-cp38-linux_aarch64.whl
```

安装torchvision (version=0.12.0)

```bash
git clone --branch <version> https://github.com/pytorch/vision torchvision  
cd torchvision
export BUILD_VERSION=0.12.0
python3 setup.py install --user
```

scipy matplotlib pandas jupyter onnx 等常用库可自行安装，特殊地，onnxruntime-gpu 需要到 [Jetson Zoo - eLinux.org](https://elinux.org/Jetson_Zoo) 上安装。

## 安装python其它必要依赖

```bash
# 把/usr/lib/python3.8/dist-packages/下jetpack自动下好的内容拷贝至虚拟环境(~/miniforge3/envs/trt/lib/dist-packages)下
sudo ln -s /usr/lib/python3.8/dist-packages/cv2/python-3.8/cv2.cpython-38-aarch64-linux-gnu.so cv2.so
```

# 五、网站访问加速配置

在 [Releases · Dreamacro/clash (github.com)](https://github.com/Dreamacro/clash/releases) 上下载 clash 软件

```bash
# 1、安装
gzip -d clash-linux-armv7-v1.14.0.gz      # 用 cat /proc/cpuinfo 查看你的arm是v5/v6/v7版本的
mkdir /opt/clash 
mv clash-linux-armv7-v1.14.0 /opt/clash/clash 
cd /opt/clash

# 2、配置
wget -O config.yaml [订阅链接] 
wget -O Country.mmdb https://www.sub-speeder.com/client-download/Country.mmdb   # 太慢或连不上请点击链接手动下载

# 3、运行
chmod 777 
clash ./clash -d .
```

保持`clash ./clash -d .`运行中，使用火狐或谷歌浏览器安装 ProxySwitchyOmega 插件用于启动浏览器代理。

安装 proxychains-ng 用于启动终端代理

```bash
git clone https://github.com/rofl0r/proxychains-ng.git
cd proxychains-ng 
./configure --sysconfdir=/etc 
make 
sudo make install
```

安装完成之后，把`src/proxychains.conf`移动到`/etc/proxychains.conf`，然后编辑该文件添加代理（与`config.yaml`文件中的内容对应）：

```bash
socks5 127.0.0.1 7891
```

至此 git 克隆失败的问题可以使用以下语句解决（保持`clash ./clash -d .`运行）：

```bash 
proxychains git clone xxxxxxxx
```

# 六、其它

[yqlbu/jetson-packages-family: The ultimate software installation guide for Nvidia Jetson Nano/Xavier Dev Kit (github.com)](https://github.com/yqlbu/jetson-packages-family) 提供了Jetson设备上的常用软件安装教程