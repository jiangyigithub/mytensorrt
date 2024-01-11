    1  ls
    2  python
    3  python3 --version
    4  ifconfig
    5  sudo apt install net-tools
    6  ifconfig
    7  sudo apt install openssh-server
    8  sudo systemctl status ssh
    9  sudo ufw allow ssh
   10  ls
   11  cd Downloads/
   12  ls
   13  sudo apt install ./foxglove-studio-1.82.0-linux-amd64.deb 
   14  ls -al
   15  sudo apt install ./foxglove-studio-1.82.0-linux-amd64.deb 
   16  python --version
   17  python3 --version
   18  locale
   19  sudo apt update
   20  sudo apt upgrade
   21  sudo apt install locales
   22  locale
   23  sudo update-locale LC_ALL=en_US.UTF-8
   24  locale
   25  sudo apt install software-properties-common
   26  sudo add-apt-repository universe
   27  sudo apt update && sudo apt install curl -y
   28  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   29  ls
   30  mkdir workspaces
   31  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   32  sudo apt update && sudo apt install ros-dev-tools
   33  sudo apt update
   34  sudo apt upgrade
   35  sudo apt install ros-rolling-desktop
   36  sudo apt install python3.8
   37  sudo apt install ros-rolling-ros-base
   38  source /opt/ros/rolling/setup.bash
   39  ros2 topic list
   40  sudo apt install ros-rolling-foxglove-bridge
   41  ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
   42  source /opt/ros/rolling/setup.bash
   43  ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
   44  sudo apt install ros-rolling-foxglove-msgs
   45  source /opt/ros/rolling/setup.bash
   46  ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
   47  sudo apt update
   48  apt list --upgradable
   49  ifconfig
   50  sudo service gdm stop
   51  sudo service lightdm stop
   52  sudo pkill -9 Xorg
   53  cd Downloads/
   54  ls
   55  chmod 755 NVIDIA-Linux-x86_64-535.146.02.run 
   56  sudo ./NVIDIA-Linux-x86_64-535.146.02.run --no-cc-version-check
   57  reboot
   58  sudo reboot
   59  lspci | grep -i nvidia
   60  sudo service gdm stop
   61  sudo service lightdm stop
   62  sudo pkill -9 Xorg
   63  cd Downloads/
   64  ls
   65  sudo ./NVIDIA-Linux-x86_64-535.146.02.run --no-cc-version-check
   66  sync
   67  sudo reboot
   68  pwd
   69  sudo apt install libssl3 libssl-dev libgstreamer1.0-0 gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-0 libjansson4 libyaml-cpp-dev
   70  ls
   71  cd workspaces/
   72  git clone https://github.com/confluentinc/librdkafka.git
   73  cd librdkafka
   74  git checkout tags/v2.2.0
   75  ./configure --enable-ssl
   76  make
   77  sudo make install
   78  sudo mkdir -p /opt/nvidia/deepstream/deepstream-6.4/lib
   79  sudo cp /usr/local/lib/librdkafka* /opt/nvidia/deepstream/deepstream-6.4/lib
   80  sudo ldconfig
   81  cd ..
   82  cd ../Downloads/
   83  ls
   84  ls -al
   85  chmod 777 deepstream-6.4_6.4.0-1_amd64.deb 
   86  ls -al
   87  sudo apt-get install ./deepstream-6.4_6.4.0-1_amd64.deb 
   88  sudo rm -rf /usr/local/deepstream /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnv* /usr/bin/deepstream* /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libnvdsgst*
   89  /usr/lib/x86_64-linux-gnu/gstreamer-1.0/deepstream*
   90  /opt/nvidia/deepstream/deepstream*
   91  sudo rm -rf /usr/lib/x86_64-linux-gnu/gstreamer-1.0/deepstream* /opt/nvidia/deepstream/deepstream*
   92  sudo rm -rf /usr/bin/deepstream* /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libnvdsgst*
   93  sudo rm -rf /usr/local/deepstream
   94  sudo rm -rf /usr/lib/x86_64-linux-gnu/gstreamer-1.0/libgstnv*
   95  sudo apt install libssl3 libssl-dev libgstreamer1.0-0 gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-0 libjansson4 libyaml-cpp-dev libjsoncpp-dev protobuf-compiler gcc make git python3
   96  sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
   97  sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /"
   98  sudo apt-get update
   99  sudo apt-get install cuda-toolkit-12-2
  100  sudo apt-get install libnvinfer8=8.6.1.6-1+cuda12.0 libnvinfer-plugin8=8.6.1.6-1+cuda12.0 libnvparsers8=8.6.1.6-1+cuda12.0 libnvonnxparsers8=8.6.1.6-1+cuda12.0 libnvinfer-bin=8.6.1.6-1+cuda12.0 libnvinfer-dev=8.6.1.6-1+cuda12.0 libnvinfer-plugin-dev=8.6.1.6-1+cuda12.0 libnvparsers-dev=8.6.1.6-1+cuda12.0 libnvonnxparsers-dev=8.6.1.6-1+cuda12.0 libnvinfer-samples=8.6.1.6-1+cuda12.0 libcudnn8=8.9.4.25-1+cuda12.2 libcudnn8-dev=8.9.4.25-1+cuda12.2
  101  ls
  102  sudo apt-get install ./deepstream-6.4_6.4.0-1_amd64.deb 
  103  sudo apt install ./deepstream-6.4_6.4.0-1_amd64.deb 
  104  cd /opt/nvidia/deepstream/deepstream-6.4/samples/configs/
  105  ls
  106  cd deepstream-app
  107  ls
  108  deepstream-app -c config_infer_primary.txt
  109  cd ..
  110  code .
  111  ls
  112  cd sources/apps/sample_apps/
  113  ls
  114  cd deepstream-test1/
  115  ls
  116  sudo make
  117  ls
  118  ./deepstream-test1-app ../../../../samples/streams/sample_1080p_h265.mp4
  119  ./deepstream-test1-app ../../../../samples/streams/sample_720p.h264
  120  pwd
  121  ls
  122  cd src/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/lib/helpers/
  123  ls
  124  ls -al
  125  ls -al
  126  nvcc --version
  127  sudo apt install nvidia-cuda-toolkit
  128  nvcc --version
  129  nvidia-smi
  130  nvcc --version
  131  ls -al
  132  sudo make
  133  ls
  134  ls -al
  135  cd ~/Downloads/
  136  ls
  137  ls -al
  138  rm torch-2.1.2+cpu-cp310-cp310-linux_x86_64.whl
  139  sudo apt install torch-2.1.2+cu121-cp310-cp310-linux_x86_64.whl 
  140  pip3 install torch-2.1.2+cu121-cp310-cp310-linux_x86_64.whl
  141  ls -al
  142  chmod 777 torch-2.1.2+cu121-cp310-cp310-linux_x86_64.whl
  143  pip3 install torch-2.1.2+cu121-cp310-cp310-linux_x86_64.whl
  144  source ~/.bashrc 
  145  echo $PATH
  146  sudo pip3 install torch-2.1.2+cu121-cp310-cp310-linux_x86_64.whl
  147  pipi3 --version
  148  pip3 --version
  149  mkdir build
  150  cd build/
  151  cmake ..
  152  python3 --version
  153  ll
  154  ls -al
  155  chmod 777 cuda_11.8.0_520.61.05_linux.run
  156  ./cuda_11.8.0_520.61.05_linux.run
  157  nvcc 
  158  ls /usr/local/cuda-12.1/
  159  ls -al /usr/local/cuda-12.1/
  160  ls -al /usr/local/cuda
  161  sudo apt-get install cuda-toolkit-12-2
  162  sudo apt-get install cuda-toolkit-12-1
  163  export PATH=/usr/local/cuda-12.2/bin${PATH:+:${PATH}}
  164  nvcc --version
  165  nvcc --version
  166  export PATH=/usr/local/cuda-12.1/bin${PATH:+:${PATH}}
  167  nvcc --version
  168  echo $PATH
  169  echo export PATH=/usr/local/cuda-12.1/bin${PATH:+:${PATH}} > ~/.bashrc 
  170  df -h
  171  ls
  172  cd 03_data/
  173  ls
  174  cd Data/
  175  cd ..
  176  cd ../inf_sense/
  177  ls
  178  code .
  179  sudo apt install code
  180  cd ..
  181  cp deepstream ~/workspaces/
  182  cp -r deepstream ~/workspaces/
  183  ls
  184  cp -r 01_tools/ ~/workspaces/
  185  cp rviz2_HY.rviz ~/workspaces/
  186  cd ..
  187  ls
  188  cd Edward/
  189  ls
  190  ls inf_sense/
  191  ls
  192  ls -al
  193  ls
  194  cp -r inf_sense/ ~/workspaces/
  195  cd ~/workspaces/librdkafka/
  196  sudo make install
  197  ls /opt/nvidia/deepstream/deepstream-6.4/lib
  198  sudo mkdir -p /opt/nvidia/deepstream/deepstream-6.4/lib
  199  sudo cp /usr/local/lib/librdkafka* /opt/nvidia/deepstream/deepstream-6.4/lib
  200  sudo ldconfig
  201  cd ..
  202  sudo apt install code
  203  snap install code
  204  sudo snap install code
  205  sudo snap install --classic code
  206  ls
  207  cd deepstream/
  208  code .
  209  cd /opt/nvidia/deepstream/deepstream-6.4/
  210  ls
  211  code .
  212  ls
  213  cd ../..
  214  ls
  215  cd deepstream/
  216  ls deepstream/deepstream-test1/model/cal_trt.bin
  217  ls deepstream-test1/model/cal_trt.bin
  218  ls
  219  cd ../inf_sense/test/trtfiles/
  220  ls
  221  ll
  222  ll
  223  ls
  224  ls -al
  225  pwd
  226  source /media/icv-ai/17aed8c1-fa27-4be1-bda0-25bb0361f3ec/home/icv/Edward/test/sample/install/perception_kit_msgs/share/perception_kit_msgs/local_setup.bash 
  227  source /media/icv-ai/17aed8c1-fa27-4be1-bda0-25bb0361f3ec/home/icv/Edward/04_tools/analysis_tool/ros2_ws/install/local_setup.bash 
  228  source /media/icv-ai/17aed8c1-fa27-4be1-bda0-25bb0361f3ec/home/icv/Edward/04_tools/analysis_tool/ros2_ws/install/adma_msgs/share/adma_msgs/local_setup.bash 
  229  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  230  pip install rclpy
  231  sudo apt install python3-pip
  232  pip install rclpy
  233  pip3 --version
  234  pip3 install rclpy
  235  source /opt/ros/rolling/setup.bash 
  236  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  237  pip3 install depth_geometry
  238  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  239  pip3 install pyds
  240  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  241  source ~/workspaces/ros2_ws/install/perception_kit_msgs/share/perception_kit_msgs/local_setup.bash 
  242  source ~/workspaces/ros2_ws/install/adma_msgs/share/adma_msgs/local_setup.bash
  243  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  244  source /opt/ros/rolling/setup.bash 
  245  lls
  246  ll
  247  sudo apt update
  248  pyds -v
  249  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  250  python3 pyds -v
  251  where python3
  252  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  253  ls /home/icv-ai/workspaces/deepstream/samples/streams/sample_1080p_h264.mp4
  254  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  255  ls /home/icv-ai/workspaces/deepstream/sample_bkp/sources/libs/nvdsinfer_monocular_cam_3d_det/build/libdemo.so
  256  python3 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4
  257  ls /home/icv-ai/workspaces/deepstream/sample_bkp/sources/libs/nvdsinfer_monocular_cam_3d_det/build/libdemo.so
  258  ls
  259  echo source /opt/ros/rolling/setup.bash > ~/.bashrc 
  260  cat ~/.bashrc 
  261  source /opt/ros/rolling/setup.bash
  262  colcon build
  263  code .
  264  g++ -v
  265  gcc -v
  266  make -v
  267  colcon build
  268  sudo apt-get install ros-rolling-diagnostic-updater
  269  colcon build
  270  colcon build radar_gen5_common
  271  colcon build --packages-select radar_gen5_common
  272  colcon build
  273  colcon build --packages-select track_to_track_fusion
  274  clear
  275  colcon build --packages-select track_to_track_fusion
  276  pwd
  277  cd /home/icv-ai/.local/lib/python3.10/site-packages/torch/share/cmake/Torch
  278  code .
  279  cd /opt/nvidia/deepstream/deepstream-6.4/
  280  code .
  281  cd ..
  282  cd ~/workspaces/
  283  ls
  284  cd deepstream/
  285  ls
  286  cd ..
  287  git clone https://github.com/NVIDIA-AI-IOT/deepstream_python_apps
  288  git clone https://github.com/NVIDIA-AI-IOT/deepstream_python_apps
  289  git clone https://github.com/NVIDIA-AI-IOT/deepstream_python_apps.git
  290  ls
  291  cd deepstream_python_apps/
  292  ls
  293  git submodule update --init
  294  history