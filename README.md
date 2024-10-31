 Attention:
"Inf_Onsite" will contain all deployment components such as visibility grid, free space, monocular camera 3d object detection, lidar perception, gt system, ...

## understand the code for training
1. how to train the model
src/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/README.md  --> `train_val.py` 

2. convert .pth to .onnx
src/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/pth2onnx.py --> onnx

3. github base code:
https://github.com/Xianpeng919/MonoCon
https://sourcecode.socialcoding.bosch.com/projects/RIX3/repos/infrastructure3dobjectdetection/browse

4. intrinsic parameter `P2` (only intrinsic paramerter is required for train phase):

     def __init__(self, calib_filepath):
        calibs = self.read_calib_file(calib_filepath)
        # Projection matrix from rect camera coord to image2 coord
        self.P = calibs['P2']
        self.P = np.reshape(self.P, [3, 4])
        self.c_u = self.P[0, 2]
        self.c_v = self.P[1, 2]
        self.f_u = self.P[0, 0]
        self.f_v = self.P[1, 1]
        self.b_x = self.P[0, 3] / (-self.f_u)  # relative
        self.b_y = self.P[1, 3] / (-self.f_v)

5. weight file -->  checkpoint 
load_checkpoint(model=model,
                    optimizer=None,
                    filename=cfg['tester']['checkpoint'],
                    map_location=device,
                    logger=logger)

6. train loss
            total_loss, stats_batch = compute_centernet3d_loss(
                outputs, targets)
            total_loss.backward()
            self.optimizer.step()
            
## code understanding for deepstream 
https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Overview.html
https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/blob/master/apps/deepstream-test3/dstest3_pgie_config.txt
https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_Zero_Coding_Sample_Graphs.html#deepstream-test3

1. extrinsic parameter
`camera_pose_info` extrinsic parameter --> camera_pose_info =[[-1.771768, -0.048427, -0.807171],[-11.039729, -1.951577, 7.277434]] # new lens w

2. deepstream API
sinkpad= streammux.get_request_pad(padname)  -- > single stream

 link
   queue1.link(pgie)
    pgie.link(queue2)

3. .so file generate
camera/monocular_3d/deepstream/inference_script/inference_parser/monocam_det/nvdsinfer_monocular_cam_3d_det/example_app.cpp  --> parse output layout

"primary-pgie" section
model-engine-file = /home/icv/Edward/inf_sense/test/trtfiles/docker_3d_det_engine.trt



4. overall pipeline based on deepstream test3
`pgie_src_pad_buffer_probe` .so --> camera/monocular_3d/deepstream/deepstream-test3/ds_3d_det_ros_main.py (custom-lib-path=/home/icv/Edward/deepstream/inference_script/inference_parser/monocam_det/nvdsinfer_monocular_cam_3d_det/build/libdemo.so)

5. kdtree --> `tree = kdtree.create(coord_ap_zc)` `search_knn`
KD-Tree原理详解 - yachen zhang的文章 - 知乎
https://zhuanlan.zhihu.com/p/112246942

6. workaround
 depth = depth/1.5

7. generate .trt file
https://docs.nvidia.com/deeplearning/tensorrt/quick-start-guide/index.html#export-from-pytorch

8. (u,v,Zc) --> (Xw,Yw,Zw) 像素坐标和深度转化为相机坐标系

9. outputs = model(inputs) --> raw output from model  --> `extract_dets_from_outputs`
(camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/inference.py)

  dets = extract_dets_from_outputs(outputs=outputs,
                                    K=test_loader.dataset.max_objs)

10. bag debug info
                # debug info
                # this data range is error beside yaw & yaw rate
                obj_det.width_variance = obj_meta.confidence
                obj_det.length_variance = obj_meta.rect_params.left
                obj_det.height_variance = obj_meta.rect_params.width
                obj_det.yaw_rate = obj_meta.rect_params.height
                obj_det.x_offset = obj_meta.rect_params.top

11. deepstram interferce output
obj_meta=pyds.NvDsObjectMeta.cast(l_obj.data)

12. GStreamer `pipeline` / GStreamer `bus` / GStreamer `element`
pipeline = Gst.Pipeline()
element = Gst.ElementFactory.make(factory_name, element_name)
bus = pipeline.get_bus()

13. test latency for gstreamer pipeline
export GST_DEBUG="GST_TRACER:7"
export GST_TRACERS="latency(flags=element+pipeline)"
export GST_DEBUG_FILE=latency.log
 
source /home/icv/Edward/test/sample/install/perception_kit_msgs/share/perception_kit_msgs/local_setup.bash
source /home/icv/Edward/04_tools/analysis_tool/ros2_ws/install/local_setup.bash 
source /home/icv/Edward/04_tools/analysis_tool/ros2_ws/install/adma_msgs/share/adma_msgs/local_setup.bash 
python3.8 ds_3d_det_ros_main.py -i  file:////home/icv/Edward/tools/test_pic_video.mp4

14. `pgie_config.txt`  `custom-lib-path` `model-engine-file`
In the context of NVIDIA DeepStream and the Primary GIE (General Inference Engine), the `pgie` (Primary GIE) configuration typically includes parameters related to how the deep learning model is used for inference. The specific parameters you've mentioned, `custom-lib-path` and `model-engine-file`, are part of this configuration and are used for specifying custom inference engines and their paths.

* `custom-lib-path`: This parameter is used to specify the path to a custom library that implements the inference engine. DeepStream allows you to use custom inference engines if the built-in engines do not meet your requirements. By specifying the path to a custom library, you can integrate your own inference engine with DeepStream.

* `model-engine-file`: This parameter is used to specify the path to the TensorRT model engine file. TensorRT is a high-performance deep learning inference library developed by NVIDIA. The model engine file contains the optimized model for a specific target GPU, and it's generated from the original model using TensorRT. This file is essential for efficient inference in DeepStream.

Here's an example of how you might use these parameters in a `pgie` configuration:

```ini
[property]
...
custom-lib-path=/path/to/custom_inference_lib.so
model-engine-file=/path/to/model.engine
...
```

In this example, you specify the path to a custom inference library with `custom-lib-path` and the path to the TensorRT model engine file with `model-engine-file`. These paths would need to be replaced with the actual paths to your custom library and model engine file.

Keep in mind that the exact configuration and usage may vary depending on your specific DeepStream application and the deep learning models you are using. You should consult the DeepStream documentation and the documentation specific to your custom inference engine and models for more detailed information on how to set up these parameters correctly.

### Gstreamer concept
In a typical GStreamer application, you create a pipeline by adding elements, configure their properties, link them together to define the data flow, and monitor events and messages through the bus. By using these components, you can build multimedia applications for various purposes, including video playback, streaming, transcoding, and more. The GStreamer framework provides a flexible and modular approach to multimedia processing.

### GStreamer code example
here's a typical GStreamer application code that includes the concepts of pipelines, elements, the bus, and some essential base concepts. This example creates a simple video player application:

```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib

# Initialize GStreamer
Gst.init(None)

# Create a GStreamer pipeline
pipeline = Gst.Pipeline()

# Create GStreamer elements
src_element = Gst.ElementFactory.make("filesrc", "file-source")
decoder_element = Gst.ElementFactory.make("decodebin", "decoder")
video_sink_element = Gst.ElementFactory.make("autovideosink", "video-sink")

if not pipeline or not src_element or not decoder_element or not video_sink_element:
    print("Not all elements could be created.")
    exit(1)

# Set the input file path
src_element.set_property("location", "your_input_video.mp4")

# Add elements to the pipeline
pipeline.add(src_element, decoder_element, video_sink_element)

# Link elements in the pipeline
src_element.link(decoder_element)

# Callback to handle dynamically added pads
def on_pad_added(element, pad):
    pad.link(video_sink_element.get_static_pad("sink"))

decoder_element.connect("pad-added", on_pad_added)

# Create a GStreamer bus and monitor messages
bus = pipeline.get_bus()

def on_message(bus, message):
    t = message.type
    if t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"Error: {err} - {debug}")
        pipeline.set_state(Gst.State.NULL)
    elif t == Gst.MessageType.EOS:
        print("End of stream")
        pipeline.set_state(Gst.State.NULL)
    elif t == Gst.MessageType.STATE_CHANGED:
        old_state, new_state, pending_state = message.parse_state_changed()
        print(f"State changed from {Gst.Element.state_get_name(old_state)} to {Gst.Element.state_get_name(new_state)}")
    else:
        print("Other message:", message.type)

bus.add_signal_watch()
bus.connect("message", on_message)

# Start the pipeline
pipeline.set_state(Gst.State.PLAYING)

# Run the GLib MainLoop
loop = GLib.MainLoop()
loop.run()

# Clean up and stop the pipeline
pipeline.set_state(Gst.State.NULL)
```

This example does the following:

1. Initializes GStreamer.

2. Creates a GStreamer pipeline with three elements: `filesrc` for reading the input video file, `decodebin` for dynamic decoding, and `autovideosink` for video display.

3. Sets the input file path.

4. Links the elements together to form a data processing chain.

5. Sets up message handlers to monitor the GStreamer bus for error, EOS, and state change messages.

6. Starts the pipeline in the PLAYING state.

7. Runs a GLib MainLoop to keep the application running.

8. When you close the application, it stops the pipeline and cleans up resources.

This code demonstrates the core concepts of GStreamer, including creating a pipeline, adding elements, linking them, monitoring the bus, and handling messages. You can customize this code for your specific use case and add more features, such as audio processing, error handling, and user interaction.

# 2024/03/11
# 训练
batch size  --> RTX3090 设置为6,与内存有关，一个epoch `batch_size: 6`
max_epoch: 260

camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/config/config.yaml
.pth 权重文件
resume model --> 断电重新训练

meansize 利用先验知识，比如车长3.5

downsample 训练做`下采样`

(`下采样`的逆运算)
camera/monocular_3d/deepstream/deepstream-test3/lib/helpers/decode_helper.py (`下采样`的逆运算)
            # 2d bboxs decoding
            x = dets[i, j, 2] * info['bbox_downsample_ratio'][i][0]
            y = dets[i, j, 3] * info['bbox_downsample_ratio'][i][1]
            w = dets[i, j, 4] * info['bbox_downsample_ratio'][i][0]
            h = dets[i, j, 5] * info['bbox_downsample_ratio'][i]

##  checkpoint  260 个 epoch，一般去第120个epoch的.pth文件
## 辅助训练
`辅助训练` --> camera/monocular_3d/deepstream/deepstream-test3/lib/models/monocon.py （额外的用于训练的HEAD）
'center2kpt_offset': 16,
'kpt_heatmap': 8,
'kpt_heatmap_offset': 2

## 多任务 loss的计算

## 网络结构
neck
    backbone='dla34',        
    neck='DLAUp',  

## 收敛曲线
P R 曲线
https://zhuanlan.zhihu.com/p/104917232

## 数据集
DAIR数据集 https://thudair.baai.ac.cn/rope

## 多卡训练
self.gpu_ids = list(map(int, cfg['gpu_ids'].split(',')))
self.model = torch.nn.DataParallel(model).cuda()

## heatmap
heatmap
`nms` --> heatmap

xs2d ys2d --> 2D bounding box 中心点
xs3d xs3d  -->  3D bounding box中心点 （像素坐标系）

locations --> 相机坐标系（把 u,v,depth 转化为 相机坐标系的结果）-->`（u,v,1）*Zc = K*RT*(X,Y,Z,1)`
       locations = calibs[i].img_to_rect(x3d, y3d, depth).reshape(-1)

## NMS
NMS（Non-Maximum Suppression，非极大值抑制）

## KD tree 求 深度信息
       K 内参
           global RT_augmented
    global K
https://blog.csdn.net/MengYa_Dream/article/details/120233806?spm=1001.2014.3001.5506
（u,v,1）*Zc = K*RT*(X,Y,Z,1)

## 训练精度 fp64, 推理 fp16

# deployment
## .pth-->.onnx-->.trt
##  后处理，解析输出

# deepstream
## .so --> decode 网络的结果

## RTSP流密码文件（$符号的ASCII码为24）  %24% --> $  -->('rtsp://service:Icv%24%241234@192.168.5.72')

## ## extract -- decoder in deepstream
`dstest3_pgie_config.txt` --> camera/monocular_3d/deepstream/deepstream-test3/dstest3_pgie_config.txt

parse-bbox-func-name
custom-lib-path

pgie_src_pad_buffer_probe

camera/monocular_3d/deepstream/inference_script/inference_parser/monocam_det/nvdsinfer_monocular_cam_3d_det/example_app.cpp
NvDsInferParseCustomResnet (std::vector<NvDsInferLayerInfo> const &outputLayersInfo

## torch::blob 和 nms算法
`torch::blob`

`nms`
  heatmap_ts = _nms(heatmap_ts);

### 数据结构问题，自己拼接数据
NvDsInfer3dObjectDetectionInfo 原始数据类型 `显卡位宽`

NvDsInferObjectDetectionInfo_4decode 自定义数据类型

      objectList.push_back(obj);
      // obj.left = 12345678.;   // 8 most                    hw      4+4=8
      // obj.top = 998.123;   // 6 most                       dep
      // obj.width = 12345678.;   // 8 most                   x3d y3d  4+4=8
      // obj.height = 5.12345;   // 6 most                    heading
      // obj.detectionConfidence = 12345678.;   // 8 most     conf l    4+4=8

      auto d_h = object_info_list[i].Size_3d[0];  // to 0.01m 
      auto d_w = object_info_list[i].Size_3d[1];  // to 0.1 m
      auto d_l = object_info_list[i].Size_3d[2];  // to 0.1 m
      obj.left = min(float(1E4*round(min(d_h*1E2,1E3))+round(min(d_w*1E2,1E5))),99999999.f);
      // obj.left = 99999999.f;
      int x_3d = object_info_list[i].Xs3d*downsample_ratio;  // to 1 pixel
      int y_3d = object_info_list[i].Ys3d*downsample_ratio;  // to 1 pixel
      obj.width = min(float(1E4*round(min(x_3d,int(1E4)))+round(min(y_3d,int(1E4)))),99999999.f);

      ------
      ###拟运算##

                    obj_det.width = (obj_meta.rect_params.left%1E4)/1E2
                obj_det.length = int(obj_meta.confidence/1E4)/1E2
                obj_det.height = int(obj_meta.rect_params.left/1E4)/1E2
                # obj_det.yaw = obj_meta.rect_params.height+math.pi/2-0.029865
                obj_det.yaw = obj_meta.rect_params.height # -math.pi/2+0.05
                obj_det.yaw = -obj_det.yaw+math.pi/2+0.05 # hongye west

### ROI filter
精度不好的区域，直接删掉目标

# 2014/03/12 train hand-on
### ground plane
地面方程

### let train run
python3.8  train_val.py --config config/config.yaml

It monitors your GPU every second, refreshing and tracking the output itself for each second.

$ watch –n 1 -d nvidia-smi

### 为什么现在的模型从180个epoch开始训练
resume-model 180
6*1488 
如果内存报错，改batch size(memory error)

`DAIR数据集` 中label的方位角度：-pi~pi --> 0~2*pi

可视化画框visualize.py

python3.8 pth2onnx.py- --config config/config.yaml

### yolo V7 (anchor-based  一阶段算法)
(原来的算法是anchor-free 也是一阶段算法)
感受野（Receptive Field）anchor 3种

迁移训练
rop3d数据集转化为coco 数据集


yolo 3d 与2d的关系
然后还做了yolo 的2d到3d的转换，就改了head，修改了loss函数，还改了一下dataload那个函数，因为要回归到3d信息

