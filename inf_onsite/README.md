 Attention:
"Inf_Onsite" will contain all deployment components such as visibility grid, free space, monocular camera 3d object detection, lidar perception, gt system, ...

## understand the code for training
1. how to train the model
src/camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/README.md  --> train_val.py 

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
`parse-bbox-func-name=NvDsInferParseCustomResnet`

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

8. (u,v,Zc) --> (Xw,Yw,Zw)

9. outputs = model(inputs) --> raw output from model  --> `extract_dets_from_outputs`
(camera/monocular_3d/infrastructure3dobjectdetection/baseline_methods/inference.py)

  dets = extract_dets_from_outputs(outputs=outputs,
                                    K=test_loader.dataset.max_objs)

10. bag debug info, fill into useless info,such as `width_variance`
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
python3.8 ds_3d_det_ros_main.py -i  file:////home/icv/workspaces/wheel/test_video.mp4

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
