1. `infer_resnet50.py` --> inference pytorch
infer_resnet50.py inputs the image, performs a classification and outputs the top 5 results with the respective probabilities.
2. `bench_resnet50.py` --> implements benchmarking of ResNet50
Benchmarking the torchvision models

3. `generate_onnx_resnet50.py` --> Convert a PyTorch model to ONNX

4. `trt_onnx_parser.py` 构建期 
To perform inference of the ONNX model using TensorRT, it must be pre-processed using the TensorRT ONNX parser. We will start with conversion of the ONNX representation to the TensorRT plan. The TensorRT plan is a serialized form of a TensorRT engine. The TensorRT engine represents the model optimized for execution on a chosen CUDA device.

5. `trt_infer_plan.py` 运行期 
implements TensorRT inference using the previously generated TensorRT plan and a pre-processed input image.

6. `trt_bench_plan.py` implements inference benchmarking using the previously generated TensorRT plan and a pre-processed input image.

