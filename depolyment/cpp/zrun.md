`cudnn_softmax.cpp`
g++ -o cudnn_softmax -I /usr/local/cuda/include \
    cudnn_softmax.cpp \
    -L /usr/local/cuda/lib64 -lcudnn -lcudart

`trt_onnx_parser.cpp`
g++ -o trt_onnx_parser \
    -I /usr/local/cuda/include \
    trt_onnx_parser.cpp common.cpp \
    -L /usr/local/cuda/lib64 -lnvonnxparser -lnvinfer -lcudart
./trt_onnx_parser ../model/resnet50.onnx ../model/resnet50_cpp.plan

`trt_infer_plan.cpp`
g++ -o trt_infer_plan \
    -I /usr/local/cuda/include \
    trt_infer_plan.cpp common.cpp \
    -L /usr/local/cuda/lib64 -lnvinfer -lcudart

./trt_infer_plan ../model/resnet50_cpp.plan ../data/husky01.dat

`trt_bench_plan.cpp`
g++ -o trt_bench_plan \
    -I /usr/local/cuda/include \
    trt_bench_plan.cpp common.cpp \
    -L /usr/local/cuda/lib64 -lnvinfer -lcudart
./trt_bench_plan ../model/resnet50_cpp.plan


`trt_onnx_parser_icv.cpp`
g++ -o trt_onnx_parser_icv \
    -I /usr/local/cuda/include \
    trt_onnx_parser_icv.cpp common.cpp \
    -L /usr/local/cuda/lib64 -lnvonnxparser -lnvinfer -lcudart
./trt_onnx_parser_icv /home/icv/code/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx ../model/resnet50_icv_cpp.plan