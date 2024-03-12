g++ -o main \
    -I /usr/local/cuda/include \
    main.cpp \
    -L /usr/local/cuda/lib64 -lnvonnxparser -lnvinfer -lcudart

./main /home/icv/code/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx ../model/resnet50_icv_cpp.plan

https://github.com/cyrusbehr/tensorrt-cpp-api/blob/main/src/engine.cpp
4: [network.cpp::validate::3062] Error Code 4: Internal Error (Network has dynamic or shape inputs, but no optimization profile has been defined.)
2: [builder.cpp::buildSerializedNetwork::751] Error Code 2: Internal Error (Assertion engine != nullptr failed. )
Segmentation fault (core dumped)
