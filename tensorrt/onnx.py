import onnx

# 加载ONNX模型
onnx_model = onnx.load_model('/home/icv/code/workspaces/inf_onsite/test/onnxfiles/3d_det_all.onnx')

# 打印模型的输入信息
for input in onnx_model.graph.input:
    print(f"Input: {input}")

