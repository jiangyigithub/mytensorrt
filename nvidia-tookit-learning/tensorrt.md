# TensorRT学习层次

> * TensorRT简介
>   * TensorRT的基本特性和用法
>   * WorkFlow：使用TensorRT-API搭建`易用性：⭐ 性能：⭐⭐⭐ 兼容：⭐⭐⭐ 不支持op：插件`
>   * WorkFlow：使用Parser `易用性：⭐⭐ 性能：⭐⭐ 兼容：⭐⭐ 不支持op：插件/改网/Parser`
>   * WorkFlow：使用框架内TensorRT接口创建 ` 易用性：⭐⭐⭐ 性能：⭐ 兼容：⭐⭐ 不支持op：返回原框架计算`
> * TensorRT开发辅助工具
>   * trtexec
>   * Netron
>   * polygraph
>   * onnx-graphsurgeon
>   * Nsight Systems
> * 插件书写
>   * 使用插件的简单例子
>   * TensorRT与插件的交互
>   * 关键API
>   * 结合使用Parser和插件
>   * 插件高级
>   * 插件应用
> * TensorRT高级
>   * 多Optimization Profile
>   * 多Stream
>   * 多Context
>   * CUDA Graph
>   * Timing Cache
>   * Algorithm Selector
>   * Refit
>   * Tactic Source
>   * 更多工具

# 一、TensorRT简介

## 1、TensorRT是什么

* 用于高效实现已训练好的深度学习模型的**推理过程**的SDK
* 内含**推理优化器**和**运行时环境**
* 使深度学习模型能以**更高吞吐量**和**更低的延迟**运行
* 有C++和python的API，基本可以等价混用

## 2、相关资料

* TensorRT官方文档：`https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html`
* C++ API文档：`https://docs.nvidia.com/deeplearning/tensorrt/api/c_api`
* python API文档：`https://docs.nvidia.com/deeplearning/tensorrt/api/python_api`
* TensorRT下载：`https://developer.nvidia.com/nvidia-tensorrt-download`
* 中文教程文档：`https://github.com/NVIDIA/trt-samples-for-hackathon-cn/tree/master/cookbook`
* **TensorRT环境**：
  * nvidia-docker：
    * 安装步骤：`https://docs.nvidia.com/datacenter/cloud-native/container-tookit/install-guide.html#docker`
    * 镜像列表：`https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html`
    * 推荐镜像：`nvcr.io/nvidia/tensorrt:21.12-py3`, `nvcr.io/nvidia/pytorch:21.12-py3`

  * python库：
    * `nvidia-pyindex`，`cuda-python`，`onnx`，`onnx-surgeon`，`onnxruntime-gpu`，`opencv-python`，`polygraphy`


## 3、TensorRT的基本特性和用法

* 构建期（推理优化器）
  * 模型解析 / 建立                    加载Onnx等其他格式的模型 / 使用原生API搭建模型
  * 计算图优化                           横向融合（Conv），纵向融合（Conv + BN + ReLU）
  * 节点消除                               去除无用层，节点变换（Pad，Slice，Concat，Shuffle）
  * 多精度支持                           FP32 / FP16 / INT8 / TF32（可能插入reformat节点）
  * 优选kernel / format              硬件有关优化
  * 导入plugin                            实现自定义操作
  * 显存优化                               显存池复用
* 运行期（运行时环境）
  * 运行时环境                           对象生命期管理，内存显存管理，异常处理
  * 序列化反序列化                   推理引擎保存为文件或从文件中加载
* TensorRT的表现
  * 不同模型的加速效果不同
  * 选用高效算子提升运算效率
  * 算子融合提升访存效率
  * 使用低精度数据类型，节约时间空间

## 4、TensorRT基本流程

* 构建期

  * 建立Builder（引擎构建器）
  * 创建Network（计算图）
  * 生成Serialized Network（网络的TRT内部表示）

* 运行期

  * 建立Engine和Context
  * Buffer相关准备（Host端、Device端、拷贝操作）
  * 执行推理（Execute）

* 相应程序

  * ```python
    logger = trt.Logger(trt.Logger.ERROR)                                       # 指定 Logger
    if os.path.isfile(trtFile):                                                 # 如果有 .plan 文件则直接读取
        with open(trtFile, 'rb') as f:
            engineString = f.read()
    else:                                                                       # 没有 .plan 文件，从头开始创建
        builder = trt.Builder(logger)                                           # 建立Builder
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        profile = builder.create_optimization_profile()
        config = builder.create_builder_config()
        config.max_workspace_size = 1 << 30
        
        inputTensor = network.add_input('inputT0', trt.float32, [-1, -1, -1])   # 指定输入张量
        profile.set_shape(inputTensor.name, [1, 1, 1], [3, 4, 5], [6, 8, 10])   # 指定输入张量 Dynamic Shape 范围
        config.add_optimization_profile(profile)
    
        identityLayer = network.add_identity(inputTensor)                       # 恒等变换
        network.mark_output(identityLayer.get_output(0))                        # 标记输出张量
    
        engineString = builder.build_serialized_network(network, config)        # 生成序列化网络
        with open(trtFile, 'wb') as f:                                          # 将序列化网络保存为 .plan 文件
            f.write(engineString)
    
    ########################### 以上是构建期 ############################# 以下是运行期 ###########################
            
    engine = trt.Runtime(logger).deserialize_cuda_engine(engineString)          # 使用 Runtime 来创建 engine
    context = engine.create_execution_context()                                 # 创建 context（相当于 GPU 进程）
    context.set_binding_shape(0, [3, 4, 5])                                     # Dynamic Shape 模式需要绑定真实数
    data = np.arange(np.prod([3, 4, 5]), dtype=np.float32).reshape(3, 4, 5)
    bufferH = [ np.ascontiguousarray(data.reshape(-1)) ]
    bufferD = [ cudart.cudaMalloc(bufferH[0].nbytes)[1] ]
    
    cudart.cudaMemcpy(
        bufferD[0], bufferH[0].ctypes.data, bufferH[0].nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyHostToDevice
    )
    
    context.execute_v2(bufferD)  
    
    cudart.cudaMemcpy(
        bufferH[i].ctypes.data, bufferD[i], bufferH[i].nbytes,
        cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost
    )
    
    cudart.cudaFree(bufferD[0])
    ```

# 二、TensorRT程序设计流程

<img src="tensorrt.assert\trt.png" alt="trt" style="zoom: 23%;" />

## 1、构建阶段

### 第一步：建立Logger（日志记录器）

> ```python
> logger = trt.Logger(trt.Logger.VERBOSE)
> ```
>
> 可选参数：`VERBOSE`、`INFO`、`WARNING`、`ERROR`、`INTERNAL_ERROR`，产生不同等级的日志，由详细到简略

### 第二步：建立Builder（网络元数据）和 BuilderConfig（网络元数据的选项）

> ```python
> builder = trt.Builder(logger)
> ```
>
> 目前不再推荐使用builder本身来进行引擎构建器的配置，尤其是需求动态形状的情况下，更推荐使用buderConfig及相关API
>
> ```python
> config = builder.create_builder_config()
> ```
>
> 常用成员包括：
>
> * `config.max_workspace_size = 1 << 30`：指定构建期可用现存，单位为Byte
> * `[X] config.max_batch_size = `：已弃用，采用Explicit Batch模式了
> * `config.flag = 1 << int(trt.BuilderFlag.FP16)` ：设置精度标志位
> * `config.int8_calibrator = `：设置calibrator
> * `config.set_tactic_sources` / `set_timing_cache` / `algorithm_selector` ...... 更多高级用法
> * 注意：动态形状下不可用builder.max_workspace_size

### 第三步：创建Network（网络计算图）

> ```python
> network = builder.create_network()
> ```
>
> 常用参数：`1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)`
>
> 常用方法：
>
> * `network.add_input('one_tensor', trt.float32, (3, 4, 5))`：标记网络输入张良
> * `convLayer = network.add_convolution_nd(xxx)`：添加网络的各种层
> * `network.mark_output(convLayer.get_output(0))`：标记网络的输出张量
>
> 常用的获取网络信息的成员：
>
> * `network.name` `network.num_layers` `network.num_inputs` `network.num_outputs`
> *  `network.has_implicit_batch_dimension` `network.has_explicit_precision`
>
> > #### Explicit  VS  Implicit
>
> * Explicit Batch 将成为主流的构建方法
> * Explicit Batch 要求显示的包含batch维度，比Implicit模式高出一个维度
> * 使用Explicit：`builder.create_network(1<<int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))`
> * Explicit相比Implicit能做更多事情：
>   * Layer Norm
>   * Reshape / Transpose / Reduce over batch dimension
>   * Dynamic Shape 模式
>   * Loop 结构
>   * 一些Layer的高级用法（例如：ShuffleLayer.set_input）
> * 从Onnx导入的模型默认就是Explicit模式的，其兼容性更好
>
> > #### Dynamic Shape
>
> * 适用于输入张量的形状在推理时才决定的网络
> * 除了Batch维度，其他维度也可以在推理时才确定
> * 需要Explicit模式
> * 需要Optimization Profile帮助网络优化
> * 需要context.set_binding_shape绑定实际输入的数据形状
>
> > #### Profile
>
> ```python
> profile = builder.create_optimization_profile()
> ```
>
> * `profile.set_shape(tensorName, minShape, commonShape, maxShape)`
> * `config.add_optimization_profile(profile)`
>
> > #### 使用Dynamic Shape & Explicit Batch & Profile的例子
>
> ```python
> network = builder.create_network(1<<int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
> profile = buikder.create_optimization_profile()
> config  = builder.create_builder_config()
> 
> inputTensor = network.add_input('inputT0', trt.DataType.FLOAT, [-1, -1, -1])
> profile.set_shape(inputTensor.name, (1, 1, 1), (3, 4, 5), (6, 8, 10))
> config.add_optimization_profile(profile)
> 
> # ...
> 
> context.set_binding_shape(0, [3, 4, 5])
> 
> # ...
> 
> outputH0 = np.empty(context.get_binding_shape(1), dtype=trt.nptype(engine.get_binding_dtype(1)))
> 
> # ...
> 
> context.execute_async_v2([int(inputD0), int(outputD0)], stream)
> ```
>
> > #### 注意Layer和Tensor的区别
>
> * `oneLayer = network.add_identity(inputTensor)`
> * `ontTensor = oneLayer.get_output(0)`
>
> layer的常用成员和方法：
>
> * `oneLayer.name = "one"`   获取或给层起名字
> * `oneLayer.type`    获取该层的类型
> * `oneLayer.precision`    指定该层的计算精度，需要配合`builder.strict_type_constrains`
> * `oneLayer.get_output(i)`    获取该层的第二个输出张量
>
> Tensor的常用成员和方法：
>
> * `oneTensor.name = "ot"`      获取或给张量起名字
> * `oneTensor.shape`     获取tensor的形状
> * `oneTensor.dtype`     获取或设定tensor的数据类型
>
> 可以打印所有层和张量的信息，相应程序如下：
>
> ```python
> for i in range(network.num_layers):
>     layer = network.get_layer(i)
>     print(i, "%s, in=%d, out=%d, %s"%(str(layer.type)[10:], layer.num_inputs, layer.num_outputs, layer.name))
>     for j in range(layer.num_inputs):
>         tensor = layer.get_input(j)
>         if tensor is None:
>             print("\tInput %2d:"%j, "None")
>         else:
>             print("\tInput %2d:%s,%s,%s"%(j, tensor.shape, str(tensor.dtype)[9:], tensor.name))
>     for j in range(layer.num_outputs):
>         tensor = layer.get_output(j)
>         if tensor is None:
>             print("\tOutput %2d:"%j, "None")
>         else:
>             print("\tOutput %2d:%s,%s,%s"%(j, tensor.shape, str(tensor.dtype)[9:], tensor.name))        
> ```
>
> > #### 使用Parser进行网络的构建
>
> pytorch-onnx-tensorrt-int8：`04-Parser/02-pyTorch-ONNX-TensorRT`(int8需要额外实现calibrator类)
>
> >  #### 使用框架自带trt转换工具
>
> pytorch-tensorrt：`07-Framework/Torch-TensorRT`
>
> > #### 精度量化
>
> * FP16模式
>   * 案例：`04-Parser/TensorFlow-ONNX-TensorRT`
>   * `config.flags = 1 << int(trt.BuilderFlag.FP16)`
>   * FP16模式的engine建立时间比FP32长，可能需要在各个源节点间插入Reformat节点
>   * Timeline中出现nchwToNchw等kernel调用
>   * 部分层可能由于精度的下降导致误差较大：
>     * 找到误差较大的层（使用polygraphy等工具）
>     * 强制该层使用FP32进行计算
>       * `config.flags = 1 << int(trt.BuilderFlag.STRICT_TYPES)`
>       * `layer.precision = trt.float32`
> * INT8模式——PTQ
>   * 案例：`04-Parser/Pytorch-ONNX-TensorRT`
>   * 需要有校准集：输入范例数据
>   * 需要实现calibrator
>   * `config.flags = 1 << int(trt.BuilderFlag.INT8)`
>   * `config.int8_calibrator = ...`
>
> * INT8模式——QAT
>   * 案例：`02-API/int8-QAT`; `04-Parser/Pytorch-ONNX-TensorRT-QAT`
>   * `config.flags = 1 << int(trt.BuilderFlag.INT8)`
>   * Pytorch原网络中就需要进行 Quantize / Dequantize 层的使用

### 第四步：生成Serialized-Network（网络的TRT内部表示）

> 将序列化后的网络保存为文件，下一次跳过构建阶段，可以直接使用
>
> ```python
> if os.path.isfile(trtFile):
>     with open(trtFile, 'rb') as f:
>         serializedNetwork = f.read()
> else:
> 	# ... 构建网络
>     serializedNetwork = builder.build_serialized_network(network, config)
>     with open(trtFile, 'wb') as f:
>         f.write(serializedNetwork)
> ```
>
> 使用serializedNetwork可以创建引擎，引擎中包含了关于硬件的优化，不能跨平台使用，相同平台多次生成的engine也可能不相同。
>
> 可以使用AlgorithmSelector和TimingCache多次生成相同的engine

## 2、运行阶段

### 第一步：建立Engine

> ```python
> engine = trt.Runtime(logger).deserialize_cuda_engine(serializedNetwork)
> ```
>
> 常用成员：
>
> * `engine.num_bindings`：获取engine绑定的输入输出的总数量（n+m）
> * `engine.max_batch_size`：获取engine的最大batch size，explicit模式下=1
> * `engine.num_layers`：获取engine（自动优化后）的总层数
>
> 常用方法：
>
> * `engine.get_binding_dtype(i)`：第i个绑定张量的数据类型0~n-1为输入；n~n+m-1为输出
> * `engine.get_binding_shape(i)`：第i个绑定张量的形状，动态形状下可能会出现-1
> * `engine.binding_is_input(i)`：第i个绑定张量是否是输入张量
> * `engine.get_binding_index('n')`：名字叫‘n’的张量在engine中的索引编号

### 第二步：创建Context

> ```python
> context = engine.create_execute_context()
> ```
>
> 常用成员：
>
> * `context.all_binding_shapes_specified`：确认所有的bindings都绑定了(尤其动态形状需要确认)
>
> 常用方法：
>
> * `context.set_binding_shape(i)`：设置第i个绑定张量的形状（动态形状中要使用）
> * `context.get_binding_shape(i)`：获取第i个绑定张量的形状
> * `context.execute_v2(listOfBuffer)`：Explicit batch模式的同步推理
> * `context.execute_async_v2(listOfBuffer, stream)`：Explicit batch模式的异步推理

### 第三步：绑定输入输出

> ```python
> context.set_binding_shape(0, [1, 3, 56, 56])
> ```
>
> * engine / context给所有的输入输出张量安排了位置
> * 一共有`engine.num_bindings`个binding，输入张量在前，输出张量在后
> * 假设输入包括：`x[B,C,H,W](float32)`；`y[B,256](int32)`，输出包括：`index[B,](int32)`；`entropy[B,](float32)`
>   * engine会按照四者在计算图中的拓扑顺序为其进行binding
>   * 运行期要按照位置绑定(输入)张量的形状，假设位置为`[0号=x, 1号=y, 2号=index, 3号=emtropy]`
>     * `context.set_binding_shape(0, [4, 3, 45, 45])`
>     * `context.set_binding_shape(1, [4, 256])`
>     * 输出张量形状会自动计算：`(-1,)、(-1,) --> (4,)、(4,)

### 第四步：Buffer准备（Host端+Device端）

```python
inputHost = np.ascontigiousarray(inputData.reshape(-1))
outputHost = np.empty(context.get_binding_shape(1), trt.nptype(engine.get_binding_dtype(1)))
inputDevice = cudart.cudaMalloc(inputHost.nbytes)[1]
outputDevice = cudart.cudaMalloc(outputHost.nbytes)[1]
```

### 第五步：Buffer拷贝（Host to Device）

```python
cudart.cudaMemcpy(
    inputDevice,										# 拷贝给谁
    inputHost.ctypes.data,								# 被拷贝的是谁
    inputHost.nbytes, 									# 拷贝了多长
    cudart.cudaMemcpyKind.cudaMemcpyHostToDevice		# 什么类型的拷贝
)
```

### 第六步：执行推理（Execute）

```python
context.execute_v2([int(inputDevice), int(outputDevice)])
```

### 第七步：Buffer拷贝（Device to Host）

```python
cudart.cudaMemcpy(
    outputHost.ctypes.data,								# 拷贝给谁
    outputDevice,										# 被拷贝的是谁
    outputHost.nbytes, 									# 拷贝了多长
    cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost		# 什么类型的拷贝
)
```

### 第八步：善后工作

```python
# 推理完成后释放显存
cudart.cudaFree(inputDevice)
cudart.cudaFree(outputDevice)
```

# 三、开发辅助工具

## 1、希望解决的问题

* 不想写脚本来跑tensorrt：命令行工具
* 怎样进行简单的推理性能(测量延迟、吞吐量等)测试？
* 网络结构可视化？
* 计算图上存在某些节点阻碍TRT自动优化：手工调整
* 怎么处理TRT不支持的网络结构：手工调整
* 怎么检验TRT上计算结果的正确性/精度：对比原框架和TRT的计算结果
* 怎么找出计算错误/精度不足的层：模型逐层比较
* 怎么进行简单的计算图优化：手工调整
* 怎么找出最耗时的层：找到热点集中优化

## 2、trtexec

> trt的命令行工具，位于tensorrt-版本/bin/trtexec
>
> * 可以从.onnx模型文件生成TRT引擎并序列化为.plan文件
> * 查看.onnx或.plan文件的网络逐层信息
> * 模型性能测试（测试trt引擎基于随机输入或给定输入下的性能）
>
> **构建阶段常用选项**
>
> * `--onnx=./model.onnx` ：指定输入模型文件名
> * ~~`--output=y:0`~~ ：指定输出张量名(onnx模型该选项无效)
> * `--minShapes=x:0:1x1x28x28 --optShapes=x:0:4x1x28x28 --maxShapes=x:0:16x1x28x28` ：指定最小;合适;最大形状
> * `--workspace=1024`(trt8.4以后要用`--memPoolSize`) ：指定最大显存占用(MiB)
> * `--fp16`  / ` --int8`  /  `--noTF32`  /  `--best`  /  `--sparsity=xxx` ：指定引擎精度和稀疏性等属性
> * `--saveEngine=./model.plan` ：指定输出序列化引擎文件名
> * `--buildOnly` ：只构建引擎不推理
> * `--verbose` ：打印详细信息
> * `--timingCacheFile=timing.cache` ：指定输出优化计时缓存文件名
> * `--profilingVerbosity=detailed` ：构建期保留更多的逐层信息
> * `--dumpLayerInfo`， `--exportLayerInfo=LayerInfo.txt` ：导出引擎逐层信息，可与profilingVerbosity合用
>
> **推理期常用选项**
>
> * `--loadEngine=model.plan` ：读取plan文件，而不是onnx文件
> * `--shapes=x:0:4x1x28x28` ：指定输入形状
> * `--warmUp=1000` ：热身阶段最短运行时间（ms）
> * `--duration=10` ：测试阶段最短运行时间（s）
> * `--iteration=100` ：指定测试阶段运行的最小迭代次数
> * `--useCudaGraph` ：使用该工具来捕获执行推理的过程
> * `--noDataTransfers` ：关闭Host和Device间的数据传输
> * `--streams=2` ：使用多个stream来进行推理
> * `--verbose` ：打印详细信息
> * `--dumpProfile`， `--exportProfile=LayerProfile.txt` ：保存逐层性能数据信息

**例子：08-Tool/trtexec/command.sh**

```bash
clear
rm ./*.pb ./*.onnx ./*.plan ./result-*.txt

# 从tf创建一个onnx用于作为trtexec的输入文件
python3 getOnnxModel.py

# 构建一个trt引擎并推理
trtexec \
	--onnx=model.onnx \ 
	--minShapes=x:0:1x1x28x28 \ 
	--optShapes=x:0:4x1x28x28 \ 
	--maxShapes=x:0:16x1x28x28 \ 
	--workspace=1024 \ 
	--saveEngine=model-fp32.plan \ 
	--shapes=x:0:4x1x28x28
	--verbose \ 
	> result-fp32.txt
	
# 构建一个trt引擎并推理, 其模式为FP16
trtexec \
	--onnx=model.onnx \ 
	--minShapes=x:0:1x1x28x28 \ 
	--optShapes=x:0:4x1x28x28 \ 
	--maxShapes=x:0:16x1x28x28 \ 
	--workspace=1024 \ 
	--saveEngine=model-fp16.plan \ 
	--shapes=x:0:4x1x28x28
	--verbose \ 
	--fp16 \
	> result-fp16.txt

# 读取model-fp32.plan再推理
trtexec \
	--loadEngine=./model-fp32.plan \ 
	--shapes=x:0:4x1x28x28 \ 
	--verbose \ 
	> result-load-fp32.txt
```

**性能测试结果如下**

![image-20220708111645186](tensorrt.assert\image-20220708111645186.png)

## 3、Netron模型可视化

> https://github.com/lutzoeder/Netron
>
> * 查看网络结构
> * 查看计算图信息
> * 查看节点信息

## 4、onnx-graphsurgeon手动修改网络

> **例子：10-BestPractice ; 08-Tool/OnnxGraphSurgeon/test.sh(9个例子：模型创建、隔离子图、替换节点、常量折叠、删除节点、**
>
> **shape操作等)**
>
> **功能**
>
> * 修改计算图：图属性 / 节点 / 张量 / 节点和张量的连接 / 权重
> * 修改子图：添加 / 删除 / 替换/ 隔离
> * 优化计算图：常量折叠 / 拓扑排序 / 去除无用层
> * （功能和API上和onnx库有区别）
>
> > https://github.com/NVIDIA/TensorRT/tree/master/tools/onnx-graphsurgeon/examples
> >
> > https://docs.nvidia.com/deeplearning/tensorrt/onnx-grapgsurgeon.docs/index.html

**例1：网络的搭建**

```python
from collections import OrderedDict
import numpy as np
import onnx
import onnx_graphsurgeon as gs

tensor0 = gs.Variable(name="tensor0", dtype=np.float32, shape=['B', 3, 64, 64])  # 定义张量（变量）
tensor1 = gs.Variable(name="tensor1", dtype=np.float32, shape=['B', 1, 64, 64])
tensor2 = gs.Variable(name="tensor2", dtype=np.float32, shape=None)  # 可以不知道形状或者数据类型
tensor3 = gs.Variable(name="tensor3", dtype=np.float32, shape=None)

constant0 = gs.Constant(name="constant0", values=np.ones(shape=[1, 3, 3, 3], dtype=np.float32))  # 定义张量（常量）
constant1 = gs.Constant(name="constant1", values=np.ones(shape=[1], dtype=np.float32))

# 定义节点，使用张量作为输入和输出
node0 = gs.Node(name="myConv", op="Conv", inputs=[tensor0, constant0], outputs=[tensor1])  
node0.attrs = OrderedDict([
    ['dilations', [1, 1]],
    ['kernel_shape', [3, 3]],
    ['pads', [1, 1, 1, 1]],
    ['strides', [1, 1]],
])  # 节点的属性参数

node1 = gs.Node(name="myAdd", op="Add", inputs=[tensor1, constant1], outputs=[tensor2])
node2 = gs.Node(name="myRelu", op="Relu", inputs=[tensor2], outputs=[tensor3])

# 定义计算图，要求给出所有节点和输入输出张量
graph = gs.Graph(nodes=[node0, node1, node2], inputs=[tensor0], outputs=[tensor3])  

graph.cleanup().toposort()  # 保存计算图前的收尾工作，详细作用见 06-Fold.py
onnx.save(gs.export_onnx(graph), "model-01.onnx")
```

**例2：图中插入新节点**

```python
############################### 创建一个图: Identity -- Identity ################################
tensor0 = gs.Variable(name="tensor0", dtype=np.float32, shape=['B', 3, 64, 64])
tensor1 = gs.Variable(name="tensor1", dtype=np.float32, shape=None)
tensor2 = gs.Variable(name="tensor2", dtype=np.float32, shape=None)

node0 = gs.Node(name="myIdentity0", op="Identity", inputs=[tensor0], outputs=[tensor1])
node1 = gs.Node(name="myIdentity1", op="Identity", inputs=[tensor1], outputs=[tensor2])

graph = gs.Graph(nodes=[node0, node1], inputs=[tensor0], outputs=[tensor2])
graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-02-01.onnx")

############################### 再上图的基础上添加一个新的节点: Add ################################
for node in graph.nodes:
    if node.op == 'Identity' and node.name == 'myIdentity0':  # 遍历计算图找到需要添加节点的位置
        # 构造新节点和新张量: 第一个Identity的输出结果和常量相加
        constant0 = gs.Constant(name="constant0", values=np.ones(shape=[1, 1, 1, 1], dtype=np.float32))  
        tensor3 = gs.Variable(name="tensor3", dtype=np.float32, shape=None)
        newNode = gs.Node(name="myAdd", op="Add", inputs=[node.outputs[0], constant0], outputs=[tensor3])

        graph.nodes.append(newNode)  # 记得把新节点加入计算图中
        index = node.o().inputs.index(node.outputs[0])  # 小心地找到下一个节点中对应输入张量的位置
        node.o().inputs[index] = tensor3  # 替换为新张量

graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-02-02.onnx")
```

**例3：删除图中的节点**

```python
tensor0 = gs.Variable(name="tensor0", dtype=np.float32, shape=['B', 3, 64, 64])
tensor1 = gs.Variable(name="tensor1", dtype=np.float32, shape=None)
tensor2 = gs.Variable(name="tensor2", dtype=np.float32, shape=None)
tensor3 = gs.Variable(name="tensor3", dtype=np.float32, shape=None)

constant0 = gs.Constant(name="constant0", values=np.ones(shape=[1, 1, 1, 1], dtype=np.float32))

node0 = gs.Node(name="myIdentity0", op="Identity", inputs=[tensor0], outputs=[tensor1])
node1 = gs.Node(name="myAdd", op="Add", inputs=[tensor1, constant0], outputs=[tensor2])
node2 = gs.Node(name="myIdentity1", op="Identity", inputs=[tensor2], outputs=[tensor3])

graph = gs.Graph(nodes=[node0, node1, node2], inputs=[tensor0], outputs=[tensor3])
graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-03-01.onnx")

################################ 构建好一个例2最终的图, 删掉Add节点 ###############################

for node in graph.nodes:
    if node.op == 'Add' and node.name == 'myAdd':
        index = node.o().inputs.index(node.outputs[0])  # 小心地找到下一个节点中该张量的位置
        node.o().inputs[index] = node.inputs[0]  # 把下一节点的对应输入张量赋为 Add 节点的输入张量
        node.outputs = []  # 关键操作：将 Add 节点的输出张量设置为空，这样 Add 节点就成为无用节点，可以被自动清理删掉

graph.cleanup().toposort()  # 在清理时会自动删掉 Add 节点
onnx.save(gs.export_onnx(graph), "model-03-02.onnx")
```

**例4：替换节点**

```python
tensor0 = gs.Variable(name="tensor0", dtype=np.float32, shape=['B', 3, 64, 64])
tensor1 = gs.Variable(name="tensor1", dtype=np.float32, shape=None)
tensor2 = gs.Variable(name="tensor2", dtype=np.float32, shape=None)
tensor3 = gs.Variable(name="tensor3", dtype=np.float32, shape=None)

constant0 = gs.Constant(name="constant0", values=np.ones(shape=[1, 1, 1, 1], dtype=np.float32))

node0 = gs.Node(name="myIdentity0", op="Identity", inputs=[tensor0], outputs=[tensor1])
node1 = gs.Node(name="myAdd", op="Add", inputs=[tensor1, constant0], outputs=[tensor2])
node2 = gs.Node(name="myIdentity1", op="Identity", inputs=[tensor2], outputs=[tensor3])

graph0 = gs.Graph(nodes=[node0, node1, node2], inputs=[tensor0], outputs=[tensor3])
graph0.cleanup().toposort()
onnx.save(gs.export_onnx(graph0), "model-04-01.onnx")

################################ 构建好一个例2最终的图, 替换Add节点 ###############################

# Add 转 Sub属于特殊操作，第一种方法如下：
graph1 = graph0.copy()
for node in graph1.nodes:
    if node.op == 'Add' and node.name == 'myAdd':
        node.op = 'Sub'  # 通过修改操作类型来替换节点
        node.name = 'mySub'  # 名字该改不改都行，主要是方便区分节点以及日后查找
graph1.cleanup().toposort()
onnx.save(gs.export_onnx(graph1), "model-04-02.onnx")

# 第二种方法
graph2 = graph0.copy()
for node in graph2.nodes:
    if node.op == 'Add' and node.name == 'myAdd':
        newNode = gs.Node(name="mySub", op="Sub", inputs=node.inputs, outputs=node.outputs)  # 照搬输入输出张量
        graph2.nodes.append(newNode)  # 把新节点加入计算图中
        node.outputs = []  # 将原节点的输出张量设置为空
graph2.cleanup().toposort()
onnx.save(gs.export_onnx(graph2), "model-04-03.onnx")
```

**例5：遍历图中的节点和张量**

```python
print("遍历节点，打印：节点信息，输入张量，输出张量，父节点名，子节点名")
for index, node in enumerate(graph.nodes):
    print("Node%4d: op=%s, name=%s, attrs=%s" % (index, node.op, node.name, \
		  "".join(["{"] + [str(key) + ":" + str(value) + ", " for key, value in node.attrs.items()] + ["}"])))
    for jndex, inputTensor in enumerate(node.inputs):
        print("\tInTensor  %d: %s" % (jndex, inputTensor))
    for jndex, outputTensor in enumerate(node.outputs):
        print("\tOutTensor %d: %s" % (jndex, outputTensor))

    fatherNodeList = []
    for i in range(nMaxAdjustNode):
        try:
            newNode = node.i(i)
            fatherNodeList.append(newNode)
        except:
            break
    for jndex, newNode in enumerate(fatherNodeList):
        print("\tFatherNode%d: %s" % (jndex, newNode.name))

    sonNodeList = []
    for i in range(nMaxAdjustNode):
        try:
            newNode = node.o(i)
            sonNodeList.append(newNode)
        except:
            break
    for jndex, newNode in enumerate(sonNodeList):
        print("\tSonNode   %d: %s" % (jndex, newNode.name))
        
        
print("遍历张量，打印：张量信息，以本张量作为输入张量的节点名，以本张量作为输出张量的节点名，父张量信息，子张量信息")
for index, (name, tensor) in enumerate(graph.tensors().items()):
    print("Tensor%4d: name=%s, desc=%s" % (index, name, tensor))
    for jndex, inputNode in enumerate(tensor.inputs):
        print("\tInNode      %d: %s" % (jndex, inputNode.name))
    for jndex, outputNode in enumerate(tensor.outputs):
        print("\tOutNode     %d: %s" % (jndex, outputNode.name))

    fatherTensorList = []
    for i in range(nMaxAdjustNode):
        try:
            newTensor = tensor.i(i)
            fatherTensorList.append(newTensor)
        except:
            break
    for jndex, newTensor in enumerate(fatherTensorList):
        print("\tFatherTensor%d: %s" % (jndex, newTensor))

    sonTensorList = []
    for i in range(nMaxAdjustNode):
        try:
            newTensor = tensor.o(i)
            sonTensorList.append(newTensor)
        except:
            break
    for jndex, newTensor in enumerate(sonTensorList):
        print("\tSonTensor   %d: %s" % (jndex, newTensor))
```

**例6：图的折叠**

```python
tensor0 = gs.Variable(name="tensor0", dtype=np.float32, shape=['B', 3, 64, 64])  # 三个真正有用的张量
tensor1 = gs.Variable(name="tensor1", dtype=np.float32, shape=['B', 3, 64, 64])
tensor2 = gs.Variable(name="tensor2", dtype=np.float32, shape=['B', 3, 64, 64])

tensor3 = gs.Variable(name="tensor3", dtype=np.float32, shape=['B', 3, 64, 64])  # 一个假输入张量
tensor4 = gs.Variable(name="tensor4", dtype=np.float32, shape=['B', 1, 64, 64])  # 一个假输出张量
tensor5 = gs.Variable(name="tensor5", dtype=np.float32, shape=['B', 1, 64, 64])  # 两个无用张量
tensor6 = gs.Variable(name="tensor6", dtype=np.float32, shape=['B', 1, 64, 64])

tensor7 = gs.Variable(name="tensor7", dtype=np.float32, shape=None)  # 中间结果张量
tensor8 = gs.Variable(name="tensor8", dtype=np.float32, shape=None)

constant0 = gs.Constant(name="w", values=np.ones(shape=[1, 1, 1, 1], dtype=np.float32))

node0 = gs.Node(name="myAdd0", op="Add", inputs=[constant0, constant0], outputs=[tensor7])
node1 = gs.Node(name="myAdd1", op="Add", inputs=[tensor7, constant0], outputs=[tensor8])
node2 = gs.Node(name="myAdd2", op="Add", inputs=[tensor0, tensor8], outputs=[tensor1])  # 有效节点
node3 = gs.Node(name="myAdd3", op="Add", inputs=[tensor1, constant0], outputs=[tensor2])  # 有效节点
node4 = gs.Node(name="myAdd4", op="Add", inputs=[tensor5, constant0], outputs=[tensor6])  # 无效节点

graph = gs.Graph(nodes=[node4, node3, node2, node1, node0], inputs=[tensor0, tensor3], outputs=[tensor2, tensor4])

onnx.save(gs.export_onnx(graph), "model-06-01.onnx")  # 原始计算图: 4 个无边张量和 1 个无边的节点，还有 1 个常数计算链
onnx.save(gs.export_onnx(graph.fold_constants()), "model-06-02.onnx")  # 常数折叠后的计算图，常数计算链合并到主链中，多出 2 个无边 Add 节点，注意常数折叠并不做节点融合的工作，主链上两个 Add 没有合并掉
onnx.save(gs.export_onnx(graph.fold_constants().cleanup()), "model-06-03.onnx")  # 打扫后的计算图，可见 3 个无用的 Add 节点被清除

print("Before toposort:")  # 原始节点顺序
for index, node in enumerate(graph.nodes):
    print("No.%d->%s" % (index, node.name))

print("After toposort:")  # 拓扑排序后的节点顺序，节点基本按照计算图的计算顺序进行排列
graph.cleanup().toposort()
for index, node in enumerate(graph.nodes):
    print("No.%d->%s" % (index, node.name))

graph.inputs = [tensor0]
graph.outputs = [tensor2]
onnx.save(gs.export_onnx(graph), "model-06-04.onnx")  # 去掉多与输入输出的计算图，才能正确被 TensorRT 处理
```

**例7：形状简化**

```python
tensor0 = gs.Variable("tensor0", np.float32, ['A', 3, 'B', 5])
tensor1 = gs.Variable("tensor1", np.int64, None)
tensor2 = gs.Variable("tensor2", np.int64, None)
tensor3 = gs.Variable("tensor3", np.float32, None)
tensor4 = gs.Variable("tensor4", np.int64, None)
tensor5 = gs.Variable("tensor5", np.int64, None)
tensor6 = gs.Variable("tensor6", np.int64, None)
tensor7 = gs.Variable("tensor7", np.int64, None)
tensor8 = gs.Variable("tensor8", np.float32, None)

constant0 = gs.Constant("constant0", values=np.array([0, 1], dtype=np.int32))  # 定义张量（常量）
constant1 = gs.Constant("constant1", values=np.array([2, 3], dtype=np.int32))

node0 = gs.Node("Shape", "myShape", inputs=[tensor0], outputs=[tensor1])  # value=(A,3,B,5), shape=(4,)
node1 = gs.Node("ReduceProd", "myReduceProd0", inputs=[tensor1], attrs={"axes": [0], "keepdims": int(True)}, outputs=[tensor2])  # value=(A*3*B*5), shape=()
node2 = gs.Node("Reshape", "myReshape0", inputs=[tensor0, tensor2], outputs=[tensor3])  # shape=(A*3*B*5,)

node3 = gs.Node("Gather", "myGather0", inputs=[tensor1, constant0], outputs=[tensor4])  # value=(A,3), shape=(2,)
node4 = gs.Node("Gather", "myGather1", inputs=[tensor1, constant1], outputs=[tensor5])  # value=(B,5), shape=(2,)
node5 = gs.Node("ReduceProd", "myReduceProd1", inputs=[tensor5], attrs={"axes": [0], "keepdims": int(True)}, outputs=[tensor6])  # value=(B*5), shape=()
node6 = gs.Node("Concat", "myConcat", inputs=[tensor4, tensor6], attrs={"axis": 0}, outputs=[tensor7])  # value=(A,3,B*5), shape=()
node7 = gs.Node("Reshape", "myReshape1", inputs=[tensor0, tensor7], outputs=[tensor8])  # shape=(A*3*B*5,)

graph = gs.Graph(nodes=[node0, node1, node2, node3, node4, node5, node6, node7], inputs=[tensor0], outputs=[tensor3, tensor8])

graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-07-01.onnx")

graph.inputs[0].shape = [2, 3, 4, 5]  # 如果是 static shape，则 fold_constants 可以化简计算图
graph.fold_constants().cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-07-02.onnx")
```

**例8：隔离计算图**

```python
################################## 把Add操作“拎”出来 ######################################
tensor0 = gs.Variable(name="tensor0", dtype=np.float32, shape=['B', 3, 64, 64])
tensor1 = gs.Variable(name="tensor1", dtype=np.float32, shape=['B', 3, 64, 64])
tensor2 = gs.Variable(name="tensor2", dtype=np.float32, shape=['B', 3, 64, 64])
tensor3 = gs.Variable(name="tensor3", dtype=np.float32, shape=['B', 3, 64, 64])

constant0 = gs.Constant(name="constant0", values=np.ones(shape=[1, 1, 1, 1], dtype=np.float32))

node0 = gs.Node(name="myIdentity0", op="Identity", inputs=[tensor0], outputs=[tensor1])
node1 = gs.Node(name="myAdd", op="Add", inputs=[tensor1, constant0], outputs=[tensor2])
node2 = gs.Node(name="myIdentity1", op="Identity", inputs=[tensor2], outputs=[tensor3])

graph = gs.Graph(nodes=[node0, node1, node2], inputs=[tensor0], outputs=[tensor3])
graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-08-01.onnx")

for node in graph.nodes:
    if node.op == 'Add' and node.name == 'myAdd':
        graph.inputs = [node.inputs[0]]
        graph.outputs = node.outputs

graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-08-02.onnx")
```

**例9：使用API创建模型**

```python
# 创建节点
# 使用 onnx_graphsurgeon.Graph.register() 将一个函数注册为计算图
@gs.Graph.register()
def add(self, a, b):
    return self.layer(op="Add", inputs=[a, b], outputs=["myAdd"])

@gs.Graph.register()
def mul(self, a, b):
    return self.layer(op="Mul", inputs=[a, b], outputs=["myMul"])

@gs.Graph.register()
def gemm(self, a, b, isTransposeA=False, isTransposeB=False):
    attrs = {"transA": int(isTransposeA), "transB": int(isTransposeB)}
    return self.layer(op="Gemm", inputs=[a, b], outputs=["myGgemm"], attrs=attrs)

@gs.Graph.register()
def min(self, *args):
    return self.layer(op="Min", inputs=args, outputs=["myMin"])

@gs.Graph.register()
def max(self, *args):
    return self.layer(op="Max", inputs=args, outputs=["myMax"])

# 注册函数时注明其版本
@gs.Graph.register(opsets=[11])
def relu(self, a):
    return self.layer(op="Relu", inputs=[a], outputs=["myReLU"])

# 注册其他版本的同名函数，在 graph 创建时只会选用指定版本的函数
@gs.Graph.register(opsets=[1])
def relu(self, a):
    raise NotImplementedError("This function has not been implemented!")

# 创建计算图
graph = gs.Graph(opset=11)
tensor0 = gs.Variable(name="tensor0", shape=[64, 64], dtype=np.float32)
#tensor1 = np.ones(shape=(64, 64), dtype=np.float32) # 可以直接使用 np.array，但是张量名字会由 onnx 自动生成
tensor1 = gs.Constant(name="tensor1", values=np.ones(shape=(64, 64), dtype=np.float32))
tensor2 = gs.Constant(name="tensor2", values=np.ones((64, 64), dtype=np.float32) * 0.5)
tensor3 = gs.Constant(name="tensor3", values=np.ones(shape=[64, 64], dtype=np.float32))
tensor4 = gs.Constant(name="tensor4", values=np.array([3], dtype=np.float32))
tensor5 = gs.Constant(name="tensor5", values=np.array([-3], dtype=np.float32))

node0 = graph.gemm(tensor1, tensor0, isTransposeB=True)
node1 = graph.add(*node0, tensor2)
node2 = graph.relu(*node1)
node3 = graph.mul(*node2, tensor3)
node4 = graph.min(*node3, tensor4)
node5 = graph.max(*node4, tensor5)

graph.inputs = [tensor0]
graph.outputs = [node5[0]]

graph.inputs[0].dtype = np.dtype(np.float32)
graph.outputs[0].dtype = np.dtype(np.float32)

onnx.save(gs.export_onnx(graph), "model-09-01.onnx")

@gs.Graph.register()
def replaceWithClip(self, inputs, outputs):
    # 砍掉末尾节点的输出张量和头节点的输入张量
    for inp in inputs:
        inp.outputs.clear()
    for out in outputs:
        out.inputs.clear()
    # 插入新节点
    return self.layer(op="Clip", inputs=inputs, outputs=outputs)

tmap = graph.tensors()

# 手工找出要砍掉的输入和输出张量，交给 replaceWithClip 函数
inputs = [tmap["myMul_6"], tmap["tensor5"], tmap["tensor4"]]
outputs = [tmap["myMax_10"]]
graph.replaceWithClip(inputs, outputs)

graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), "model-09-02.onnx")
```

## 5、深度学习模型调试器Polygraphy

> > https://docs.nvidia.com/deeplearning/tensorrt/polygraphy/docs/index.html
> >
> > https://www.nvidia.com/en-us/on-demand/session/gtcspring21-s31695
>
> * 使用多种后端运行推理计算，包括TRT、OnnxRuntime、TF
> * 比较不同后端的逐层计算结果
> * 由模型生成TRT-engine并序列化为plan文件
> * 查看模型网络的逐层信息
> * 修改Onnx模型，如提取子图、计算图化简等
> * 分析Onnx转TRT失败的原因，将原计算图中可以 / 不可以被转化为TRT的部分分离保存
> * 隔离TRT中的错误tactic

### 1、RUN模式

> `--model-type` ：输入模型文件类型，可用frozen、keras、ckpt、onnx、engine等
>
> `--verbose` ：打印详细日志
>
> `--trt --tf --onnxrt `：选用的后端，可以指定多个
>
> `--input-shapes "x:[4,1,28,28]" "y:[4,8]"` ：实际推理时数据形状，和trtexec的格式有区别
>
> `--save-onnx "./model.onnx" --opset 13` ：指定导出的onnx文件名和算子集编号
>
> `--shape-inference` ：启用onnx的形状推理功能
>
> `--trt-min-shapes 'x:[1,1,28,28]' --trt-opt-shapes 'x:[4,1,28,28]' --trt-max-shapes 'x:[16,1,28,28]'` 
>
> `--fp16 --int8 --tf32 --sparse-weights` ：指定trt的计算精度和稀疏性
>
> `--workspace 1G`
>
> `--rtol 1e-6 --atol 1e-6 --validate` ：计较计算结果时的相对误差和绝对误差上限，并检查NaN和Inf的存在
>
> `--save-engine "./model.plan"`

```bash
clear 
rm ./*.pb ./*.onnx ./*.plan ./result-*.txt
python getOnnxModel.py  # 从 TensorFlow 创建一个 .onnx 用来做 polygraphy 的输入文件

# 01 用上面的 .onnx 构建一个 TensorRT(FP16) 引擎，同时在 onnxruntime 和 TensorRT 中运行，对比结果
polygraphy run model.onnx \
    --onnxrt --trt \
    --workspace 1000000000 \
    --save-engine=model-FP16.plan \
    --atol 1e-3 --rtol 1e-3 \
    --fp16 \
    --verbose \
    --trt-min-shapes 'tensor-0:[1,1,28,28]' \
    --trt-opt-shapes 'tensor-0:[4,1,28,28]' \
    --trt-max-shapes 'tensor-0:[16,1,28,28]' \
    --input-shapes   'tensor-0:[4,1,28,28]' \
    > result-run-FP16.txt

# 注意参数名和格式跟 trtexec 不一样，多个形状之间用空格分隔，如：
# --trt-max-shapes 'input0:[16,320,256]' 'input1:[16，320]' 'input2:[16]'
    
# 02 用上面的 .onnx 构建一个 TensorRT(FP32) 引擎，输出所有层的计算结果作对比
polygraphy run model.onnx \
    --onnxrt --trt \
    --workspace 1000000000 \
    --save-engine=model-FP32-MarkAll.plan \
    --atol 1e-3 --rtol 1e-3 \
    --verbose \
    --onnx-outputs mark all \
    --trt-outputs mark all \
    --trt-min-shapes 'tensor-0:[1,1,28,28]' \
    --trt-opt-shapes 'tensor-0:[4,1,28,28]' \
    --trt-max-shapes 'tensor-0:[16,1,28,28]' \
    --input-shapes   'tensor-0:[4,1,28,28]'
    > result-run-FP32-MarkAll.txt
```

<img src="C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708155022813.png" alt="image-20220708155022813" style="zoom:67%;" />

2、Inspect模式

> 检查onnx、trt模型的详细信息并打印；检查trt和onnx的支持性

```bash
clear
rm ./*.pb ./*.onnx ./*.plan ./result-*.txt
python getOnnxModel.py  # 从 TensorFlow 创建一个 .onnx 用来做 polygraphy 的输入文件

# 导出上面 .onnx 的详细信息
polygraphy inspect model model.onnx \
    --mode=full \
    > result-inspectOnnxModel.txt

# 用上面 .onnx 生成一个 .plan 及其相应的 tactics 用于后续分析
polygraphy run model.onnx \
    --trt \
    --workspace 1000000000 \
    --save-engine="./model.plan" \
    --save-tactics="./model.tactic" \
    --save-inputs="./model-input.txt" \
    --save-outputs="./model-output.txt" \
    --silent \
    --trt-min-shapes 'tensor-0:[1,1,28,28]' \
    --trt-opt-shapes 'tensor-0:[4,1,28,28]' \
    --trt-max-shapes 'tensor-0:[16,1,28,28]' \
    --input-shapes   'tensor-0:[4,1,28,28]'
    
# 导出上面 .plan 的详细信息（要求 TensorRT >= 8.2）
polygraphy inspect model model.plan \
    --mode=full \
    > result-inspectPlanModel.txt

# 导出上面 .tactic 的信息，就是 json 转 text
polygraphy inspect tactics model.tactic \
    > result-inspectPlanTactic.txt
    
# 导出上面 .plan 输入输出数据信息
polygraphy inspect data model-input.txt \
    > result-inspectPlanInputData.txt

polygraphy inspect data model-output.txt \
    > result-inspectPlanOutputData.txt
    
# 确认 TensorRT 是否完全原生支持该 .onnx
polygraphy inspect capability model.onnx
# 输出信息：[I] Graph is fully supported by TensorRT; Will not generate subgraphs.

# 生成一个含有 TensorRT 不原生支持的 .onnx，再次用 inspect capability 来确认
python getOnnxModel-NonZero.py

polygraphy inspect capability model-NonZero.onnx > result-NonZero.txt
# 产生目录 .results，包含网络分析信息和支持的子图(.onnx)、不支持的子图(.onnx)
```

输出结果如下：

![image-20220708210540107](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708210540107.png)

### 3、surgeon模式

> 计算图优化

```bash
clear
rm ./*.pb ./*.onnx ./*.plan ./result-*.txt
python getShapeOperateOnnxModel.py  # 从 TensorFlow 创建一个 .onnx 用来做 polygraphy 的输入文件

# 01 对上面的 .onnx 进行图优化
polygraphy surgeon sanitize model.onnx \
    --fold-constants \
    -o model-foldConstant.onnx \
    > result-surgeon.txt 
```

<img src="C:\Users\Lenovo\AppData\Roaming\Typora\typora-user-images\image-20220708211035899.png" alt="image-20220708211035899" style="zoom:67%;" />

### 4、其它模式

* convert模式
  * 基本同run模式
  * 08-Tool/Ploygraphy/convertExample
* debug模式
  * 检查转trt的错误并分离出最大可运行子图
  * 08-Tool/Ploygraphy/debugExample
* data模式
  * 调整和分析用于运行模型的输入输出、权重数据
* template模式
  * 用于生成polygraphy的python脚本，使用脚本对模型进行调整
  * 08-Tool/Ploygraphy/templateExample

## 6、nsight systems

> **GPU性能监控**
>
> * https://developer.nvidia.com/nsight-systems
> * https://docs.nvidia.com/nsight-systems/UserGuide/index.html
> * `08-Tool/NsightSystem`
>
> 运行命令：`nsys profile xxx`, 获得`qdrep`或`qdrep-nsys`文件，打开`nsys-ui`，将文件拖如查看`timeline`
>
> 注意事项：
>
> * 指计量运行阶段，不计量构建期
> * 构建期打开profiling以便获取关于Layer的信息
>   * `builder_config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED`
>   * `09-Advance/ProfilingVerbosity`
> * 可以搭配trtexec使用：`nsys profile -0 myProfile trtexec --loadEngine=model.plan --warmup=0 --duration=0 --iteration=50`
> * 搭配自定义脚本使用：`nsys profile -o myProfile python myScript.py`
> * nsys不能向前兼容

![image-20220708213653985](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708213653985.png)

![image-20220708213730773](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708213730773.png)

![image-20220708213810348](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708213810348.png)

![image-20220708213901323](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708213901323.png)

![image-20220708213929624](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708213929624.png)

![image-20220708213958434](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708213958434.png)

![image-20220708214025765](C:\Users\Lenovo\Desktop\jit\tensorrt.assert\image-20220708214025765.png)
