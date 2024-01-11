# Infrastructure Monocular 3D Object Detection

Welcome! This is the repo for you to start your journey with Infrastructure Monocular 3D Object Detection challenge.
We provide scripts for you to load labels and visualize 3D objects on image.

[Overview](#Overview) // [Get Started](#getstarted) // [Requirements](#requirements)

## Overview

![challenge5](./.assets/challenge5.png)

Infrastructure perception is a key technology to enable intelligent and safe Autonomous Driving (AD) functions, ICV (Intelligent Connected Vehicle) China is developing such monocular 3D object perception technology.

The task is challenging in: 1) various infrastructure camera installation poses, 2) camera optical axis is no longer parallel to the ground, 3) larger number of objects than on-vehicle.

In this challenge, we provide you more than 10000 images and the corresponding annotations to taste this task. You are invited to detect and classify 3D objects using only image from roadside camera: it means the input is images and their intrisic and extrinsic parameters, the output is 3D object properties, including type, location, shape and rotation.

## GetStarted

### Datasets

The challenge5 dataset is organized as follows:

```
<DATASET_ROOT>
├── training
│   ├── calib
│   ├── groundplane
│   ├── images
│   └── labels
└── test
    ├── calib
    ├── groundplane
    └── images
```

### Annotations

An example of annotation is shown in below table:

| 1   | 2   | 3   | 4       | 5       | 6       | 7       | 8     | 9     | 10    | 11      | 12      | 13     | 14    |
| --- | --- | --- | ------- | ------- | ------- | ------- | ----- | ----- | ----- | ------- | ------- | ------ | ----- |
| Car | 0.0 | 0.5 | 385.959 | 167.884 | 493.861 | 235.018 | 1.545 | 1.886 | 4.332 | -16.361 | -10.232 | 68.357 | 1.689 |

String1: describes the type of object

String2: indicates whether the object is truncated

String3: represents whether the object is occluded

String4-7: 2d boundary box of object (xmin, ymin, xmax, ymax)

String8-10: the dimensions (height, width and length) of a 3-dimensional object in meters

String11-13: the position of the three-dimensional object (x, y, z) in meters;

String14: the spatial direction of the 3-dimensional object: rotation_y, in the camera coordinate system, the global direction angle of the object (the included angle between the forward direction of the object and the x axis of the camera coordinate system), range: -pi~ pi.

### Scripts

We provide scripts to get you started:

1. `data_reader.py`, it helps you read the dataset. It also has a `pytorch` dataset, you could uncomment it if you do not use `pytorch`. You could also copy it to your existing training repo.
2. `visualize.py`, it helps you visualize 3D objects on the image
3. `interfaces.py`, it defines 3D object and Calibration classes
4. `projection.py`, it has some useful project related functions for visualization

### Visualization Usage

Step0. Download the dataset

Step1. Make a soft link to your downloaded dataset by `ln -s /your/downloaded/dataset/path data`

Step2. Simply run `python visualize.py`, which will visualize samples of dataset.

### Requirements

This package requires some simple packages installed: `numpy`, `cv2` and optional `pytorch` if you need.

## Baseline Usage

[Baseline guide](./baseline_methods/README.md)

This is a quick start to run the baseline method code:

Please make sure you have all the dependency installed, and link your data to `root/data/`

step0. `cd baseline_methods`

step1. modify the config file `./config/train.yaml`

step2. run `python ./train_val.py --config ./config/train.yaml` or `bash train.sh ./config/config.yaml`

step3. run `python ./inferenec.py --config ./config/train.yaml` or `bash inferenec.sh ./config/config.yaml`

### Requirements

We run our code on (python=3.8.0, pytorch=1.10.1 cuda=11.3.1), and you need other packages in the `./requirements.txt`.

You can run `conda install pytorch==1.10.1 torchvision cudatoolkit=11.3.1 -c pytorch` and `pip install -r requirements.txt` to install them

More detailes is shown in [`./baseline_methods/README.md`](./baseline_methods/README.md) you can setup the environment step by step.
