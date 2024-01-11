# Introduction

This repository is an unofficial implementation of the paper MonoCon for personal study.
We will continuously update it for better performance.

This repo benefits from [MonoDLE](https://github.com/xinzhuma/monodle) and [MonoCon](https://github.com/Xianpeng919/MonoCon).

# Usage

## Installation

Code is tested on (python=3.8.0, pytorch=1.10.1 cuda=11.3.1)

```bash
conda create -n challenge5_venv python=3.8
```

Then, activate the environment:

```bash
conda activate challenge5_venv
```

Install  Install PyTorch:

```bash
conda install pytorch==1.10.1 torchvision cudatoolkit=11.3.1 -c pytorch
```

and other requirements:

```bash
pip install -r requirements.txt
```protobuf version 3.20.3
```

## Data Preparation

```
#ROOT
  |data/
      |testing/
        |calib/
        |groundplane/
        |images/
      |training/
        |calib/
        |groundplane/
        |images/
        |labels/
      |validation/
        |calib/
        |groundplane/
        |images/
        |labels/
  |baseline_methods/
    |...
  |...

```

## Training & Evaluation

### Train

```sh
python tools/train_val.py --config config/config.yaml
```

### inference

To infrnce the model, run python script

```sh
python inference.py --config config/config.yaml
```

or use Jupyter notebook
`inference.ipynb`

The result txt will be saved to `../result`

# Troubleshooting

The baseline model use a pre-train weight for some module.

It will show this message

```
Downloading: "http://dl.yf.io/dla/models/imagenet/dla34-ba72cf86.pth" to /your_workspace/.cache/torch/hub/checkpoints/dla34-ba72cf86.pth
```

If you cannot download the pre-train weight.

Please go to  line 38 of `./baseline_mtehods/lib/modules/monocon.py` and set (pretrained=False)

Or download it manually from `http://dl.yf.io/dla/models/imagenet` and put `.pth` file to the target path.
