## Metric

The evaluation follows nuScenes 3d object detection tasks.The score is based on center distance based mAP over different distance threshold.

## Environment Requirements

`python3.9+`  `numpy`  `matplotlib` modules are needed for running.

## Tutorial

1. Check submissions file list

```
python3 evaluation.py -p [PATH_TO_YOUR_PREDICTION_FOLDER] -c
```

2. Generate report for your own validation set

```
python3 evaluation.py -p [PATH_TO_YOUR_PREDICTION_FOLDER] -g [PATH_TO_YOUR_GROUND_TRUTH_FOLDER]

```

3. Generate graph pdf report

```
 python3 evaluation.py -p [PATH_TO_YOUR_PREDICTION_FOLDER] -g [PATH_TO_YOUR_GROUND_TRUTH_FOLDER] -d [PATH_TO_YOUR_PDF_FOLDER]
```

## Submission

For each image in the test dataset named "000XXX.jpg", you must provide predictions in the following format.

**IMPORTANT:
Only string 1, 11-13, 15 are required for score calculation.
Other strings are retained for keeping the format and generate report for users to develop.**

### Prediction format

An example of prediction is shown in below table:

| 1   | 2   | 3   | 4       | 5       | 6       | 7       | 8     | 9     | 10    | 11      | 12      | 13     | 14    | 15   |
| --- | --- | --- | ------- | ------- | ------- | ------- | ----- | ----- | ----- | ------- | ------- | ------ | ----- | ---- |
| Car | 0.0 | 0.5 | 385.959 | 167.884 | 493.861 | 235.018 | 1.545 | 1.886 | 4.332 | -16.361 | -10.232 | 68.357 | 1.689 | 0.99 |

String1: **(Mandatory)** describes the type of object

String2: indicates whether the object is truncated

String3: represents whether the object is occluded

String4-7: 2d boundary box of object (xmin, ymin, xmax, ymax)

String8-10: the dimensions (height, width and length) of a 3-dimensional object in meters

String11-13: **(Mandatory)** the position of the three-dimensional object (x, y, z) in meters;

String14:  the spatial direction of the 3-dimensional object: rotation_y, in the camera coordinate system, the global direction angle of the object (the included angle between the forward direction of the object and the x axis of the camera coordinate system), range: -pi~ pi.

String15: **(Mandatory)** the confidence of the object

**Note**: String 2-10, 14 can be place holder like "0" for submission. eg:

```
Car 0 0 0 0 0 0 0 0 0 -16.361 -10.232 68.357 0 0.99
```

Please make sure that you checked your submission files by using the command:

```
python3 evaluation.py -p [PATH_TO_YOUR_PREDICTION_FOLDER] -c
```

An example `sample_submission.zip` is provided in the data set.

## License Acknowledgement

The codes is based on nuScenes-dev-kit, thus following the apache-2.0 license for usage.
