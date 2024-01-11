/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <cstring>
#include <iostream>
#include "/opt/nvidia/deepstream/deepstream-6.3/sources/includes/nvdsinfer_custom_impl.h"
#include <cassert>
#include <cmath>
#include <malloc.h>
#include <algorithm>
#include <vector>
#include <float.h>
#include <torch/torch.h>

using namespace std;

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define CLIP(a,min,max) (MAX(MIN(a, max), min))
#define DIVIDE_AND_ROUND_UP(a, b) ((a + b - 1) / b)

struct MrcnnRawDetection {
    float y1, x1, y2, x2, class_id, score;
};

float clamp_f(float x, float min, float max)
{
  return (std::max(std::min(x, max),min));
}

float sigmoid_f(float x)
{
  return (1.f/(1.f+exp(-x)));
}

torch::Tensor occupy_value(float *input, int (&dim)[3])
{
  torch::Tensor ret = torch::zeros({dim[0],dim[1],dim[2]});
  int counter=0;
  std::cout << (input) << std::endl;
  for(int i=0;i<dim[0];++i)
  {
    for(int j=0;j<dim[1];++j)
    {
      for(int k=0;k<dim[2];++k)
      {
        ret[i][j][k] = *(input+counter);
        // if(counter <20) 
        // {
        //   std::cout << *(input+counter) << std::endl;
        // }
        counter++;
      }
    }
  }
  return ret;
}

// auto img = torch::zeros({3, 6});

// torch::Tensor nms_f(torch::Tensor x, int kernel)
// {
//   int padding_temp = ceil((kernel -1)/2);
//   torch::Tensor x_max = torch::nn::functional::max_pool2d(x, torch::nn::functional::MaxPool2dFuncOptions(3).stride(1).padding(padding_temp));
//   torch::Tensor keep = torch::eq(x_max, x);
//   // std::cout << (torch::eq(x_max, x))*x << std::endl;
//   return keep*x;
// }

torch::Tensor _nms(torch::Tensor heat, int64_t kernel = 3)
{
    int64_t pad = (kernel - 1) / 2;
    torch::Tensor hmax = at::max_pool2d(heat, {kernel, kernel}, {1, 1}, {pad, pad});

    torch::Tensor keep = (hmax == heat).toType(torch::kFloat32);

    return heat * keep;
}


// torch::Tensor gather_feat_f(torch::Tensor feat, torch::Tensor indx)
// {
//   // std::cout << feat.sizes() << std::endl;
//   int64_t dim = (feat.sizes())[2];
//   // std::cout << dim << std::endl;

//   torch::Tensor ind = torch::unsqueeze(indx,2).expand({(indx.sizes())[0],(indx.sizes())[1],dim});
//   // std::cout << ind.sizes() << std::endl;
//   // std::cout << feat.sizes() << std::endl;
//   // std::cout << ind.sizes()<< std::endl;
//   auto ret = feat.gather(1, ind);
//   std::cout << ind.sizes()<< std::endl;
  
//   return ret;
// }

torch::Tensor _gather_feat(torch::Tensor feat, torch::Tensor ind)
{
    int64_t dim = feat.size(2);
    ind = ind.unsqueeze(2).expand({ind.size(0), ind.size(1), dim});
    feat = feat.gather(1, ind);
    return feat;
}


// torch::Tensor topk_f(torch::Tensor x, int k, int(&dim)[4])
// {
//   // dim b c h w
//   auto x_v = x.view({dim[0], dim[1],-1});
//   std::tuple<torch::Tensor,torch::Tensor> topk_all = torch::topk(x_v, k);
//   auto topk_inds = std::get<1>(topk_all)%(dim[2]*dim[3]);
//   auto topk_ys = ceil(torch::div(topk_inds, dim[3]));
//   auto topk_xs = topk_inds % dim[3];
//   auto topk_temp = std::get<0>(topk_all).view({dim[0],-1});
//   std::tuple<torch::Tensor,torch::Tensor> topk_s = torch::topk(topk_temp, k);
//   auto topk_cls_ids = torch::div(std::get<1>(topk_s), k);
//   // std::cout <<   (topk_inds.view({dim[0],-1,1})).sizes() << std::endl;
//   topk_inds = gather_feat_f(topk_inds.view({dim[0],-1,1}),std::get<1>(topk_s)).view({dim[0], k});
//   // std::cout <<   (topk_inds).sizes() << std::endl;
//   topk_ys = gather_feat_f(topk_ys.view({dim[0],-1,1}),std::get<1>(topk_s)).view({dim[0], k});
//   topk_xs = gather_feat_f(topk_xs.view({dim[0],-1,1}),std::get<1>(topk_s)).view({dim[0], k});
//   // std::cout <<   (topk_ys.view({dim[0],-1,1})).sizes() << std::endl;
//   auto ret = torch::stack({std::get<0>(topk_s),topk_inds,topk_cls_ids,topk_xs,topk_ys},0);
//   // std::cout << topk_ys << std::endl;
//   return ret;
// }

void _topk(const torch::Tensor &scores,
           torch::Tensor &topk_score,
           torch::Tensor &topk_inds,
           torch::Tensor &topk_clses,
           torch::Tensor &topk_ys,
           torch::Tensor &topk_xs,
           int64_t K = 50)
{
    int64_t batch = scores.sizes()[0];
    int64_t cat = scores.sizes()[1];
    int64_t height = scores.sizes()[2];
    int64_t width = scores.sizes()[3];

    std::tuple<torch::Tensor, torch::Tensor> topk_score_inds =
        torch::topk(scores.view({batch, cat, -1}), K);

    torch::Tensor topk_scores = std::get<0>(topk_score_inds);
    topk_inds = std::get<1>(topk_score_inds);
    
    topk_inds = topk_inds % (height * width);

    topk_ys = (topk_inds / width).toType(torch::kInt32).toType(torch::kFloat32);
    topk_xs = (topk_inds % width).toType(torch::kInt32).toType(torch::kFloat32);
	
    std::tuple<torch::Tensor, torch::Tensor> topk_score_ind =
        torch::topk(topk_scores.view({batch, -1}), K);
        
    topk_score = std::get<0>(topk_score_ind);
    torch::Tensor topk_ind = std::get<1>(topk_score_ind);
    topk_clses = (topk_ind / K).toType(torch::kInt32);
    topk_inds = _gather_feat(topk_inds.view({batch, -1, 1}), topk_ind).view({batch, K});
    topk_ys = _gather_feat(topk_ys.view({batch, -1, 1}), topk_ind).view({batch, K});
    topk_xs = _gather_feat(topk_xs.view({batch, -1, 1}), topk_ind).view({batch, K});
}



// torch::Tensor transpose_and_gather_feat_f(torch::Tensor feat, torch::Tensor indx)
// {
//   auto feat_v = feat.permute({0,2,3,1}).contiguous();
//   feat_v = feat_v.view({feat_v.sizes()[0],-1,feat_v.sizes()[3]});
//   // std::cout << feat_v.sizes() << std::endl;
//   auto ret = gather_feat_f(feat_v, indx);
//   // std::cout << ret.sizes() << std::endl;
//   return ret;
// }

torch::Tensor _tranpose_and_gather_feat(torch::Tensor feat, torch::Tensor ind)
{
    feat = feat.permute({0, 2, 3, 1}).contiguous();
    feat = feat.view({feat.size(0), -1, feat.size(3)});
    feat = _gather_feat(feat, ind);
    return feat;
}

float get_heading_angle(vector<float> &heading)
{
    std::vector<float> heading_bin, heading_res;
    for(int i=0;i<24;i++)
    {
      if(i<12)
      {
        heading_bin.push_back(heading[i]);
      }
      else
      {
        heading_res.push_back(heading[i]);
      }
      
    }
    auto cls = max_element(heading_bin.begin(),heading_bin.end()) - heading_bin.begin(); 
    auto res = heading_res[cls];
    float angle_per_class = 2*M_PI/12.0f;
    auto angle_center = cls*angle_per_class;
    auto angle = angle_center+res;
    if(angle>M_PI)
    {
      angle = angle - 2*M_PI;
    }
    return angle;
}

float get_alpha2ry(float alpha, float u)
{
  auto ry = alpha + atan2(u - 517.0099155, 517.0099155); // need to be confirmed
  if (ry > M_PI)
  {
    ry -=2*M_PI;
  }
  if (ry < M_PI)
  {
    ry +=2*M_PI;
  }
  return ry;
}

/* This is a sample bounding box parsing function for the sample Resnet10
 * detector model provided with the SDK. */

/* C-linkage to prevent name-mangling */
extern "C"
bool NvDsInferParseCustomResnet (std::vector<NvDsInferLayerInfo> const &outputLayersInfo,
        NvDsInferNetworkInfo  const &networkInfo,
        NvDsInferParseDetectionParams const &detectionParams,
        std::vector<NvDsInferParseObjectInfo> &objectList);

/* This is a sample bounding box parsing function for the tensorflow SSD models
 * detector model provided with the SDK. */
//  NvDsInferObjectDetectionInfo  NvDsInfer3dObjectDetectionInfo
extern "C"
bool NvDsInferParseCustomResnet (std::vector<NvDsInferLayerInfo> const &outputLayersInfo,
        NvDsInferNetworkInfo  const &networkInfo,
        NvDsInferParseDetectionParams const &detectionParams,
        std::vector<NvDsInferParseObjectInfo> &objectList)
{
  static NvDsInferDimsCHW heatmap_LayerDims, offset_2d_LayerDims,size_2d_LayerDims,depth_LayerDims,offset_3d_LayerDims,size_3d_LayerDims,heading_LayerDims,center2kpt_offset_LayerDims,kpt_heatmap_LayerDims,kpt_heatmap_offset_LayerDims;
  static int heatmap_LayerIndex, offset_2d_LayerIndex,size_2d_LayerIndex,depth_LayerIndex,offset_3d_LayerIndex,size_3d_LayerIndex,heading_LayerIndex,center2kpt_offset_LayerIndex,kpt_heatmap_LayerIndex,kpt_heatmap_offset_LayerIndex;
  heatmap_LayerIndex = -1;
  offset_2d_LayerIndex = -1;
  size_2d_LayerIndex = -1;
  depth_LayerIndex = -1;
  offset_3d_LayerIndex = -1;
  size_3d_LayerIndex = -1;
  heading_LayerIndex = -1;
  center2kpt_offset_LayerIndex = -1;
  kpt_heatmap_LayerIndex = -1;
  kpt_heatmap_offset_LayerIndex = -1;
  
  static bool classMismatchWarn = false;
  int numClassesToParse;

  // heatmap;offset_2d;size_2d;depth;offset_3d;size_3d;heading;center2kpt_offset;kpt_heatmap;kpt_heatmap_offset

  /* Find the heatmap layer */
  if (heatmap_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "heatmap") == 0) {
        heatmap_LayerIndex = i;
        getDimsCHWFromDims(heatmap_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (heatmap_LayerIndex == -1) {
    std::cerr << "Could not find heatmap layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the offset_2d layer */
  if (offset_2d_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "offset_2d") == 0) {
        offset_2d_LayerIndex = i;
        getDimsCHWFromDims(offset_2d_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (offset_2d_LayerIndex == -1) {
    std::cerr << "Could not find offset_2d layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the size_2d layer */
  if (size_2d_LayerIndex  == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "size_2d") == 0) {
        size_2d_LayerIndex = i;
        getDimsCHWFromDims(size_2d_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (size_2d_LayerIndex == -1) {
    std::cerr << "Could not find size_2d layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the depth layer */
  if (depth_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "depth") == 0) {
        depth_LayerIndex = i;
        getDimsCHWFromDims(depth_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (depth_LayerIndex == -1) {
    std::cerr << "Could not find depth layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the offset_3d layer */
  if (offset_3d_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "offset_3d") == 0) {
        offset_3d_LayerIndex = i;
        getDimsCHWFromDims(offset_3d_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (offset_3d_LayerIndex == -1) {
    std::cerr << "Could not find offset_3d layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the size_3d layer */
  if (size_3d_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "size_3d") == 0) {
        size_3d_LayerIndex = i;
        getDimsCHWFromDims(size_3d_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (size_3d_LayerIndex == -1) {
    std::cerr << "Could not find size_3d layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the heading layer */
  if (heading_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "heading") == 0) {
        heading_LayerIndex = i;
        getDimsCHWFromDims(heading_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (heading_LayerIndex == -1) {
    std::cerr << "Could not find heading layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the center2kpt_offset layer */
  if (center2kpt_offset_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "center2kpt_offset") == 0) {
        center2kpt_offset_LayerIndex = i;
        getDimsCHWFromDims(center2kpt_offset_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (center2kpt_offset_LayerIndex == -1) {
    std::cerr << "Could not find center2kpt_offset layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the kpt_heatmap layer */
  if (kpt_heatmap_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "kpt_heatmap") == 0) {
        kpt_heatmap_LayerIndex = i;
        getDimsCHWFromDims(kpt_heatmap_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (kpt_heatmap_LayerIndex == -1) {
    std::cerr << "Could not find kpt_heatmap layer buffer while parsing" << std::endl;
    return false;
    }
  }
  /* Find the kpt_heatmap_offset layer */
  if (kpt_heatmap_offset_LayerIndex == -1) {
    for (unsigned int i = 0; i < outputLayersInfo.size(); i++) {
      if (strcmp(outputLayersInfo[i].layerName, "kpt_heatmap_offset") == 0) {
        kpt_heatmap_offset_LayerIndex = i;
        getDimsCHWFromDims(kpt_heatmap_offset_LayerDims, outputLayersInfo[i].inferDims);
        break;
      }
    }
    if (kpt_heatmap_offset_LayerIndex == -1) {
    std::cerr << "Could not find kpt_heatmap_offset layer buffer while parsing" << std::endl;
    return false;
    }
  }

  /* Warn in case of mismatch in number of classes */
  if (!classMismatchWarn) {
    if (kpt_heatmap_offset_LayerDims.c != detectionParams.numClassesConfigured) {
      std::cerr << "WARNING: Num classes mismatch. Configured:" <<
        detectionParams.numClassesConfigured << ", detected by network: " <<
        kpt_heatmap_offset_LayerDims.c << std::endl;
    }
    classMismatchWarn = true;
  }


  /* Calculate the number of classes to parse */
  // numClassesToParse = MIN (kpt_heatmap_offset_LayerDims.c, detectionParams.numClassesConfigured);

  // int gridW = covLayerDims.w;
  // int gridH = covLayerDims.h;
  // int gridSize = gridW * gridH;
  // float gcCentersX[gridW];
  // float gcCentersY[gridH];
  // float bboxNormX = 35.0;
  // float bboxNormY = 35.0;

  // float *outputCovBuf = (float *) outputLayersInfo[covLayerIndex].buffer;
  // float *outputBboxBuf = (float *) outputLayersInfo[bboxLayerIndex].buffer;
  // int strideX = DIVIDE_AND_ROUND_UP(networkInfo.width, bboxLayerDims.w);
  // int strideY = DIVIDE_AND_ROUND_UP(networkInfo.height, bboxLayerDims.h);

  float *output_heatmap_Buf = (float *) outputLayersInfo[heatmap_LayerIndex].buffer;
  float *output_offset_2d_Buf = (float *) outputLayersInfo[offset_2d_LayerIndex].buffer;
  float *output_size_2d_Buf = (float *) outputLayersInfo[size_2d_LayerIndex].buffer;
  float *output_depth_Buf = (float *) outputLayersInfo[depth_LayerIndex].buffer;
  float *output_offset_3d_Buf = (float *) outputLayersInfo[offset_3d_LayerIndex].buffer;
  float *output_size_3d_Buf = (float *) outputLayersInfo[size_3d_LayerIndex].buffer;
  float *output_heading_Buf = (float *) outputLayersInfo[heading_LayerIndex].buffer;
  float *output_center2kpt_offset_Buf = (float *) outputLayersInfo[center2kpt_offset_LayerIndex].buffer;
  float *output_kpt_heatmap_Buf = (float *) outputLayersInfo[kpt_heatmap_LayerIndex].buffer;
  float *output_kpt_heatmap_offset_Buf = (float *) outputLayersInfo[kpt_heatmap_offset_LayerIndex].buffer;
  // heatmap;offset_2d;size_2d;depth;offset_3d;size_3d;heading;center2kpt_offset;kpt_heatmap;kpt_heatmap_offset
  // std::cout << "wxywxywxywxywxywxywxywxywxywxywxywxywxywxywxywxywxy" << std::endl;
  // std::cout << output_heatmap_Buf << std::endl;
  // std::cout << *(output_heatmap_Buf) << std::endl;
  // std::cout << *(output_heatmap_Buf+1296000-1) << std::endl;
  // std::cout << heatmap_LayerDims.c << std::endl;
  int bs = 1;
  // int cc = clamp_f(3,5,10);
  // std::cout << cc << std::endl;
  // torch::Tensor kpt_heatmap_offset_inf = torch::zeros({2,270,480});
  // c10::IntArrayRef kpt_heatmap_offset_inf_size = kpt_heatmap_offset_inf.sizes();
  // std::cout << kpt_heatmap_offset_inf_size << std::endl;
  // int temp[3]= {10,270,480};
  // torch::Tensor heatmap_ts = occupy_value(output_heatmap_Buf, temp);
  auto heatmap_ts = torch::from_blob(output_heatmap_Buf,{bs,10,270,480}).clone();
  auto offset_2d_ts = torch::from_blob(output_offset_2d_Buf,{bs,2,270,480}).clone();
  auto size_2d_ts = torch::from_blob(output_size_2d_Buf,{bs,2,270,480}).clone();
  auto depth_ts_all = torch::from_blob(output_depth_Buf,{bs,2,270,480}).clone();
  auto depth_ts_depth = torch::unsqueeze(depth_ts_all[0][0],0);
  auto depth_ts_sigma = torch::unsqueeze(depth_ts_all[0][1],0);
  depth_ts_depth = torch::unsqueeze(depth_ts_depth,0);
  depth_ts_sigma = torch::unsqueeze(depth_ts_sigma,0);
  // std::cout << depth_ts_depth.sizes() << std::endl;
  auto offset_3d_ts = torch::from_blob(output_offset_3d_Buf,{bs,2,270,480}).clone();
  auto size_3d_ts = torch::from_blob(output_size_3d_Buf,{bs,3,270,480}).clone();
  auto heading_ts = torch::from_blob(output_heading_Buf,{bs,24,270,480}).clone();
  auto center2kpt_offset_ts = torch::from_blob(output_center2kpt_offset_Buf,{bs,16,270,480}).clone();
  auto kpt_heatmap_ts = torch::from_blob(output_kpt_heatmap_Buf,{bs,8,270,480}).clone();
  auto kpt_heatmap_offset_ts = torch::from_blob(output_kpt_heatmap_offset_Buf,{bs,2,270,480}).clone();
  // c10::IntArrayRef heatmap_ts_size = heatmap_ts.sizes();
  // std::cout << heatmap_ts_size << std::endl;
  // std::cout << heatmap_ts[0][0][0]<< std::endl;
  // std::cout << heatmap_ts[9][269][479]<< std::endl;
  heatmap_ts = heatmap_ts.sigmoid();
  heatmap_ts = heatmap_ts.clamp((float)1e-4,(float)1-1e-4);
  // std::cout << heatmap_ts[0][0][0]<< std::endl;
  // std::cout << heatmap_ts[9][269][479]<< std::endl;
  depth_ts_depth = 1.f/(depth_ts_depth.sigmoid()+1e-6)-1.f;
  depth_ts_sigma = -depth_ts_sigma.exp();
  int temp_[4]= {1,10,270,480};
  int temp_kernel = 4;
  heatmap_ts = _nms(heatmap_ts);

  torch::Tensor topk_scores;
  torch::Tensor topk_inds;
  torch::Tensor topk_clses;
  torch::Tensor topk_ys;
  torch::Tensor topk_xs;
  _topk(heatmap_ts, topk_scores, topk_inds, topk_clses, topk_ys, topk_xs);

  int k_top = 50;

  offset_2d_ts = _tranpose_and_gather_feat(offset_2d_ts, topk_inds);
  // std::cout << offset_2d_ts.sizes()<< std::endl;
  offset_2d_ts = offset_2d_ts.view({1,50,2});
  auto xs_2d = topk_xs.view({1,50,1})+offset_2d_ts.slice(2,0,1);
  auto ys_2d = topk_ys.view({1,50,1})+offset_2d_ts.slice(2,1,2);

  offset_3d_ts = _tranpose_and_gather_feat(offset_3d_ts, topk_inds);
  offset_3d_ts = offset_3d_ts.view({1,50,2});
  auto xs_3d = topk_xs.view({1,50,1})+offset_3d_ts.slice(2,0,1);
  auto ys_3d = topk_ys.view({1,50,1})+offset_3d_ts.slice(2,1,2);

  heading_ts = _tranpose_and_gather_feat(heading_ts, topk_inds);
  heading_ts = heading_ts.view({1,50,24});

  depth_ts_depth = _tranpose_and_gather_feat(depth_ts_depth, topk_inds);
  depth_ts_depth = depth_ts_depth.view({1,50,1});

  depth_ts_sigma = _tranpose_and_gather_feat(depth_ts_sigma, topk_inds);
  depth_ts_sigma = depth_ts_sigma.view({1,50,1});

  size_3d_ts = _tranpose_and_gather_feat(size_3d_ts, topk_inds);
  size_3d_ts = size_3d_ts.view({1,50,3});

  auto cls_ids = topk_clses.view({1,50,1});
  auto scores = topk_scores.view({1,50,1});

  xs_2d = xs_2d.view({1,50,1});
  ys_2d = ys_2d.view({1,50,1});
  xs_3d = xs_3d.view({1,50,1});
  ys_3d = ys_3d.view({1,50,1});

  size_2d_ts = _tranpose_and_gather_feat(size_2d_ts, topk_inds);
  size_2d_ts = size_2d_ts.view({1,50,2});
  // std::cout << cls_ids.sizes()<< std::endl; 
  // std::cout << scores.sizes()<< std::endl; 
  // std::cout << xs_2d.sizes()<< std::endl; 
  // std::cout << ys_2d.sizes()<< std::endl; 
  // std::cout << depth_ts_depth.sizes()<< std::endl; 
  // std::cout << heading_ts.sizes()<< std::endl; 
  // std::cout << size_3d_ts.sizes()<< std::endl; 
  // std::cout << xs_3d.sizes()<< std::endl; 
  // std::cout << ys_3d.sizes()<< std::endl; 
  // std::cout << depth_ts_sigma.sizes()<< std::endl; 


  auto ret_obj = torch::cat({cls_ids,scores,xs_2d,ys_2d,size_2d_ts,depth_ts_depth,heading_ts,size_3d_ts,xs_3d,ys_3d,depth_ts_sigma},2);
  ret_obj = torch::squeeze(ret_obj,0);
  // std::cout << ret_obj[0][6].item<int>()<< std::endl; 
  // std::cout << ret_obj.sizes()<< std::endl; 
  
  {
    std::vector<NvDsInferObjectDetectionInfo_4decode> object_info_list;
    NvDsInferObjectDetectionInfo_4decode obj_info;
    int counter = 0;
    for (int j=0;j<50;j++)
    {
      obj_info.classId = ret_obj[j][0].item<int>();
      obj_info.Scores = ret_obj[j][1].item<float>();
      obj_info.Xs2d = ret_obj[j][2].item<float>();
      obj_info.Ys2d = ret_obj[j][3].item<float>();
      obj_info.Size_2d[0] = ret_obj[j][4].item<float>();
      obj_info.Size_2d[1] = ret_obj[j][5].item<float>();
      obj_info.Depth = ret_obj[j][6].item<float>();

      for (int k=0;k<24;k++)
      {
        obj_info.Heading[k] = ret_obj[j][k+7].item<float>();
      }
      obj_info.Size_3d[0] = ret_obj[j][31].item<float>();
      obj_info.Size_3d[1] = ret_obj[j][32].item<float>();
      obj_info.Size_3d[2] = ret_obj[j][33].item<float>();
      obj_info.Xs3d = 4;
      obj_info.Ys3d = 4;
      obj_info.Xs3d = ret_obj[j][34].item<float>();
      obj_info.Ys3d = ret_obj[j][35].item<float>();
      obj_info.Sigma = ret_obj[j][36].item<float>();
      // counter= counter+1;
      // std::cout << counter<< std::endl; 
      object_info_list.push_back(obj_info);
    }

    // final outtput
    // classid    bbox    dim   xy_3d+dep(4_loc)   heading   score

    // unsigned int classId;      -> classid
    // float left;                -> bbox
    // float top;                 -> dim 
    // float width;               -> (loc) xy_3d+dep
    // float height;              -> heading
    // float detectionConfidence; -> score

    NvDsInferParseObjectInfo obj;
    float downsample_ratio = 4.0f;
    for (int i=0;i<object_info_list.size();i++)
    {
      // NvDsInferParseObjectInfo obj;
      // // obj.classId = int(object_info_list[i].classId);
      // obj.classId = 2;
      // if (object_info_list[i].Scores < 0.2f)
      // {
      //   continue;
      // }
      // // downsample_ratio = 4.0
      // auto x = object_info_list[i].Xs2d*downsample_ratio;
      // auto y = object_info_list[i].Ys2d*downsample_ratio;
      // auto w = object_info_list[i].Size_2d[0]*downsample_ratio;
      // auto h = object_info_list[i].Size_2d[1]*downsample_ratio;
      // vector<int> boundingbox = {int(x-w/2), int(y-h/2), int(x+w/2), int(y+h/2)};
      // obj.left = 1E15*round(boundingbox[0])+1E10*round(boundingbox[1])+1E5*round(boundingbox[2])+round(boundingbox[3]);
      // auto d_h = object_info_list[i].Size_3d[0];
      // auto d_w = object_info_list[i].Size_3d[1];
      // auto d_l = object_info_list[i].Size_3d[2];
      // obj.top = 1E8*round(min(d_h*1E2,3E3))+1E4*round(min(d_w*1E2,3E3))+round(min(d_l*1E2,3E3));
      // auto x_3d = object_info_list[i].Xs3d*downsample_ratio;
      // auto y_3d = object_info_list[i].Ys3d*downsample_ratio;
      // auto dep = object_info_list[i].Depth;
      // obj.width = 1E8*round(min(x_3d*1E2,3E3))+1E4*round(min(y_3d*1E2,3E3))+round(min(dep*1E2,3E3));
      // std::vector<float> obj_heading_info(std::begin(object_info_list[i].Heading), std::end(object_info_list[i].Heading));
      // auto alpha = get_heading_angle(obj_heading_info);
      // obj.height = get_alpha2ry(alpha, x_3d);
      // obj.detectionConfidence = object_info_list[i].Scores*object_info_list[i].Sigma;
      // // std::cout <<x_3d<< std::endl;
      // // std::cout <<y_3d<< std::endl;
      // objectList.push_back(obj);

      // test-begin
      // obj.classId = int(object_info_list[i].classId);
      if(int(object_info_list[i].classId) > 2)
      {
        obj.classId = 3;
      }
      else
      {
        obj.classId = int(object_info_list[i].classId);
      }

      if (object_info_list[i].Scores < 0.2f)
      {
        continue;
      }
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
      // obj.width = 99999999.f;

      float dep = object_info_list[i].Depth;   //to 0.01m
      obj.top = min(dep,999.f);    
      // obj.top = 999.f;    

      std::vector<float> obj_heading_info(std::begin(object_info_list[i].Heading), std::end(object_info_list[i].Heading));
      auto alpha = get_heading_angle(obj_heading_info);
      // obj.height = get_alpha2ry(alpha, x_3d);
      float temp_h = get_alpha2ry(alpha, x_3d);
      // obj.height = max(min((float)(temp_h),2*M_PI),0.0);
      // obj.height = 0.123;
      obj.height = temp_h;

      // obj.detectionConfidence = min(float(object_info_list[i].Scores*object_info_list[i].Sigma),1.f);
      
      obj.detectionConfidence = 1E4*round(min(d_l*1E2,1E5))+object_info_list[i].Scores*1E3;
      // std::cout <<x_3d<< std::endl;
      // std::cout <<y_3d<< std::endl;

      objectList.push_back(obj);
      // test-end

    }

    // std::cout << objectList[0].width<< std::endl;
    // std::cout << objectList[1].width<< std::endl;
    // std::cout << objectList[2].width<< std::endl;
  }

  return true;
}



/* Check that the custom function has been defined correctly */
// CHECK_3d_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInfer_3d_det);
// CHECK_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInfer_3d_det);
CHECK_CUSTOM_PARSE_FUNC_PROTOTYPE(NvDsInferParseCustomResnet);

