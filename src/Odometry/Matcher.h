/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef __MATCHER_H__
#define __MATCHER_H__

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>

#include "Matrix.h"
#include "Matches.h"

class Matcher {

public:

  // parameter settings
  struct parameters {
  
    int32_t nms_n;                  // non-max-suppression: min. distance between maxima (in pixels)
    int32_t nms_tau;                // non-max-suppression: interest point peakiness threshold
    int32_t match_binsize;          // matching bin width/height (affects efficiency only)
    int32_t match_radius;           // matching radius (du/dv in pixels)
    int32_t match_disp_tolerance;   // dv tolerance for stereo matches (in pixels)
    int32_t outlier_disp_tolerance; // outlier removal: disparity tolerance (in pixels)
    int32_t outlier_flow_tolerance; // outlier removal: flow tolerance (in pixels)
    int32_t multi_stage;            // 0=disabled,1=multistage matching (denser and faster)
    int32_t half_resolution;        // 0=disabled,1=match at half resolution, refine at full resolution
    int32_t refinement;             // refinement (0=none,1=pixel,2=subpixel)
    double  f,cu,cv,base;           // calibration (only for match prediction)
    int32_t ncc_size;
    float   ncc_tolerance;
    
    // default settings
    parameters () {
      nms_n                  = 3;
      nms_tau                = 50;
      match_binsize          = 50;
      match_radius           = 200;
      match_disp_tolerance   = 2;
      outlier_disp_tolerance = 5;
      outlier_flow_tolerance = 5;
      multi_stage            = 1;
      half_resolution        = 1; // default is 1
      refinement             = 2; // default is 1
      ncc_size               = 5;
      ncc_tolerance          = 0.9;
    }
  };

  // constructor (with default parameters)
  Matcher(parameters param, Matches* matches);

  // deconstructor
  ~Matcher();
  
  // intrinsics
  void setIntrinsics(double f,double cu,double cv,double base) {
    param.f = f;
    param.cu = cu;
    param.cv = cv;
    param.base = base;
  }

  // computes features from left/right images and pushes them back to a ringbuffer,
  // which interally stores the features of the current and previous image pair
  // use this function for stereo or quad matching
  // input: I1,I2 .......... pointers to left and right image (row-aligned), range [0..255]
  //        dims[0,1] ...... image width and height (both images must be rectified and of same size)
  //        dims[2] ........ bytes per line (often equals width)
  //        replace ........ if this flag is set, the current image is overwritten with
  //                         the input images, otherwise the current image is first copied
  //                         to the previous image (ring buffer functionality, descriptors need
  //                         to be computed only once)    
  void pushBack (uint8_t *I1,uint8_t* I2,int32_t* dims,const bool replace);
  
  // computes features from a single image and pushes it back to a ringbuffer,
  // which interally stores the features of the current and previous image pair
  // use this function for flow computation
  // parameter description see above
  void pushBack (uint8_t *I1,int32_t* dims,const bool replace) { pushBack(I1,0,dims,replace); }

  // match features currently stored in ring buffer (current and previous frame)
  // input: method ... 0 = flow, 1 = stereo, 2 = quad matching
  //        Tr_delta: uses motion from previous frame to better search for
  //                  matches, if specified
  void matchFeatures(int32_t method, Matrix *Tr_delta = 0);

  // feature bucketing: keeps only max_features per bucket, where the domain
  // is split into buckets of size (bucket_width,bucket_height) but considering
  // feature strength, type and age
  void bucketFeatures(int32_t max_features,float bucket_width,float bucket_height);

  // given a vector of inliers computes gain factor between the current and
  // the previous frame. this function is useful if you want to reconstruct 3d
  // and you want to cancel the change of (unknown) camera gain.
  float getGain (std::vector<int32_t> inliers);

private:
  
  // u/v ranges for matching stage 0-3
  struct range {
    float u_min[4];
    float u_max[4];
    float v_min[4];
    float v_max[4];
  };
  
  struct delta {
    float val[8];
    delta () {}
    delta (float v) {
      for (int32_t i=0; i<8; i++)
        val[i] = v;
    }
  };
  
  // computes the address offset for coordinates u,v of an image of given width
  inline int32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
    return v*width+u;
  }

  // Alexander Neubeck and Luc Van Gool: Efficient Non-Maximum Suppression, ICPR'06, algorithm 4
  void nonMaximumSuppression (int16_t* I_f1,int16_t* I_f2,const int32_t* dims,std::vector<Matches::maximum> &maxima,int32_t nms_n);

  float calculateNCC(const uint8_t* img1, const uint8_t* img2, const int32_t u1, const int32_t v1, const int32_t u2, const int32_t v2, const int32_t imageWidth, const int32_t imageHeight);
  float correlationValue( float SumI, float SumISq, float SumIT, float SumT, float cPixels, float fDenomExp );
  uint8_t readPixel(const uint8_t* image, int width, int height, int x, int y );

  // descriptor functions
  inline uint8_t saturate(int16_t in);
  void filterImageAll (uint8_t* I,uint8_t* I_du,uint8_t* I_dv,int16_t* I_f1,int16_t* I_f2,const int* dims);
  void filterImageSobel (uint8_t* I,uint8_t* I_du,uint8_t* I_dv,const int* dims);
  inline void computeDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
  inline void computeSmallDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
  void computeDescriptors (uint8_t* I_du,uint8_t* I_dv,const int32_t bpl,std::vector<Matches::maximum> &maxima, bool rightImage);
  
  void getHalfResolutionDimensions(const int32_t *dims,int32_t *dims_half);
  uint8_t* createHalfResolutionImage(uint8_t *I,const int32_t* dims);

  // compute sparse set of features from image
  // inputs:  I ........ image
  //          dims ..... image dimensions [width,height]
  //          n ........ non-max neighborhood
  //          tau ...... non-max threshold
  // outputs: max ...... vector with maxima [u,v,value,class,descriptor (128 bits)]
  //          I_du ..... gradient in horizontal direction
  //          I_dv ..... gradient in vertical direction
  // WARNING: max,I_du,I_dv has to be freed by yourself!
  void computeFeatures (uint8_t *I,const int32_t* dims,int32_t* &max1,int32_t &num1,int32_t* &max2,int32_t &num2,uint8_t* &I_du,uint8_t* &I_dv,uint8_t* &I_du_full,uint8_t* &I_dv_full, bool rightImage);

  // matching functions
  void computePriorStatistics (Matches* p_matched,int32_t method);
  void createIndexVector (int32_t* m,int32_t n,std::vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num);
  inline void findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,
                         std::vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                         int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_=-1,double v_=-1);
  void matching (int32_t *m1p,int32_t *m2p,int32_t *m1c,int32_t *m2c,
                 int32_t n1p,int32_t n2p,int32_t n1c,int32_t n2c,
                 Matches* p_matched,int32_t method,bool use_prior,Matrix *Tr_delta = 0);

  // outlier removal
  void removeOutliers (Matches* p_matched,int32_t method);
  void removeOutliersNCC();

  // parabolic fitting
  bool parabolicFitting(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                        const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                        const float &u1,const float &v1,
                        float       &u2,float       &v2,
                        Matrix At,Matrix AtA,
                        uint8_t* desc_buffer);
  void relocateMinimum(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                       const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                       const float &u1,const float &v1,
                       float       &u2,float       &v2,
                       uint8_t* desc_buffer);
  void refinement (Matches* p_matched,int32_t method);

  int32_t getMatchId();

  //void updatePersistentMatches();

  // mean for gain computation
  inline float mean(const uint8_t* I,const int32_t &bpl,const int32_t &u_min,const int32_t &u_max,const int32_t &v_min,const int32_t &v_max);

  // parameters
  parameters param;
  int32_t    margin;
  
  int32_t *m1p1,*m2p1,*m1c1,*m2c1;
  int32_t *m1p2,*m2p2,*m1c2,*m2c2;
  int32_t n1p1,n2p1,n1c1,n2c1;
  int32_t n1p2,n2p2,n1c2,n2c2;
  uint8_t *I1p,*I2p,*I1c,*I2c;
  uint8_t *I1p_du,*I2p_du,*I1c_du,*I2c_du;
  uint8_t *I1p_dv,*I2p_dv,*I1c_dv,*I2c_dv;
  uint8_t *I1p_du_full,*I2p_du_full,*I1c_du_full,*I2c_du_full; // only needed for
  uint8_t *I1p_dv_full,*I2p_dv_full,*I1c_dv_full,*I2c_dv_full; // half-res matching
  int32_t dims_p[3],dims_c[3];

  int32_t imageWidth, imageHeight;
  int32_t matchId;
  std::vector<Matcher::range>   ranges;

  Matches* p_matched_p;
};

#endif

