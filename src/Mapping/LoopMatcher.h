#pragma once

#include "Util/SLImage.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>

#include "Odometry/Matrix.h"
#include "Odometry/Triangle.h"


class LoopMatcher
{
    public:

        // bucketing parameters
        struct bucketing {  
            int32_t max_features;  // maximal number of features per bucket 
            double  bucket_width;  // width of bucket
            double  bucket_height; // height of bucket
            bucketing () {
            max_features  = 2;
            bucket_width  = 50;
            bucket_height = 50;
            }
        };
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
            int32_t ransac_iters;     // number of RANSAC iterations
            double  inlier_threshold; // fundamental matrix inlier threshold
            bool    reweighting;      // lower border weights (more robust to calibration errors)
            double                      height;           // camera height above ground (meters)
            double                      pitch;            // camera pitch (rad, negative=pointing down)
            int32_t                     mono_ransac_iters;     // number of RANSAC iterations
            double                      mono_inlier_threshold; // fundamental matrix inlier threshold
            double                      motion_threshold; // directly return false on small motions
            bucketing   bucket;           // bucketing parameters              
            
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
            half_resolution        = 1;
            refinement             = 1;
            ransac_iters     = 200;
            inlier_threshold = 2.0;
            reweighting      = true;
            height           = 1.65;
            pitch            = -0.08;
            mono_ransac_iters     = 2000;
            mono_inlier_threshold = 0.00001;
            motion_threshold = 100.0;
            }
        };

        // constructor (with default parameters)
        LoopMatcher(parameters param);
        ~LoopMatcher();

        // intrinsics
        void setIntrinsics(double f,double cu,double cv,double base) {
            param.f = f;
            param.cu = cu;
            param.cv = cv;
            param.base = base;
        }

        // structure for storing matches
        struct p_match {
            float   u1p,v1p; // u,v-coordinates in previous left  image
            int32_t i1p;     // feature index (for tracking)
            float   u2p,v2p; // u,v-coordinates in previous right image
            int32_t i2p;     // feature index (for tracking)
            float   u1c,v1c; // u,v-coordinates in current  left  image
            int32_t i1c;     // feature index (for tracking)
            float   u2c,v2c; // u,v-coordinates in current  right image
            int32_t i2c;     // feature index (for tracking)
            p_match(){}
            p_match(float u1p,float v1p,int32_t i1p,float u2p,float v2p,int32_t i2p,
                    float u1c,float v1c,int32_t i1c,float u2c,float v2c,int32_t i2c):
                    u1p(u1p),v1p(v1p),i1p(i1p),u2p(u2p),v2p(v2p),i2p(i2p),
                    u1c(u1c),v1c(v1c),i1c(i1c),u2c(u2c),v2c(v2c),i2c(i2c) {}
        };

        void pushBack (uint8_t *I1,int32_t* dims,const bool replace);
                
        // match features currently stored in ring buffer (current and previous frame)
        // input: method ... 0 = flow, 1 = stereo, 2 = quad matching
        //        Tr_delta: uses motion from previous frame to better search for
        //                  matches, if specified
        void matchFeatures();

        // feature bucketing: keeps only max_features per bucket, where the domain
        // is split into buckets of size (bucket_width,bucket_height)
        void bucketFeatures(int32_t max_features,float bucket_width,float bucket_height);

        // return vector with matched feature points and indices
        std::vector<LoopMatcher::p_match> getMatches() { return p_matched_2; };

        bool updateMotion (slam2::Matrix* tr);

        int getNumberOfMatches();
        int getNumberOfInliers();


    private:

        // structure for storing interest points
        struct maximum {
            int32_t u;   // u-coordinate
            int32_t v;   // v-coordinate
            int32_t val; // value
            int32_t c;   // class
            int32_t d1,d2,d3,d4,d5,d6,d7,d8; // descriptor
            maximum() {}
            maximum(int32_t u,int32_t v,int32_t val,int32_t c):u(u),v(v),val(val),c(c) {}
        };

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

        template<class T> struct idx_cmp {
            idx_cmp(const T arr) : arr(arr) {}
            bool operator()(const size_t a, const size_t b) const { return arr[a] < arr[b]; }
            const T arr;
        };  


        // compute sparse set of features from image
        // inputs:  I ........ image
        //          dims ..... image dimensions [width,height]
        //          n ........ non-max neighborhood
        //          tau ...... non-max threshold
        // outputs: max ...... vector with maxima [u,v,value,class,descriptor (128 bits)]
        //          I_du ..... gradient in horizontal direction
        //          I_dv ..... gradient in vertical direction
        // WARNING: max,I_du,I_dv has to be freed by yourself!
        void computeFeatures (uint8_t *I,const int32_t* dims,int32_t* &max1,int32_t &num1,int32_t* &max2,int32_t &num2,uint8_t* &I_du,uint8_t* &I_dv,uint8_t* &I_du_full,uint8_t* &I_dv_full);
        
        inline void computeDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);
        inline void computeSmallDescriptor (const uint8_t* I_du,const uint8_t* I_dv,const int32_t &bpl,const int32_t &u,const int32_t &v,uint8_t *desc_addr);      
        void computeDescriptors (uint8_t* I_du,uint8_t* I_dv,const int32_t bpl,std::vector<LoopMatcher::maximum> &maxima);
  
        // Alexander Neubeck and Luc Van Gool: Efficient Non-Maximum Suppression, ICPR'06, algorithm 4
        void nonMaximumSuppression (int16_t* I_f1,int16_t* I_f2,const int32_t* dims,std::vector<LoopMatcher::maximum> &maxima,int32_t nms_n);
                
        void getHalfResolutionDimensions(const int32_t *dims,int32_t *dims_half);
        uint8_t* createHalfResolutionImage(uint8_t *I,const int32_t* dims);

        void refinement (std::vector<LoopMatcher::p_match> &p_matched);


        // parabolic fitting
        bool parabolicFitting(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                                const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                                const float &u1,const float &v1,
                                float       &u2,float       &v2,
                                slam2::Matrix At, slam2::Matrix AtA,
                                uint8_t* desc_buffer);

        void relocateMinimum(const uint8_t* I1_du,const uint8_t* I1_dv,const int32_t* dims1,
                       const uint8_t* I2_du,const uint8_t* I2_dv,const int32_t* dims2,
                       const float &u1,const float &v1,
                       float       &u2,float       &v2,
                       uint8_t* desc_buffer);

        // outlier removal
        void removeOutliers (std::vector<LoopMatcher::p_match> &p_matched);

        void matching (int32_t *m1p,int32_t *m1c,int32_t n1p,int32_t n1c, std::vector<LoopMatcher::p_match> &p_matched, bool use_prior,slam2::Matrix *Tr_delta);

        void createIndexVector (int32_t* m,int32_t n,std::vector<int32_t> *k,const int32_t &u_bin_num,const int32_t &v_bin_num);
        inline void findMatch (int32_t* m1,const int32_t &i1,int32_t* m2,const int32_t &step_size,
                         std::vector<int32_t> *k2,const int32_t &u_bin_num,const int32_t &v_bin_num,const int32_t &stat_bin,
                         int32_t& min_ind,int32_t stage,bool flow,bool use_prior,double u_=-1,double v_=-1);

        void computePriorStatistics (std::vector<LoopMatcher::p_match> &p_matched);

        // computes the address offset for coordinates u,v of an image of given width
        inline int32_t getAddressOffsetImage (const int32_t& u,const int32_t& v,const int32_t& width) {
            return v*width+u;
        }

        // parameters
        parameters param;
        int32_t    margin;
        
        int32_t *m1p1,*m1c1;
        int32_t *m1p2,*m1c2;
        int32_t n1p1,n1c1;
        int32_t n1p2,n1c2;
        uint8_t *I1p,*I1c;
        uint8_t *I1p_du,*I1c_du;
        uint8_t *I1p_dv,*I1c_dv;
        uint8_t *I1p_du_full,*I1c_du_full; // only needed for
        uint8_t *I1p_dv_full,*I1c_dv_full; // half-res matching
        int32_t dims_p[3],dims_c[3];

                
        std::vector<LoopMatcher::p_match> p_matched_1;
        std::vector<LoopMatcher::p_match> p_matched_2;
        std::vector<LoopMatcher::range>   ranges;
        std::vector<int32_t>           inliers;    // inlier set  

        slam2::Matrix                         Tr_delta;   // transformation (previous -> current frame)  
        bool                           Tr_valid;   // motion estimate exists?

        std::vector<double>  estimateMotion (std::vector<LoopMatcher::p_match> p_matched);  
        slam2::Matrix        smallerThanMedian (slam2::Matrix &X,double &median);
        bool                 normalizeFeaturePoints (std::vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &Tp,slam2::Matrix &Tc);
        void                 fundamentalMatrix (const std::vector<LoopMatcher::p_match> &p_matched,const std::vector<int32_t> &active,slam2::Matrix &F);
        void                 EtoRt(slam2::Matrix &E,slam2::Matrix &K,std::vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &X,slam2::Matrix &R,slam2::Matrix &t);
        int32_t              triangulateChieral (std::vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &K,slam2::Matrix &R,slam2::Matrix &t,slam2::Matrix &X);
        std::vector<int32_t> getInlier (std::vector<LoopMatcher::p_match> &p_matched,slam2::Matrix &F);

        // get random and unique sample of num numbers from 1:N
        std::vector<int32_t> getRandomSample (int32_t N,int32_t num);
        slam2::Matrix transformationVectorToMatrix (std::vector<double> tr);        
};