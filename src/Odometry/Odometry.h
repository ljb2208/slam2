#pragma once

#include "Util/SLImage.h"
#include "Pangolin/SlamViewer.h"
#include "Matcher.h"
#include "Matches.h"
#include "Util/Timer.h"
#include "Mapping/Mapping.h"
#include <cv.h>
#include <Eigen/Geometry>
#include "IO/DataSetReader.h"
#include "5Point/5point.h"

class Odometry
{
    public:
        Odometry(SlamViewer* viewer, Mapping* mapping, cv::Mat cameraMatrix, float baseLine, int imageHeight, int imageWidth);
        ~Odometry();
        bool addStereoFrames(SLImage* image, SLImage* imageRight);
        float getAverageTranslationError();
        float getAverageMotionError();
        float getPercentageMotionError();

        // camera parameters (all are mandatory / need to be supplied)
        struct calibration {  
            double f;  // focal length (in pixels)
            double cu; // principal point (u-coordinate)
            double cv; // principal point (v-coordinate)
            calibration () {
            f  = 1;
            cu = 0;
            cv = 0;
            }
        };
        
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
        
        // general parameters
        struct parameters {
            Matcher::parameters         match;            // matching parameters
            bucketing   bucket;           // bucketing parameters
            calibration calib;            // camera calibration parameters
            double  base;             // baseline (meters)
            int32_t ransac_iters;     // number of RANSAC iterations
            double  inlier_threshold; // fundamental matrix inlier threshold
            bool    reweighting;      // lower border weights (more robust to calibration errors)
            double                      height;           // camera height above ground (meters)
            double                      pitch;            // camera pitch (rad, negative=pointing down)
            int32_t                     mono_ransac_iters;     // number of RANSAC iterations
            double                      mono_inlier_threshold; // fundamental matrix inlier threshold
            double                      motion_threshold; // directly return false on small motions
            int                         min_depth_diplay;
            int                         max_depth_display;
            int                         motion_method;
            parameters () {
                base             = 1.0;
                ransac_iters     = 200;
                inlier_threshold = 2.0;
                reweighting      = true;
                height           = 1.65;
                pitch            = 0.0;
                mono_ransac_iters     = 2000;
                mono_inlier_threshold = 0.00001;
                motion_threshold = 100.0;
                min_depth_diplay = 5;
                max_depth_display = 120;
                motion_method = 0;
            }
        };

        //std::vector<Matches::p_match>  p_matched;  // feature point matches
        Timer* timer;

    private:
        SlamViewer* viewer;        
        Matcher* matcher;
        Matches* matches;        
        Mapping* mapping;

        double *X,*Y,*Z;    // 3d points
        double *p_residual; // residuals (p_residual=p_observe-p_predict)
        double *J;          // jacobian
        double *p_observe;  // observed 2d points
        double *p_predict;  // predicted 2d points
        std::vector<int32_t>           inliers;    // inlier set
        Matrix pose = Matrix::eye(4);
        Matrix pose2 = Matrix::eye(4);
        Matrix posePrior = Matrix::eye(4);

        //essential matricies
        cv::Mat essMat;
        cv::Mat essMat2;
        cv::Mat essMat3;
        int rotationDepth;

        //quaternions
        Eigen::Quaternionf qR;
        Eigen::Quaternionf qR2;
        Eigen::Quaternionf qR3;

        // parameters
        parameters param;

        enum result { UPDATED, FAILED, CONVERGED };

        bool Tr_valid;
        Matrix Tr_delta;
        uint8_t* getImageArray(SLImage* image);
        bool updateMotion();
        bool updateMotion2();
        bool updateMotion3();

        std::vector<double> estimateMotion ();
        std::vector<double> estimateMotion2 ();
        Matrix estimateMotion3 (bool* result);
        std::vector<int32_t> getInliers5Point(PMatrix P);


        bool normalizeFeaturePoints(std::vector<Matches::p_match> &p_matched,Matrix &Tp,Matrix &Tc);
        Matrix smallerThanMedian (Matrix &X,double &median);
        std::vector<int32_t> getInlier (std::vector<Matches::p_match> &p_matched,Matrix &F);
        void EtoRt(Matrix &E,Matrix &K,Matrix &X,Matrix &R,Matrix &t);
        int32_t triangulateChieral (Matrix &K,Matrix &R,Matrix &t,Matrix &X);
        void fundamentalMatrix (const std::vector<Matches::p_match> &p_matched,const std::vector<int32_t> &active,Matrix &F);
  

        Matrix transformationVectorToMatrix (std::vector<double> tr);
        std::vector<int32_t> getRandomSample(int32_t N,int32_t num);
        Odometry::result updateParameters(Matches* matches,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
        std::vector<int32_t> getInlier(Matches* matches, std::vector<double> &tr);
        void computeObservations(Matches* matches,std::vector<int32_t> &active);
        void computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);

        void calculateDepth();
        void createDepthColorArray();
           
        // returns the number of inliers: num_inliers <= num_matched
        int32_t getNumberOfInliers () { return inliers.size(); }
        Matrix getMotion () { return Tr_delta; }  

        cv::Scalar getColorFromDepth(float depth);
        cv::Mat getDepthImage();

        bool estimateRotation();
        bool convertRotations();

        float getRotationError(int index);
        float getTranslationError(int index);
        float getMotionError(int index);

        std::vector<Matrix> groundTruth;
        double groundTruthMotionError;
        double groundTruthTranslationError;
        double totalMotion;
        double computedMotion;
        double groundTruthMotion;
        int frameProcessedCount;

        std::ofstream outputFile;

        std::vector<cv::Scalar> depthDisplayVec;

        template<class T> struct idx_cmp {
            idx_cmp(const T arr) : arr(arr) {}
            bool operator()(const size_t a, const size_t b) const { return arr[a] < arr[b]; }
            const T arr;
        };
};