#pragma once

#include "Util/SLImage.h"
#include "Pangolin/SlamViewer.h"
#include "Features.h"
#include "Matcher.h"
#include "Util/Timer.h"
#include "Mapping/Mapping.h"
#include <cv.h>
#include <Eigen/Geometry>
#include "IO/DataSetReader.h"

class Odometry
{
    public:
        Odometry(SlamViewer* viewer, Mapping* mapping, cv::Mat cameraMatrix, float baseLine);
        ~Odometry();
        bool addStereoFrames(SLImage* image, SLImage* imageRight);

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
            parameters () {
                base             = 1.0;
                ransac_iters     = 200;
                inlier_threshold = 2.0;
                reweighting      = true;
            }
        };

        std::vector<Matches::p_match>  p_matched;  // feature point matches
        Timer* timer;

    private:
        SlamViewer* viewer;
        Features* features;
        Matcher* matcher;        
        Mapping* mapping;

        double *X,*Y,*Z;    // 3d points
        double *p_residual; // residuals (p_residual=p_observe-p_predict)
        double *J;          // jacobian
        double *p_observe;  // observed 2d points
        double *p_predict;  // predicted 2d points
        std::vector<int32_t>           inliers;    // inlier set
        Matrix pose = Matrix::eye(4);

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

        std::vector<double> estimateMotion ();
        Matrix transformationVectorToMatrix (std::vector<double> tr);
        std::vector<int32_t> getRandomSample(int32_t N,int32_t num);
        Odometry::result updateParameters(std::vector<Matches::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
        std::vector<int32_t> getInlier(std::vector<Matches::p_match> &p_matched, std::vector<double> &tr);
        void computeObservations(std::vector<Matches::p_match> &p_matched,std::vector<int32_t> &active);
        void computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);
        // returns the number of successfully matched points, after bucketing
        int32_t getNumberOfMatches () { return p_matched.size(); }
        
        // returns the number of inliers: num_inliers <= num_matched
        int32_t getNumberOfInliers () { return inliers.size(); }
        Matrix getMotion () { return Tr_delta; }      

        cv::Scalar getColorFromDepth(float depth);
        cv::Mat getDepthImage();

        bool estimateRotation();
        bool convertRotations();

        float getRotationError(int index);
        float getTranslationError(int index);

        std::vector<Matrix> groundTruth;

        std::ofstream outputFile;
};