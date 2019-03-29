#include "Odometry.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <chrono>

using namespace std::chrono;

Odometry::Odometry(SlamViewer* viewer, Mapping* mapping, cv::Mat cameraMatrix, float baseLine, int imageHeight, int imageWidth)
{
    this->viewer = viewer;
    this->mapping = mapping;

    matches = new Matches(imageWidth, imageHeight);

    Tr_valid = false;    

    // f_x = 0,0
    // f_y = 1,1
    // c_x = 0,2
    // c_y = 1,2
    param.base = baseLine;
    param.calib.f = cameraMatrix.at<float>(0, 0);
    param.calib.fy = cameraMatrix.at<float>(1, 1);
    param.calib.cu = cameraMatrix.at<float>(0, 2);
    param.calib.cv = cameraMatrix.at<float>(1, 2);

    viewer->setCalibration(param.calib.f, param.calib.f, param.calib.cu, param.calib.cv);

    matcher = new Matcher(Matcher::parameters(), matches);
    matcher->setIntrinsics(param.calib.f, param.calib.fy, param.calib.cu, param.calib.cv, param.base);

    timer = new Timer();

    ImageFolderReader* reader = new ImageFolderReader();
    groundTruth = reader->getGroundTruth();
    delete reader;

    outputFile.open("outputs.csv", std::ios::trunc);
    outputFile << "Index,NumMatches,NumInliers,";
    outputFile << "pose00,pose01,pose02,pose03,pose10,pose11,pose12,pose13,pose20,pose21,pose22,pose23,pose30,pose31,pose32,pose33,";
    outputFile << "motion00,motion01,motion02,motion03,motion10,motion11,motion12,motion13,motion20,motion21,motion22,motion23,motion30,motion31,motion32,motion33";
    outputFile << ",errorR,errorT,errorM,motion,gtMotion";
    outputFile << std::endl;

    groundTruthTranslationError = 0;
    groundTruthMotionError = 0;
    totalMotion = 0;
    frameProcessedCount = 0;

    createDepthColorArray();
    setInitialPose();

    std::cout << "Initial pose: \n" << pose << "\n";
}

Odometry::~Odometry()
{
    outputFile.close();
    timer->outputAll();
    delete timer;
    delete matcher;
}

bool Odometry::addStereoFrames(SLImage* image, SLImage* imageRight)
{    
    int32_t dims[] = {image->w, image->h, image->w};

    uint8_t* left_image = image->getImageArray();
    uint8_t* right_image = imageRight->getImageArray();

    timer->startTimer("pushBack");
    matcher->pushBack(left_image, right_image, dims, false);
    timer->stopTimer();

    if (!Tr_valid)
    {
        timer->startTimer("matchFeatures");
        matcher->matchFeatures(2);
        timer->stopTimer();

        timer->startTimer("bucketFeatures");        
        //matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
        matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
        timer->stopTimer();

        // timer->startTimer("getMatches");
        // p_matched = matcher->getMatches();
        // timer->stopTimer();
    }

    // match features and update motion
    if (Tr_valid)
    {
        timer->startTimer("matchFeatures2");
        matcher->matchFeatures(2,&Tr_delta);        
        timer->stopTimer();                
    }
    else          
    {
        timer->startTimer("matchFeatures");
        matcher->matchFeatures(2);
        timer->stopTimer();
    }

    printf("Num matches pre bucket: %i\n", matches->getInlierCount());
    

    timer->startTimer("bucketFeatures2");
    //matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
    matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
    timer->stopTimer();

    // timer->startTimer("updateMotion");
    // bool result = updateMotion();
    // timer->stopTimer();

    bool result;
    timer->startTimer("updateMotion");
    if (param.motion_method == 0)
        result = updateMotion();
    else if (param.motion_method == 1)
        result = updateMotion2();
    else if (param.motion_method == 2)
        result = updateMotion3();
    timer->stopTimer();

    timer->startTimer("calculateDepth");
    calculateDepth();
    timer->stopTimer();


    int maxAge = -1;

    for (int i=0; i < matches->selectedMatches.size(); i++)
    {
        cv::Point2f matchPoint(matches->selectedMatches[i]->u1c, matches->selectedMatches[i]->v1c);
        cv::Point2f matchPointRight(matches->selectedMatches[i]->u2c, matches->selectedMatches[i]->v2c);
        cv::circle(image->imageColor, matchPoint, 4, getColorFromDepth(matches->selectedMatches[i]->depth), -1, 8, 0);
        cv::circle(imageRight->imageColor, matchPointRight, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

        if (matches->selectedMatches[i]->age > maxAge)
            maxAge = matches->selectedMatches[i]->age;
    }

    printf("Max age: %i\n", maxAge);

    std::string imageIndex = std::to_string(image->index);
    cv::putText(image->imageColor, imageIndex.c_str(), cv::Point(40, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 1, 8, false);

    free(left_image);
    free(right_image);


    viewer->pushLiveImageFrame(image->imageColor, imageRight->imageColor, image->index);
    
    // possible duplicate
    //bool result = updateMotion();

    if (result)
    {
        // on success, update current pose
        slam2::Matrix motion = getMotion();
        pose = pose * slam2::Matrix::inv(motion);

        //Matrix motion = getMotion2();
        //pose = pose * Matrix::inv(motion);

        // output some statistics
        double num_matches = matches->getSelectedCount();
        double num_inliers = getNumberOfInliers();
        std::cout << "Index: " << image->index;
        std::cout << ", Matches: " << num_matches;
        std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << std::endl;
        std::cout << pose << std::endl << std::endl;
        std::cout << "Motion:" << std::endl;
        std::cout << motion <<  std::endl << std::endl;

        outputFile << image->index << ",";
        outputFile << num_matches << ",";
        outputFile << num_inliers << ",";
        outputFile << pose.val[0][0] << ",";
        outputFile << pose.val[1][0] << ",";
        outputFile << pose.val[2][0] << ",";
        outputFile << pose.val[3][0] << ",";
        outputFile << pose.val[0][1] << ",";
        outputFile << pose.val[1][1] << ",";
        outputFile << pose.val[2][1] << ",";
        outputFile << pose.val[3][1] << ",";
        outputFile << pose.val[0][2] << ",";
        outputFile << pose.val[1][2] << ",";
        outputFile << pose.val[2][2] << ",";
        outputFile << pose.val[3][2] << ",";
        outputFile << pose.val[0][3] << ",";
        outputFile << pose.val[1][3] << ",";
        outputFile << pose.val[2][3] << ",";
        outputFile << pose.val[3][3] << ",";
        outputFile << motion.val[0][0] << ",";
        outputFile << motion.val[1][0] << ",";
        outputFile << motion.val[2][0] << ",";
        outputFile << motion.val[3][0] << ",";
        outputFile << motion.val[0][1] << ",";
        outputFile << motion.val[1][1] << ",";
        outputFile << motion.val[2][1] << ",";
        outputFile << motion.val[3][1] << ",";
        outputFile << motion.val[0][2] << ",";
        outputFile << motion.val[1][2] << ",";
        outputFile << motion.val[2][2] << ",";
        outputFile << motion.val[3][2] << ",";
        outputFile << motion.val[0][3] << ",";
        outputFile << motion.val[1][3] << ",";
        outputFile << motion.val[2][3] << ",";
        outputFile << motion.val[3][3] << ",";
        outputFile << getRotationError(image->index) << ",";
        outputFile << getTranslationError(image->index) <<  ",";
        float motionError = getMotionError(image->index);
        outputFile << motionError << ",";
        outputFile << computedMotion << "," << groundTruthMotion << std::endl;
        
        
        groundTruthMotionError += motionError;
        groundTruthTranslationError += getTranslationError(image->index);
        frameProcessedCount++;

        mapping->addFrame(pose, image, imageRight, matches);
    }
    else
    {
        std::cout << " ... failed!" << std::endl;
    }

    return result;
}

void Odometry::setInitialPose()
{
    return;
    
    // adjust initial pose for pitch
    double cs = cos(param.pitch);
    double sn = sin(param.pitch);

    pose.val[1][1] = cs;
    pose.val[1][2] = -sn;
    pose.val[2][1] = sn;
    pose.val[2][2] = cs;

    //pose.val[1][3] = param.height;
}

void Odometry::createDepthColorArray()
{
    int dist = param.max_depth_display - param.min_depth_diplay;
    int mid_dist = dist / 2;
    float scale = 510.0 / (float)dist;
    int r,g,b;

    for (int i=0; i < dist; i++)
    {
        if (i <= mid_dist)
        {
            r = 255 - (i * scale);
            g = 255 - r;
            b = 0;        
        }
        else
        {
            r = 0;
            g = 255 - ((i - mid_dist) * scale);
            b = 255 - g;
        }

        depthDisplayVec.push_back(cv::Scalar(r, g, b));
    }
}

cv::Scalar Odometry::getColorFromDepth(float depth)
{
    int iDepth = (int) depth;

    if (iDepth < param.min_depth_diplay)
        iDepth = param.min_depth_diplay;
    else if (iDepth > param.max_depth_display)
        iDepth = param.max_depth_display;

    return depthDisplayVec[iDepth - param.min_depth_diplay];
}

bool Odometry::updateMotion()
{
    // estimate motion
    std::vector<double> tr_delta = estimateMotion();
    
    // on failure
    if (tr_delta.size()!=6)
        return false;
    
    // set transformation matrix (previous to current frame)
    Tr_delta = transformationVectorToMatrix(tr_delta);
    Tr_valid = true;
    
    // success
    return true;
}

bool Odometry::updateMotion2()
{
    // estimate motion
    std::vector<double> tr_delta = estimateMotion2();
    
    // on failure
    if (tr_delta.size()!=6)
        return false;
    
    // set transformation matrix (previous to current frame)
    Tr_delta = transformationVectorToMatrix(tr_delta);
    Tr_valid = true;
    
    // success
    return true;
}

bool Odometry::updateMotion3()
{
    bool result;

    // estimate motion
    Tr_delta = estimateMotion3(&result);
    
    return result;
}

std::vector<double> Odometry::estimateMotion ()
{
    // return value
    bool success = true;

    // compute minimum distance for RANSAC samples
    double width=0,height=0;
    for (int i=0; i < matches->selectedMatches.size(); i++)
    {     
        if (matches->selectedMatches[i]->u1c>width)  width  = matches->selectedMatches[i]->u1c;
        if (matches->selectedMatches[i]->v1c>height) height = matches->selectedMatches[i]->v1c;
    }

    double min_dist = std::min(width,height)/3.0;
    
    // get number of matches
    //int32_t N  = p_matched.size();
    int32_t N  = matches->getSelectedCount();
    if (N<6)
    {
        return std::vector<double>();
    }
    // allocate dynamic memory
    X          = new double[N];
    Y          = new double[N];
    Z          = new double[N];
    J          = new double[4*N*6];
    p_predict  = new double[4*N];
    p_observe  = new double[4*N];
    p_residual = new double[4*N];

    for (int i=0; i < matches->selectedMatches.size(); i++)
    {
        double d = std::max(matches->selectedMatches[i]->u1p - matches->selectedMatches[i]->u2p,0.0001f);
        X[i] = (matches->selectedMatches[i]->u1p-param.calib.cu)*param.base/d;
        Y[i] = (matches->selectedMatches[i]->v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
        //matches->selectedMatches[i]->depth = Z[i];
    }

    // // project matches of previous image into 3d
    // for (int32_t i=0; i<N; i++) {
    //     double d = std::max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
    //     X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
    //     Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/rd;
    //     Z[i] = param.calib.f*param.base/d;
    //     p_matched[i].depth = param.calib.f*param.base/d;
    // }

    // loop variables
    std::vector<double> tr_delta;
    std::vector<double> tr_delta_curr;
    tr_delta_curr.resize(6);
    
    // clear parameter vector
    inliers.clear();

    // initial RANSAC estimate
    for (int32_t k=0;k<param.ransac_iters;k++) {

        // draw random sample set
        std::vector<int32_t> active = getRandomSample(N,3);

        // clear parameter vector
        for (int32_t i=0; i<6; i++)
        tr_delta_curr[i] = 0;

        // minimize reprojection errors
        result res = UPDATED;
        int32_t iter=0;
        while (res==UPDATED) {
        res = updateParameters(matches,active,tr_delta_curr,1,1e-6);
        if (iter++ > 20 || res==CONVERGED)
            break;
        }

        // overwrite best parameters if we have more inliers
        if (res !=FAILED) {
        std::vector<int32_t> inliers_curr = getInlier(matches,tr_delta_curr);
        if (inliers_curr.size()>inliers.size()) {
            inliers = inliers_curr;
            tr_delta = tr_delta_curr;
        }
        }
    }
    
    // final optimization (refinement)
    if (inliers.size()>=6) {
        int32_t iter=0;
        result res = UPDATED;
        while (res==UPDATED) {     
        res = updateParameters(matches,inliers,tr_delta,1,1e-8);
        if (iter++ > 100 || res==CONVERGED)
            break;
        }

        // not converged
        if (res!=CONVERGED)
        {
            success = false;
        }

    // not enough inliers
    } else {
        success = false;
    }

    // release dynamic memory
    delete[] X;
    delete[] Y;
    delete[] Z;
    delete[] J;
    delete[] p_predict;
    delete[] p_observe;
    delete[] p_residual;
    
    // parameter estimate succeeded?
    if (success)
    {
        return tr_delta;
    }
    else         
        return std::vector<double>();
}

std::vector<double> Odometry::estimateMotion2()
{
    // get number of matches
  int32_t N = matches->selectedMatches.size();
  if (N<10)
    return std::vector<double>();
   
  // create calibration matrix
  double K_data[9] = {param.calib.f,0,param.calib.cu,0,param.calib.f,param.calib.cv,0,0,1};
  slam2::Matrix K(3,3,K_data);
    
  // normalize feature points and return on errors
  slam2::Matrix Tp,Tc;
  std::vector<Matches::p_match> p_matched_normalized = matches->copySelectedMatches();
  if (!normalizeFeaturePoints(p_matched_normalized,Tp,Tc))
    return std::vector<double>();

  // initial RANSAC estimate of F
  slam2::Matrix E,F;
  inliers.clear();
  for (int32_t k=0;k<param.mono_ransac_iters;k++) {

    // draw random sample set
    std::vector<int32_t> active = getRandomSample(N,8);

    // estimate fundamental matrix and get inliers
    fundamentalMatrix(p_matched_normalized,active,F);
    std::vector<int32_t> inliers_curr = getInlier(p_matched_normalized,F);

    // update model if we are better
    if (inliers_curr.size()>inliers.size())
      inliers = inliers_curr;
  }
  
  // are there enough inliers?
  if (inliers.size()<10)
    return std::vector<double>();
  
  // refine F using all inliers
  fundamentalMatrix(p_matched_normalized,inliers,F); 
  
  // denormalise and extract essential matrix
  F = ~Tc*F*Tp;
  E = ~K*F*K;
  
  // re-enforce rank 2 constraint on essential matrix
  slam2::Matrix U,W,V;
  E.svd(U,W,V);
  W.val[2][0] = 0;
  E = U*slam2::Matrix::diag(W)*~V;
  
  // compute 3d points X and R|t up to scale
  slam2::Matrix X,R,t;
  EtoRt(E,K,X,R,t);
  
  // normalize 3d points and remove points behind image plane
  X = X/X.getMat(3,0,3,-1);
  std::vector<int32_t> pos_idx;
  for (int32_t i=0; i<X.n; i++)
    if (X.val[2][i]>0)
      pos_idx.push_back(i);
  slam2::Matrix X_plane = X.extractCols(pos_idx);
  
  // we need at least 10 points to proceed
  if (X_plane.n<10)
    return std::vector<double>();
  
  // get elements closer than median
  double median;
  smallerThanMedian(X_plane,median);
  
  // return error on large median (litte motion)
  if (median>param.motion_threshold)
    return std::vector<double>();
  
  // project features to 2d
  slam2::Matrix x_plane(2,X_plane.n);
  x_plane.setMat(X_plane.getMat(1,0,2,-1),0,0);
  
  slam2::Matrix n(2,1);
  n.val[0][0]       = cos(-param.pitch);
  n.val[1][0]       = sin(-param.pitch);
  slam2::Matrix   d        = ~n*x_plane;
  double   sigma    = median/50.0;
  double   weight   = 1.0/(2.0*sigma*sigma);
  double   best_sum = 0;
  int32_t  best_idx = 0;

  // find best plane
  for (int32_t i=0; i<x_plane.n; i++) {
    if (d.val[0][i]>median/param.motion_threshold) {
      double sum = 0;
      for (int32_t j=0; j<x_plane.n; j++) {
        double dist = d.val[0][j]-d.val[0][i];
        sum += exp(-dist*dist*weight);
      }
      if (sum>best_sum) {
        best_sum = sum;
        best_idx = i;
      }
    }
  }
  t = t*param.height/d.val[0][best_idx];
  
  // compute rotation angles
  double ry = asin(R.val[0][2]);
  double rx = asin(-R.val[1][2]/cos(ry));
  double rz = asin(-R.val[0][1]/cos(ry));
  
  // return parameter vector
  std::vector<double> tr_delta;
  tr_delta.resize(6);
  tr_delta[0] = rx;
  tr_delta[1] = ry;
  tr_delta[2] = rz;
  tr_delta[3] = t.val[0][0];
  tr_delta[4] = t.val[1][0];
  tr_delta[5] = t.val[2][0];
  return tr_delta;
}

slam2::Matrix Odometry::smallerThanMedian (slam2::Matrix &X,double &median) {
  
  // set distance and index vector
  std::vector<double> dist;
  std::vector<int32_t> idx;
  for (int32_t i=0; i<X.n; i++) {
    dist.push_back(fabs(X.val[0][i])+fabs(X.val[1][i])+fabs(X.val[2][i]));
    idx.push_back(i);
  }
  
  // sort elements
  std::sort(idx.begin(),idx.end(),idx_cmp<std::vector<double>&>(dist));
  
  // get median
  int32_t num_elem_half = idx.size()/2;
  median = dist[idx[num_elem_half]];
  
  // create matrix containing elements closer than median
  slam2::Matrix X_small(4,num_elem_half+1);
  for (int32_t j=0; j<=num_elem_half; j++)
    for (int32_t i=0; i<4; i++)
      X_small.val[i][j] = X.val[i][idx[j]];
	return X_small;
}

bool Odometry::normalizeFeaturePoints(std::vector<Matches::p_match> &p_matched,slam2::Matrix &Tp,slam2::Matrix &Tc) {
  
  // shift origins to centroids
  double cpu=0,cpv=0,ccu=0,ccv=0;
  for (std::vector<Matches::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    cpu += it->u1p;
    cpv += it->v1p;
    ccu += it->u1c;
    ccv += it->v1c;
  }
  cpu /= (double)p_matched.size();
  cpv /= (double)p_matched.size();
  ccu /= (double)p_matched.size();
  ccv /= (double)p_matched.size();
  for (std::vector<Matches::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p -= cpu;
    it->v1p -= cpv;
    it->u1c -= ccu;
    it->v1c -= ccv;
  }
  
  // scale features such that mean distance from origin is sqrt(2)
  double sp=0,sc=0;
  for (std::vector<Matches::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    sp += sqrt(it->u1p*it->u1p+it->v1p*it->v1p);
    sc += sqrt(it->u1c*it->u1c+it->v1c*it->v1c);
  }
  if (fabs(sp)<1e-10 || fabs(sc)<1e-10)
    return false;
  sp = sqrt(2.0)*(double)p_matched.size()/sp;
  sc = sqrt(2.0)*(double)p_matched.size()/sc;
  for (std::vector<Matches::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p *= sp;
    it->v1p *= sp;
    it->u1c *= sc;
    it->v1c *= sc;
  }
  
  // compute corresponding transformation matrices
  double Tp_data[9] = {sp,0,-sp*cpu,0,sp,-sp*cpv,0,0,1};
  double Tc_data[9] = {sc,0,-sc*ccu,0,sc,-sc*ccv,0,0,1};
  Tp = slam2::Matrix(3,3,Tp_data);
  Tc = slam2::Matrix(3,3,Tc_data);
  
  // return true on success
  return true;
}

void Odometry::fundamentalMatrix (const std::vector<Matches::p_match> &p_matched,const std::vector<int32_t> &active,slam2::Matrix &F) {
  
  // number of active p_matched
  int32_t N = active.size();
  
  // create constraint matrix A
  slam2::Matrix A(N,9);
  for (int32_t i=0; i<N; i++) {
    Matches::p_match m = p_matched[active[i]];
    A.val[i][0] = m.u1c*m.u1p;
    A.val[i][1] = m.u1c*m.v1p;
    A.val[i][2] = m.u1c;
    A.val[i][3] = m.v1c*m.u1p;
    A.val[i][4] = m.v1c*m.v1p;
    A.val[i][5] = m.v1c;
    A.val[i][6] = m.u1p;
    A.val[i][7] = m.v1p;
    A.val[i][8] = 1;
  }
   
  // compute singular value decomposition of A
  slam2::Matrix U,W,V;
  A.svd(U,W,V);
   
  // extract fundamental matrix from the column of V corresponding to the smallest singular value
  F = slam2::Matrix::reshape(V.getMat(0,8,8,8),3,3);
  
  // enforce rank 2
  F.svd(U,W,V);
  W.val[2][0] = 0;
  F = U*slam2::Matrix::diag(W)*~V;
}

std::vector<int32_t> Odometry::getInlier (std::vector<Matches::p_match> &p_matched,slam2::Matrix &F) {

  // extract fundamental matrix
  double f00 = F.val[0][0]; double f01 = F.val[0][1]; double f02 = F.val[0][2];
  double f10 = F.val[1][0]; double f11 = F.val[1][1]; double f12 = F.val[1][2];
  double f20 = F.val[2][0]; double f21 = F.val[2][1]; double f22 = F.val[2][2];
  
  // loop variables
  double u1,v1,u2,v2;
  double x2tFx1;
  double Fx1u,Fx1v,Fx1w;
  double Ftx2u,Ftx2v;
  
  // vector with inliers
  std::vector<int32_t> inliers;
  
  // for all matches do
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++) {

    // extract matches
    u1 = p_matched[i].u1p;
    v1 = p_matched[i].v1p;
    u2 = p_matched[i].u1c;
    v2 = p_matched[i].v1c;
    
    // F*x1
    Fx1u = f00*u1+f01*v1+f02;
    Fx1v = f10*u1+f11*v1+f12;
    Fx1w = f20*u1+f21*v1+f22;
    
    // F'*x2
    Ftx2u = f00*u2+f10*v2+f20;
    Ftx2v = f01*u2+f11*v2+f21;
    
    // x2'*F*x1
    x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;
    
    // sampson distance
    double d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);
    
    // check threshold
    if (fabs(d)<param.mono_inlier_threshold)
      inliers.push_back(i);
  }

  // return set of all inliers
  return inliers;
}

void Odometry::EtoRt(slam2::Matrix &E,slam2::Matrix &K,slam2::Matrix &X,slam2::Matrix &R,slam2::Matrix &t) {

  // hartley matrices
  double W_data[9] = {0,-1,0,+1,0,0,0,0,1};
  double Z_data[9] = {0,+1,0,-1,0,0,0,0,0};
  slam2::Matrix W(3,3,W_data);
  slam2::Matrix Z(3,3,Z_data); 
  
  // extract T,R1,R2 (8 solutions)
  slam2::Matrix U,S,V;
  E.svd(U,S,V);
  slam2::Matrix T  = U*Z*~U;
  slam2::Matrix Ra = U*W*(~V);
  slam2::Matrix Rb = U*(~W)*(~V);
  
  // convert T to t
  t = slam2::Matrix(3,1);
  t.val[0][0] = T.val[2][1];
  t.val[1][0] = T.val[0][2];
  t.val[2][0] = T.val[1][0];
  
  // assure determinant to be positive
  if (Ra.det()<0) Ra = -Ra;
  if (Rb.det()<0) Rb = -Rb;
  
  // create vector containing all 4 solutions
  std::vector<slam2::Matrix> R_vec;
  std::vector<slam2::Matrix> t_vec;
  R_vec.push_back(Ra); t_vec.push_back( t);
  R_vec.push_back(Ra); t_vec.push_back(-t);
  R_vec.push_back(Rb); t_vec.push_back( t);
  R_vec.push_back(Rb); t_vec.push_back(-t);
  
  // try all 4 solutions
  slam2::Matrix X_curr;
  int32_t max_inliers = 0;
  for (int32_t i=0; i<4; i++) {
    int32_t num_inliers = triangulateChieral(K,R_vec[i],t_vec[i],X_curr);
    if (num_inliers>max_inliers) {
      max_inliers = num_inliers;
      X = X_curr;
      R = R_vec[i];
      t = t_vec[i];
    }
  }
}


int32_t Odometry::triangulateChieral (slam2::Matrix &K,slam2::Matrix &R,slam2::Matrix &t,slam2::Matrix &X) {
  
  // init 3d point matrix
  X = slam2::Matrix(4,matches->selectedMatches.size());
  
  // projection matrices
  slam2::Matrix P1(3,4);
  slam2::Matrix P2(3,4);
  P1.setMat(K,0,0);
  P2.setMat(R,0,0);
  P2.setMat(t,0,3);
  P2 = K*P2;
  
  // triangulation via orthogonal regression
  slam2::Matrix J(4,4);
  slam2::Matrix U,S,V;
  for (int32_t i=0; i<(int)matches->selectedMatches.size(); i++) {
    for (int32_t j=0; j<4; j++) {
      J.val[0][j] = P1.val[2][j]*matches->selectedMatches[i]->u1p - P1.val[0][j];
      J.val[1][j] = P1.val[2][j]*matches->selectedMatches[i]->v1p - P1.val[1][j];
      J.val[2][j] = P2.val[2][j]*matches->selectedMatches[i]->u1c - P2.val[0][j];
      J.val[3][j] = P2.val[2][j]*matches->selectedMatches[i]->v1c - P2.val[1][j];
    }
    J.svd(U,S,V);
    X.setMat(V.getMat(0,3,3,3),0,i);
  }
  
  // compute inliers
  slam2::Matrix  AX1 = P1*X;
  slam2::Matrix  BX1 = P2*X;
  int32_t num = 0;
  for (int32_t i=0; i<X.n; i++)
    if (AX1.val[2][i]*X.val[3][i]>0 && BX1.val[2][i]*X.val[3][i]>0)
      num++;
  
  // return number of inliers
  return num;
}



std::vector<int32_t> Odometry::getRandomSample(int32_t N,int32_t num)
{
    // init sample and totalset
    std::vector<int32_t> sample;
    std::vector<int32_t> totalset;
    
    // create vector containing all indices
    for (int32_t i=0; i<N; i++)
        totalset.push_back(i);

    // add num indices to current sample
    sample.clear();
    for (int32_t i=0; i<num; i++) {
        int32_t j = rand()%totalset.size();
        sample.push_back(totalset[j]);
        totalset.erase(totalset.begin()+j);
    }
    
    // return sample
    return sample;
}

Odometry::result Odometry::updateParameters(Matches* matches,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps)
{
    // we need at least 3 observations
    if (active.size()<3)
        return FAILED;
    
    // extract observations and compute predictions
    computeObservations(matches,active);
    computeResidualsAndJacobian(tr,active);

    // init
    slam2::Matrix A(6,6);
    slam2::Matrix B(6,1);

    // fill matrices A and B
    for (int32_t m=0; m<6; m++) {
        for (int32_t n=0; n<6; n++) {
        double a = 0;
        for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
            a += J[i*6+m]*J[i*6+n];
        }
        A.val[m][n] = a;
        }
        double b = 0;
        for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
        b += J[i*6+m]*(p_residual[i]);
        }
        B.val[m][0] = b;
    }

    // perform elimination
    if (B.solve(A)) {
        bool converged = true;
        for (int32_t m=0; m<6; m++) {
        tr[m] += step_size*B.val[m][0];
        if (fabs(B.val[m][0])>eps)
            converged = false;
        }
        if (converged)
        return CONVERGED;
        else
        return UPDATED;
    } else {
        return FAILED;
    }
}

std::vector<int32_t> Odometry::getInlier(Matches* matches, std::vector<double> &tr)
{
     // mark all observations active
    std::vector<int32_t> active;
    for (int i=0; i < matches->selectedMatches.size(); i++)
    {
        active.push_back(i);
    }

    // extract observations and compute predictions
    computeObservations(matches,active);
    computeResidualsAndJacobian(tr,active);

    // compute inliers
    std::vector<int32_t> inliers;
    for (int32_t i=0; i<(int32_t)matches->selectedMatches.size(); i++)
    {
        if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
            pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
        inliers.push_back(i);
    }
    return inliers;
}

void Odometry::computeObservations(Matches* matches,std::vector<int32_t> &active)
{
    // set all observations
    for (int32_t i=0; i<(int32_t)active.size(); i++) {
        p_observe[4*i+0] = matches->selectedMatches[active[i]]->u1c; // u1
        p_observe[4*i+1] = matches->selectedMatches[active[i]]->v1c; // v1
        p_observe[4*i+2] = matches->selectedMatches[active[i]]->u2c; // u2
        p_observe[4*i+3] = matches->selectedMatches[active[i]]->v2c; // v2
    }  
}

void Odometry::computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active)
{
    // extract motion parameters
    double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
    double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

    // precompute sine/cosine
    double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
    double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

    // compute rotation matrix and derivatives
    double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
    double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
    double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
    double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
    double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
    double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
    double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
    double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
    double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
    double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
    double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

    // loop variables
    double X1p,Y1p,Z1p;
    double X1c,Y1c,Z1c,X2c;
    double X1cd,Y1cd,Z1cd;

    // for all observations do
    for (int32_t i=0; i<(int32_t)active.size(); i++) {

        // get 3d point in previous coordinate system
        X1p = X[active[i]];
        Y1p = Y[active[i]];
        Z1p = Z[active[i]];

        // compute 3d point in current left coordinate system
        X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
        Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
        Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

        // weighting
        double weight = 1.0;
        if (param.reweighting)
        weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);
        
        // compute 3d point in current right coordinate system
        X2c = X1c-param.base;

        // for all paramters do
        for (int32_t j=0; j<6; j++) {

        // derivatives of 3d pt. in curr. left coordinates wrt. param j
        switch (j) {
            case 0: X1cd = 0;
                    Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                    Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                    break;
            case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                    Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                    Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                    break;
            case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                    Y1cd = rdrz10*X1p+rdrz11*Y1p;
                    Z1cd = rdrz20*X1p+rdrz21*Y1p;
                    break;
            case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
            case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
            case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
        }

        // set jacobian entries (project via K)
        J[(4*i+0)*6+j] = weight*param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
        J[(4*i+1)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
        J[(4*i+2)*6+j] = weight*param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
        J[(4*i+3)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
        }

        // set prediction (project via K)
        p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
        p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
        p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
        p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v
        
        // set residuals
        p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
        p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
        p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
        p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
    }
}


slam2::Matrix Odometry::transformationVectorToMatrix (std::vector<double> tr)
{
    // extract parameters
    double rx = tr[0];
    double ry = tr[1];
    double rz = tr[2];
    double tx = tr[3];
    double ty = tr[4];
    double tz = tr[5];

    // precompute sine/cosine
    double sx = sin(rx);
    double cx = cos(rx);
    double sy = sin(ry);
    double cy = cos(ry);
    double sz = sin(rz);
    double cz = cos(rz);

    // compute transformation
    slam2::Matrix Tr(4,4);
    Tr.val[0][0] = +cy*cz;          Tr.val[0][1] = -cy*sz;          Tr.val[0][2] = +sy;    Tr.val[0][3] = tx;
    Tr.val[1][0] = +sx*sy*cz+cx*sz; Tr.val[1][1] = -sx*sy*sz+cx*cz; Tr.val[1][2] = -sx*cy; Tr.val[1][3] = ty;
    Tr.val[2][0] = -cx*sy*cz+sx*sz; Tr.val[2][1] = +cx*sy*sz+sx*cz; Tr.val[2][2] = +cx*cy; Tr.val[2][3] = tz;
    Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
    return Tr;
}

bool Odometry::estimateRotation()
{
    std::vector<cv::Point> points1;
    std::vector<cv::Point> points2;
    cv::Mat mask;

    for (int i=0; i < matches->inlierMatches.size(); i++)
    {
        points1.push_back(cv::Point(matches->inlierMatches[i]->u1c, matches->inlierMatches[i]->v1c));
        points2.push_back(cv::Point(matches->inlierMatches[i]->u1p, matches->inlierMatches[i]->v1p));
    }

    if (points1.size() < 5)
    {
        rotationDepth = 0;
        return false;
    }

    essMat = cv::findEssentialMat(points1, points2, param.calib.f, cv::Point(param.calib.cu, param.calib.cv),
            cv::RANSAC, 0.999, 1.0, mask);

    points1.clear();
    points2.clear();

    for (int i=0; i < matches->inlierMatches.size(); i++)
    {
        if (matches->inlierMatches[i]->age > 1)
        {
            points1.push_back(cv::Point(matches->inlierMatches[i]->u1c, matches->inlierMatches[i]->v1c));
            points2.push_back(cv::Point(matches->inlierMatches[i]->u1p2, matches->inlierMatches[i]->v1p2));
        }
    }

    if (points1.size() < 5)
    {
        rotationDepth = 1;
        return true;
    }
    
    essMat2 = cv::findEssentialMat(points1, points2, param.calib.f, cv::Point(param.calib.cu, param.calib.cv),
            cv::RANSAC, 0.999, 1.0, mask);
    
    points1.clear();
    points2.clear();

    for (int i=0; i < matches->inlierMatches.size(); i++)
    {
        if (matches->inlierMatches[i]->age > 2)
        {
            points1.push_back(cv::Point(matches->inlierMatches[i]->u1c, matches->inlierMatches[i]->v1c));
            points2.push_back(cv::Point(matches->inlierMatches[i]->u1p3, matches->inlierMatches[i]->v1p3));
        }
    }

    if (points1.size() < 5)
    {
        rotationDepth = 2;
        return true;
    }
    
    essMat3 = cv::findEssentialMat(points1, points2, param.calib.f, cv::Point(param.calib.cu, param.calib.cv),
            cv::RANSAC, 0.999, 1.0, mask);
    
    rotationDepth = 3;
    return true;
}

bool Odometry::convertRotations()
{
    //quaternions
    Eigen::Quaternionf qEss;
    Eigen::Quaternionf qEss2;
    Eigen::Quaternionf qEss3;

    Eigen::Quaternionf qRs;
    Eigen::Quaternionf qRs2;
    Eigen::Quaternionf qRs3;

    if (rotationDepth ==0)
        return false;
    
    if (rotationDepth > 0)
    {
        Eigen::Matrix3f eig;
        cv::cv2eigen(essMat, eig);
        qEss = eig;

        qR = qEss;
        qRs = qEss;
    }

    if (rotationDepth > 1)
    {
        Eigen::Matrix3f eig;
        cv::cv2eigen(essMat2, eig);
        qEss2 = eig;

        qRs2 = qR2.inverse() * qEss2;
        qR = qR.slerp(0.5, qRs2); 
        //Eigen::Quaternionf qTemp = sqrt(qR.inverse() * qRs2);
        //qR = qR * (^0.5);
    }

    if (rotationDepth > 2)
    {
        Eigen::Matrix3f eig;
        cv::cv2eigen(essMat3, eig);
        qEss3 = eig; 

        qRs3 = qR3.inverse() * qEss3;
        qR = qR.slerp(0.3333333, qRs3); 
    }

    
    return true;
}

float Odometry::getRotationError(int index)
{
    if (groundTruth.size() <= index)
        return 9999999;

     // get rotation angle between this keyframe and last keyframe on stack    
    slam2::Matrix rA = pose.getMat(0, 0, 2, 2);
    slam2::Matrix rB = groundTruth[index].getMat(0, 0, 2, 2);

    slam2::Matrix rAT = rA.operator~();
    slam2::Matrix rAB = rAT.operator*(rB);

    // calculate trace
    float trace = rAB.val[0][0] + rAB.val[1][1] + rAB.val[2][2];

    if (trace > 3)
        trace = 3;

    float f1 = acos((trace - 1) / 2);
    float f2 = (f1 * 180) / M_PI;

    //printf("trace: %f, f1: %f f2:%f f5: %f f6: %f\n", trace, f1, f2, f5, f6);

    return f2;
}
    
float Odometry::getTranslationError(int index)
{
    if (groundTruth.size() <= index)
        return 9999999;

    // get euclidian distance between this keyframe and last keyframe on stack
    float f1, f2, f3;

    f1 = pose.val[0][3] - groundTruth[index].val[0][3];
    f2 = pose.val[1][3] - groundTruth[index].val[1][3];
    f3 = pose.val[2][3] - groundTruth[index].val[2][3];

    return fabs(sqrt(f1*f1 + f2*f2 + f3*f3));    
}

float Odometry::getMotionError(int index)
{
    // get euclidian distance between this keyframe and last keyframe on stack
    float f1, f2, f3;
    float r1, r2, r3;
    float gf1, gf2, gf3;

    if (index == 0)
    {
        gf1 = groundTruth[index].val[0][3];
        gf2 = groundTruth[index].val[1][3];
        gf3 = groundTruth[index].val[2][3];

        f1 = pose.val[0][3];
        f2 = pose.val[1][3];
        f3 = pose.val[2][3];
    }
    else
    {
        gf1 = groundTruth[index].val[0][3] - groundTruth[index - 1].val[0][3];
        gf2 = groundTruth[index].val[1][3] - groundTruth[index - 1].val[1][3];
        gf3 = groundTruth[index].val[2][3] - groundTruth[index - 1].val[2][3];


        f1 = pose.val[0][3] - posePrior.val[0][3];
        f2 = pose.val[1][3] - posePrior.val[1][3];
        f3 = pose.val[2][3] - posePrior.val[2][3];
    }
    

    r1 = f1 - gf1;
    r2 = f2 - gf2;
    r3 = f3 - gf3;

    groundTruthMotion = fabs(sqrt(gf1*gf1 + gf2*gf2 + gf3*gf3)); 
    computedMotion = fabs(sqrt(f1*f1 + f2*f2 + f3*f3)); 
    totalMotion += groundTruthMotion;
    posePrior = slam2::Matrix(pose);
    //printf("gtm: %f cm: %f tm: %f return: %f\n", groundTruthMotion, computedMotion, totalMotion,fabs(sqrt(r1*r1 + r2*r2 + r3*r3)));
    //printf("f1: %f f2: %f f3: %f gf1: %f gf2: %f gf3: %f r1: %f r2: %f r3: %f\n", f1, f2, f3, gf1,gf2,gf3,r1,r2,r3);
    return fabs(sqrt(r1*r1 + r2*r2 + r3*r3));
    
}

float Odometry::getAverageTranslationError()
{
    return groundTruthTranslationError/frameProcessedCount;
}

float Odometry::getAverageMotionError()
{ 
    return groundTruthMotionError/frameProcessedCount;
}

float Odometry::getPercentageMotionError()
{
    return getAverageMotionError()/totalMotion * 100;
}

void Odometry::calculateDepth()
{
    for (int i=0; i < matches->selectedMatches.size(); i++)
    {
        double d = std::max(matches->selectedMatches[i]->u1c - matches->selectedMatches[i]->u2c,0.0001f);
        d = param.calib.f*param.base/d;
        matches->selectedMatches[i]->depth = d;

        if (d > 2000)
        {
            printf("Crazy Depth: %f u1c: %f u2c: %f\n", d, matches->selectedMatches[i]->u1c, matches->selectedMatches[i]->u2c);
        }
    }   
}

slam2::Matrix Odometry::estimateMotion3(bool* result)
{
    printf("estimateMotion3 start\n");
    // get number of matches
    slam2::Matrix Tr(4,4);
  int32_t N = matches->selectedMatches.size();
  if (N<10)
  {
    * result = false;
    return Tr;
  }

    printf("estimateMotion3 1\n");
  
  // initial RANSAC estimate of F
  inliers.clear();
  double points1[16];
  double points2[16];

    std::vector<EMatrix> E; // essential matrix
    std::vector<PMatrix> P; // 3x4 projection matrix
    std::vector<int> inl;

    PMatrix bestP;

  for (int32_t k=0;k<param.mono_ransac_iters;k++) {

    // draw random sample 
    std::vector<int32_t> active = getRandomSample(N,8);

    if (active.size() < 8)
        break;

    for (int i=0; i < active.size(); i++)
    {
        points1[i*2] = matches->selectedMatches[active[i]]->u1p;
        points1[i*2 + 1] = matches->selectedMatches[active[i]]->v1p;

        points2[i*2] = matches->selectedMatches[active[i]]->u1c;
        points2[i*2 + 1] = matches->selectedMatches[active[i]]->v1c;

        //printf("points x1:%f y1:%f x2:%f y2:%f\n", points1[i*2], points1[i*2+1], points2[i*2], points2[i*2+1]); 
    }

    bool ret = Solve5PointEssential(points1, points2, 8, E, P, inl);

    if (!ret)
        continue;

    EMatrix bestE;
    PMatrix bestPTemp;
    int bestInliers = 0;

    for (int z=0; z < E.size(); z++)
    {
        if (inl[z] > bestInliers)
        {
            //bestE = E[z];
            bestPTemp = PMatrix(P[z]);
            bestInliers = inl[z];
        }
    }
    
    std::vector<int32_t> inliers_curr = getInliers5Point(bestPTemp);

    // update model if we are better
     if (inliers_curr.size()>inliers.size())
     {
       inliers = inliers_curr;
       bestP = PMatrix(bestPTemp);
     }
  }

  printf("estimateMotion3 2\n");
  
  if(bestP.block(0,0,3,3).determinant() < 0) {
      bestP = bestP * -1;
      printf("estimateMotion3 invert\n");
    }
  
  // are there enough inliers?
  if (inliers.size()<10)
  {
    *result = false;
    return Tr;
  }

  printf("estimateMotion3 3\n");

  printf("Updatemotion3: inliers: %i\n", static_cast<int>(inliers.size()));

  *result = true;

    Tr.val[0][0] = bestP(0,0);Tr.val[0][1] = bestP(0,1);Tr.val[0][2] = bestP(0,2);Tr.val[0][3] = bestP(0,3);
    Tr.val[1][0] = bestP(1,0);Tr.val[1][1] = bestP(1,1);Tr.val[1][2] = bestP(1,2);Tr.val[1][3] = bestP(1,3);
    Tr.val[2][0] = bestP(2,0);Tr.val[2][1] = bestP(2,1);Tr.val[2][2] = bestP(2,2);Tr.val[2][3] = bestP(2,3);
    Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
    return Tr;
  
  
  // return parameter vector
  //std::vector<double> tr_delta;
//   tr_delta.resize(6);
//   tr_delta[0] = rx;
//   tr_delta[1] = ry;
//   tr_delta[2] = rz;
//   tr_delta[3] = t.val[0][0];
//   tr_delta[4] = t.val[1][0];
//   tr_delta[5] = t.val[2][0];
  return Tr;
}

std::vector<int32_t> Odometry::getInliers5Point(PMatrix P)
{
    int best_inliers = 0;
    bool found = false;
    PMatrix P_ref = PMatrix::Identity();
    std::vector<int32_t> inliers;

    for(int i=0; i < matches->selectedMatches.size(); i++) {
        Matches::p_match* match = matches->selectedMatches[i];
        Eigen::Vector4d pt3d = TriangulatePoint(match->u1p, match->v1p, match->u1c, match->u2c, P_ref, P);
        double depth1 = CalcDepth(pt3d, P_ref);
        double depth2 = CalcDepth(pt3d, P);

        if(depth1 > 0 && depth2 > 0){
            inliers.push_back(i);
        }
    }

    return inliers;
}