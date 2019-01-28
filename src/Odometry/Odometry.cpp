#include "Odometry.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <chrono>

using namespace std::chrono;

Odometry::Odometry(SlamViewer* viewer, Mapping* mapping, cv::Mat cameraMatrix, float baseLine)
{
    this->viewer = viewer;
    this->mapping = mapping;

    Tr_valid = false;
    features = new Features();

    // f_x = 0,0
    // f_y = 1,1
    // c_x = 0,2
    // c_y = 1,2
    param.base = baseLine;
    param.calib.f = cameraMatrix.at<float>(0, 0);
    param.calib.cu = cameraMatrix.at<float>(0, 2);
    param.calib.cv = cameraMatrix.at<float>(1, 2);

    matcher = new Matcher(Matcher::parameters());
    matcher->setIntrinsics(param.calib.f, param.calib.cu, param.calib.cv, param.base);

    timer = new Timer();
}

Odometry::~Odometry()
{
    timer->outputAll();
    delete timer;
    delete matcher;
}

bool Odometry::addStereoFrames(SLImage* image, SLImage* imageRight)
{    
    int32_t dims[] = {image->w, image->h, image->w};

    uint8_t* left_image = getImageArray(image);
    uint8_t* right_image = getImageArray(imageRight);

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
        matcher->bucketFeaturesSTA(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
        timer->stopTimer();

        timer->startTimer("getMatches");
        p_matched = matcher->getMatches();
        timer->stopTimer();
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

    printf("Num matches pre bucket: %i\n", static_cast<int>(matcher->getMatches().size()));
    

    timer->startTimer("bucketFeatures2");
    //matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
    matcher->bucketFeaturesSTA(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);                          
    timer->stopTimer();

    timer->startTimer("getMatches");
    p_matched = matcher->getMatches();
    timer->stopTimer();

    timer->startTimer("updateMotion");
    bool result = updateMotion();
    timer->stopTimer();

    for (int i=0; i < p_matched.size(); i++)
    {
        cv::Point2f matchPoint(p_matched[i].u1c, p_matched[i].v1c);
        cv::Point2f matchPointRight(p_matched[i].u2c, p_matched[i].v2c);
        cv::circle(image->imageColor, matchPoint, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(imageRight->imageColor, matchPointRight, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    }

    free(left_image);
    free(right_image);


    viewer->pushLiveImageFrame(image->imageColor, imageRight->imageColor);
    
    // possible duplicate
    //bool result = updateMotion();

    if (result)
    {
        // on success, update current pose
        pose = pose * Matrix::inv(getMotion());
        
        // output some statistics
        double num_matches = getNumberOfMatches();
        double num_inliers = getNumberOfInliers();
        std::cout << ", Matches: " << num_matches;
        std::cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << std::endl;
        std::cout << pose << std::endl << std::endl;

        mapping->addFrame(pose, image, imageRight, p_matched);
    }
    else
    {
        std::cout << " ... failed!" << std::endl;
    }

    return result;
}


bool Odometry::updateMotion()
{
    // estimate motion
    std::vector<double> tr_delta = estimateMotion(p_matched);
    
    // on failure
    if (tr_delta.size()!=6)
        return false;
    
    // set transformation matrix (previous to current frame)
    Tr_delta = transformationVectorToMatrix(tr_delta);
    Tr_valid = true;
    
    // success
    return true;
}

std::vector<double> Odometry::estimateMotion (std::vector<Matcher::p_match> p_matched)
{
    // return value
    bool success = true;

    // compute minimum distance for RANSAC samples
    double width=0,height=0;
    for (std::vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
        if (it->u1c>width)  width  = it->u1c;
        if (it->v1c>height) height = it->v1c;
    }
    double min_dist = std::min(width,height)/3.0;
    
    // get number of matches
    int32_t N  = p_matched.size();
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

    // project matches of previous image into 3d
    for (int32_t i=0; i<N; i++) {
        double d = std::max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
        X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
        Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
    }

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
        res = updateParameters(p_matched,active,tr_delta_curr,1,1e-6);
        if (iter++ > 20 || res==CONVERGED)
            break;
        }

        // overwrite best parameters if we have more inliers
        if (res !=FAILED) {
        std::vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
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
        res = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
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
    else         return std::vector<double>();
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

Odometry::result Odometry::updateParameters(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps)
{
    // we need at least 3 observations
    if (active.size()<3)
        return FAILED;
    
    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobian(tr,active);

    // init
    Matrix A(6,6);
    Matrix B(6,1);

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

std::vector<int32_t> Odometry::getInlier(std::vector<Matcher::p_match> &p_matched, std::vector<double> &tr)
{
     // mark all observations active
    std::vector<int32_t> active;
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
        active.push_back(i);

    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobian(tr,active);

    // compute inliers
    std::vector<int32_t> inliers;
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
        if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
            pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
        inliers.push_back(i);
    return inliers;
}

void Odometry::computeObservations(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active)
{
    // set all observations
    for (int32_t i=0; i<(int32_t)active.size(); i++) {
        p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
        p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
        p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
        p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
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


Matrix Odometry::transformationVectorToMatrix (std::vector<double> tr)
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
    Matrix Tr(4,4);
    Tr.val[0][0] = +cy*cz;          Tr.val[0][1] = -cy*sz;          Tr.val[0][2] = +sy;    Tr.val[0][3] = tx;
    Tr.val[1][0] = +sx*sy*cz+cx*sz; Tr.val[1][1] = -sx*sy*sz+cx*cz; Tr.val[1][2] = -sx*cy; Tr.val[1][3] = ty;
    Tr.val[2][0] = -cx*sy*cz+sx*sz; Tr.val[2][1] = +cx*sy*sz+sx*cz; Tr.val[2][2] = +cx*cy; Tr.val[2][3] = tz;
    Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
    return Tr;
}

uint8_t* Odometry::getImageArray(SLImage* image)
{
    uint8_t* img_data  = (uint8_t*)malloc(image->w*image->h*sizeof(uint8_t));
    int32_t k=0;

    for (int32_t v=0; v<image->h; v++) {
    for (int32_t u=0; u<image->w; u++) {
        img_data[k]  = image->image.at<uint8_t>(v,u); 
        k++;
    }
    }

    return img_data;
}