#include "G2ODriver.h"

G2ODriver::G2ODriver()
{
    optimizer.setVerbose( true );

    // auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    // linearSolver->setBlockOrdering(false);
    // OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    // g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    // linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    // std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    // g2o::BlockSolver_6_3 * solver_ptr;// = new g2o::BlockSolver_6_3(linearSolver);
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    // optimizer.setAlgorithm(solver);


    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    optimizeCount = 0;
}

void G2ODriver::createModel(std::vector<KeyFrame> keyFrames, int optimizationCount)
{
    optimizeCount = optimizationCount;
    /* The uncertainty matrix: Represents the inverse covariance of the measurement, and thus is symmetric
    * and positive definite. */
    Eigen::Matrix<double,6,6> uncertainty = Eigen::Matrix<double,6,6>::Identity();
    
    g2o::VertexSE3* currentVertex = nullptr;
    g2o::VertexSE3* previousVertex = currentVertex;

    //add poses
    for (int i=0; i < keyFrames.size(); i++)
    {
        KeyFrame kf = keyFrames[i];
        currentVertex = new g2o::VertexSE3();
        currentVertex->setId(kf.index);
        currentVertex->setEstimate(getEigenPose(kf.pose));

        if (i > 0)
            currentVertex->setFixed(false);
        else
        {
            currentVertex->setFixed(true);
        }
        
        
        optimizer.addVertex(currentVertex);

        /* Adding edges between sucesives keyframes of the map */
        if(i > 0) {
            // Pose of current in previous's reference frame, t = Tpc (takes points in current's reference and returns it in previous's reference)
            Eigen::Isometry3d t = previousVertex->estimate().inverse() * currentVertex->estimate();
            g2o::EdgeSE3* e = new g2o::EdgeSE3;
            e->setVertex(0, previousVertex);
            e->setVertex(1, currentVertex);
            e->setMeasurement(t);
            // e->information() = uncertainty;
            e->information() = getUncertaintyMatrix(kf, keyFrames[i-1]);
            optimizer.addEdge(e);
        }

        previousVertex = currentVertex;
    }

    //add poses
    for (int i=0; i < keyFrames.size(); i++)
    {
        KeyFrame kf = keyFrames[i];

        if (kf.loopKeyFrames.size() < 1)
            continue;

        currentVertex = (g2o::VertexSE3*)optimizer.vertex(kf.index);

        for (int y=0; y < kf.loopKeyFrames.size(); y++)
        {
            KeyFrameLoop lkf = kf.loopKeyFrames[y];
            KeyFrame loopKeyFrame = keyFrames[lkf.targetKeyFrame];
            g2o::VertexSE3* loopClosureVertex = (g2o::VertexSE3*)optimizer.vertex(loopKeyFrame.index);

            Eigen::Isometry3d t = getEigenPose(lkf.pose).inverse();//.inverse() * getEigenPose(lkf.pose); //  = previousVertex->estimate().inverse() * currentVertex->estimate();
            g2o::EdgeSE3* e = new g2o::EdgeSE3;
            e->setVertex(0, currentVertex);
            e->setVertex(1, loopClosureVertex);
            e->setMeasurement(t);
            e->information() = uncertainty;
            //e->information() = getUncertaintyMatrix(kf, loopKeyFrame);
            bool result = optimizer.addEdge(e);

            printf("LKF. %i %i % i\n", kf.index, loopKeyFrame.index, result);
        }

    }
    //add constraints
}

void G2ODriver::optimize()
{

    std::string fileName = "optimizer_before";
    fileName.append(std::to_string(optimizeCount));
    fileName.append(".g2o");

    optimizer.save(fileName.c_str());

    optimizer.initializeOptimization();
    int result = optimizer.optimize(10);

    printf("OptimizerResult: %i\n", result);

    fileName = "optimizer_after";
    fileName.append(std::to_string(optimizeCount));
    fileName.append(".g2o");
    optimizer.save(fileName.c_str());

    optimizeCount++;
}

Eigen::Isometry3d G2ODriver::getEigenPose(slam2::Matrix pose)
{
    std::cout << "getEigenPose\n" << pose << "\n";

    Mat33 omat;
    omat(0, 0) = pose.val[0][0];
    omat(0, 1) = pose.val[0][1];
    omat(0, 2) = pose.val[0][2];
    omat(1, 0) = pose.val[1][0];
    omat(1, 1) = pose.val[1][1];
    omat(1, 2) = pose.val[1][2];
    omat(2, 0) = pose.val[2][0];
    omat(2, 1) = pose.val[2][1];
    omat(2, 2) = pose.val[2][2];

    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = omat;
    iso.translation() = Eigen::Vector3d(pose.val[0][3], pose.val[1][3], pose.val[2][3]);

    printf("getEigenTranslation %f %f %f\n", iso.translation().x(), iso.translation().y(), iso.translation().z()); 

    // Eigen::Isometry3d keyframe_pose = Eigen::Isometry3d::Identity();
    // keyframe_pose.linear() = keyframe->GetCameraPose().GetOrientationMatrix();
    // keyframe_pose.translation() = keyframe->GetCameraPose().GetPosition();

    return iso;
}

Eigen::Matrix<double,6,6> G2ODriver::getUncertaintyMatrix(KeyFrame kf, KeyFrame kf2)
{
    Eigen::Matrix<double,6,6> uc = Eigen::Matrix<double,6,6>::Identity();

    // return uc;

    // uc(0,0) = fabs(kf.pose.val[3][0] - kf2.pose.val[3][0]);
    // uc(1,1) = fabs(kf.pose.val[3][1] - kf2.pose.val[3][1]);
    // uc(2,2) = fabs(kf.pose.val[3][2] - kf2.pose.val[3][2]);
    // uc(3,3) = getRotationAngle(kf, kf2);

    return uc * getRotationAngle(kf, kf2) * getTranslationDistance(kf, kf2);
}

float G2ODriver::getRotationAngle(KeyFrame keyFrame, KeyFrame keyFrame2)
{
    // get rotation angle between the two keyframes    

    slam2::Matrix rA = keyFrame.pose.getMat(0, 0, 2, 2);
    slam2::Matrix rB = keyFrame2.pose.getMat(0, 0, 2, 2);

    slam2::Matrix rAT = rA.operator~();
    slam2::Matrix rAB = rAT.operator*(rB);

    // calculate trace
    float trace = rAB.val[0][0] + rAB.val[1][1] + rAB.val[2][2];

    if (trace > 3)
        trace = 3;

    float f1 = acos((trace - 1) / 2);
    float f2 = (f1 * 180) / M_PI;

    //printf("rotation angle: %f\n", f2);
    //printf("trace: %f, f1: %f f2:%f f5: %f f6: %f\n", trace, f1, f2, f5, f6);

    return f2;
}

float G2ODriver::getTranslationDistance(KeyFrame keyFrame, KeyFrame keyFrame2)
{
    // get euclidian distance between the two keyframes
    float f1, f2, f3;

    f1 = keyFrame.pose.val[0][3] - keyFrame2.pose.val[0][3];
    f2 = 0;//keyFrame.pose.val[1][3] - keyFrame2.pose.val[1][3];
    f3 = keyFrame.pose.val[2][3] - keyFrame2.pose.val[2][3];

    return fabs(sqrt(f1*f1 + f2*f2 + f3*f3));    
}
