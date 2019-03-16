#include "G2ODriver.h"

G2ODriver::G2ODriver()
{
    optimizer.setVerbose( false );

    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);
    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);
}

void G2ODriver::createModel(std::vector<KeyFrame> keyFrames)
{
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
        currentVertex->setId(i);
        currentVertex->setEstimate(getEigenPose(kf));

        if (i == 0)
            currentVertex->setFixed(true);
        else
            currentVertex->setFixed(false);
        
        optimizer.addVertex(currentVertex);

        /* Adding edges between sucesives keyframes of the map */
        if(i > 0) {
            // Pose of current in previous's reference frame, t = Tpc (takes points in current's reference and returns it in previous's reference)
            Eigen::Isometry3d t = previousVertex->estimate().inverse() * currentVertex->estimate();
            g2o::EdgeSE3* e = new g2o::EdgeSE3;
            e->setVertex(0, previousVertex);
            e->setVertex(1, currentVertex);
            e->setMeasurement(t);
            e->information() = uncertainty;
            optimizer.addEdge(e);
        }

        

        previousVertex = currentVertex;
    }
    //add constraints
}

void G2ODriver::optimize()
{
    optimizer.initializeOptimization();
    optimizer.optimize(10);
}

Eigen::Isometry3d G2ODriver::getEigenPose(KeyFrame kf)
{
    Mat33 omat;
    omat(0, 0) = kf.pose.val[0][0];
    omat(0, 1) = kf.pose.val[0][1];
    omat(0, 2) = kf.pose.val[0][2];
    omat(1, 0) = kf.pose.val[1][0];
    omat(1, 1) = kf.pose.val[1][1];
    omat(1, 2) = kf.pose.val[1][2];
    omat(2, 0) = kf.pose.val[2][0];
    omat(2, 1) = kf.pose.val[2][1];
    omat(2, 2) = kf.pose.val[2][2];

    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = omat;
    iso.translation() << kf.pose.val[3][0], kf.pose.val[3][1], kf.pose.val[3][2];


    // Eigen::Isometry3d keyframe_pose = Eigen::Isometry3d::Identity();
    // keyframe_pose.linear() = keyframe->GetCameraPose().GetOrientationMatrix();
    // keyframe_pose.translation() = keyframe->GetCameraPose().GetPosition();

    return iso;
}

