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
    //add poses
    for (int i=0; i < keyFrames.size(); i++)
    {
        KeyFrame kf = keyFrames[i];
        //VertexSE3* vert =  new VertexSE3;
        //vert->setId(kf.index);
        //vert->setEstimate(t);
        //optimizer.addVertex(vert);
    }

    //add constraints
}

void G2ODriver::optimize()
{
    optimizer.initializeOptimization();
    optimizer.optimize(10);
}


