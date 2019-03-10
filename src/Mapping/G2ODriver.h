#pragma once


#include "KeyFrame.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;
using namespace g2o;

class G2ODriver
{
    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    public:
        G2ODriver();
        void optimize();
        void createModel(std::vector<KeyFrame> keyFrames);

    private:
        // G2O optimizer
        g2o::SparseOptimizer optimizer;
};