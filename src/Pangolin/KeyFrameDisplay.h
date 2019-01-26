#pragma once

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include "Util/NumType.h"

#include <sstream>
#include <fstream>

class KeyFrameDisplay
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        KeyFrameDisplay();
        ~KeyFrameDisplay();


        // copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
        bool refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity);

        // renders cam & pointcloud.
        void drawCam(float lineWidth = 1, float* color = 0, float sizeFactor=1);
        void drawPC(float pointSize);

        // render ground truth
        void drawGTCam(Sophus::Matrix4f m, float lineWidth, float* color, float sizeFactor);

        Sophus::SE3 camToWorld;

    private:
        float fx,fy,cx,cy;
	    float fxi,fyi,cxi,cyi;
        int width, height;

        bool bufferValid;
        int numGLBufferPoints;
        int numGLBufferGoodPoints;
        pangolin::GlBuffer vertexBuffer;
        pangolin::GlBuffer colorBuffer;
};