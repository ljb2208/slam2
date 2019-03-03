#include "KeyFrameDisplay.h"
#include <stdio.h>
#include <pangolin/pangolin.h>

KeyFrameDisplay::KeyFrameDisplay(KeyFrame keyFrame)
{
    this->keyFrame = keyFrame;
    fx = 718.856;
    fy = 718.856;
    cx = 607.1928;
    cy = 185.2157;

	// fx = 370.500793;
    // fy = 918.3968;
    // cx = 312.904266;
    // cy = 236.561417;

    //originalInputSparse = 0;
	numSparseBufferSize=0;
	numSparsePoints=0;

	id = 0;
	active= true;
	Vec3d trans;			

	trans(0) = keyFrame.pose.val[0][3];
	trans(1) = keyFrame.pose.val[1][3];
	trans(2) = keyFrame.pose.val[2][3];

	Mat33 rot;

	rot(0, 0) = keyFrame.pose.val[0][0];
	rot(0, 1) = keyFrame.pose.val[0][1];
	rot(0, 2) = keyFrame.pose.val[0][2];

	rot(1, 0) = keyFrame.pose.val[1][0];
	rot(1, 1) = keyFrame.pose.val[1][1];
	rot(1, 2) = keyFrame.pose.val[1][2];

	rot(2, 0) = keyFrame.pose.val[2][0];
	rot(2, 1) = keyFrame.pose.val[2][1];
	rot(2, 2) = keyFrame.pose.val[2][2];

	camToWorld =  SE3(rot, trans);

	needRefresh=true;

	my_scaledTH =1e10;
	my_absTH = 1e10;
	my_displayMode = 1;
	my_minRelBS = 0;
	my_sparsifyFactor = 1;

	numGLBufferPoints=0;
	bufferValid = false;

	width = keyFrame.width;
	height = keyFrame.height;	
}
       
KeyFrameDisplay::~KeyFrameDisplay()
{

}

Matrix KeyFrameDisplay::getPose()
{
    return keyFrame.pose;
}


// copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
bool KeyFrameDisplay::refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity)
{
    if(canRefresh)
	{
		needRefresh = needRefresh ||
				my_scaledTH != scaledTH ||
				my_absTH != absTH ||
				my_displayMode != mode ||
				my_minRelBS != minBS ||
				my_sparsifyFactor != sparsity;
	}

	if(!needRefresh) return false;
	needRefresh=false;

	my_scaledTH = scaledTH;
	my_absTH = absTH;
	my_displayMode = mode;
	my_minRelBS = minBS;
	my_sparsifyFactor = sparsity;


    numSparsePoints = keyFrame.p_matched.size();

	// if there are no vertices, done!
	if(numSparsePoints == 0)
		return false;

	printf("refreshpc. points: %i\n", numSparsePoints);

    int patternNum = 8;

	// make data
	Vec3f* tmpVertexBuffer = new Vec3f[numSparsePoints];
	Vec3b* tmpColorBuffer = new Vec3b[numSparsePoints];
	int vertexBufferNumPoints=0;

	for(int i=0;i<numSparsePoints;i++)
	{
		/* display modes:
		 * my_displayMode==0 - all pts, color-coded
		 * my_displayMode==1 - normal points
		 * my_displayMode==2 - active only
		 * my_displayMode==3 - nothing
		 */

		// if(my_displayMode==1 && originalInputSparse[i].status != 1 && originalInputSparse[i].status!= 2) continue;
		// if(my_displayMode==2 && originalInputSparse[i].status != 1) continue;
		// if(my_displayMode>2) continue;

		if(keyFrame.p_matched[i].depth < 0) continue;

		float depth = keyFrame.p_matched[i].depth;
		float depth4 = depth*depth; depth4*= depth4;
		// float var = (1.0f / (originalInputSparse[i].idepth_hessian+0.01));

		// if(var * depth4 > my_scaledTH)
		// 	continue;

		// if(var > my_absTH)
		// 	continue;

		// if(originalInputSparse[i].relObsBaseline < my_minRelBS)
		// 	continue;

		float z = depth;// / my_scale;
		float x = (keyFrame.p_matched[i].u1c - cx)*z*(1.0f/fx);
		float y = (keyFrame.p_matched[i].v1c - cy)*z*(1.0f/fy);
		//float x = z *(cx - keyFrame.p_matched[i].u1c)  / fx;
		//float y = z *(cy - keyFrame.p_matched[i].v1c) / fy;

		printf("x:%f y:%f z:%f\n", x, y, z);

		tmpVertexBuffer[vertexBufferNumPoints][0] = x;
		tmpVertexBuffer[vertexBufferNumPoints][1] = y; 
		tmpVertexBuffer[vertexBufferNumPoints][2] = z;

		tmpColorBuffer[vertexBufferNumPoints][0] = 255;
		tmpColorBuffer[vertexBufferNumPoints][1] = 255;
		tmpColorBuffer[vertexBufferNumPoints][2] = 255;

		vertexBufferNumPoints++;

	}

	if(vertexBufferNumPoints==0)
	{
		delete[] tmpColorBuffer;
		delete[] tmpVertexBuffer;
		return true;
	}

	numGLBufferGoodPoints = vertexBufferNumPoints;
	if(numGLBufferGoodPoints > numGLBufferPoints)
	{
		numGLBufferPoints = vertexBufferNumPoints;//*1.3;
		vertexBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_FLOAT, 3, GL_DYNAMIC_DRAW );
		colorBuffer.Reinitialise(pangolin::GlArrayBuffer, numGLBufferPoints, GL_UNSIGNED_BYTE, 3, GL_DYNAMIC_DRAW );
	}
	vertexBuffer.Upload(tmpVertexBuffer, sizeof(float)*3*numGLBufferGoodPoints, 0);
	colorBuffer.Upload(tmpColorBuffer, sizeof(unsigned char)*3*numGLBufferGoodPoints, 0);
	bufferValid=true;
	delete[] tmpColorBuffer;
	delete[] tmpVertexBuffer;


	return true;
}

// renders cam & pointcloud.
void KeyFrameDisplay::drawCam(float lineWidth, float* color, float sizeFactor)
{
    if(width == 0)
		return;

	float sz=sizeFactor;	

	glPushMatrix();
    
    Sophus::Matrix4f m = camToWorld.matrix().cast<float>();	
    glMultMatrixf((GLfloat*)m.data());

    if(color == 0)
    {
        glColor3f(1,0,0);
    }
    else
        glColor3f(color[0],color[1],color[2]);

    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);
    glVertex3f(0,0,0);
    glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);
    glVertex3f(0,0,0);
    glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);
    glVertex3f(0,0,0);
    glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);

    glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);
    glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);

    glVertex3f(sz*(width-1-cx)/fx,sz*(height-1-cy)/fy,sz);
    glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);

    glVertex3f(sz*(0-cx)/fx,sz*(height-1-cy)/fy,sz);
    glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);

    glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);
    glVertex3f(sz*(width-1-cx)/fx,sz*(0-cy)/fy,sz);

	

    glEnd();
	glPopMatrix();
    
}

void KeyFrameDisplay::drawPC(float pointSize)
{
    if(!bufferValid || numGLBufferGoodPoints==0)
		return;

	glDisable(GL_LIGHTING);

	glPushMatrix();
				
		Sophus::Matrix4f m = camToWorld.matrix().cast<float>();	
    	glMultMatrixf((GLfloat*)m.data());
		
		colorBuffer.Bind();
		glColorPointer(colorBuffer.count_per_element, colorBuffer.datatype, 0, 0);
		glEnableClientState(GL_COLOR_ARRAY);

		vertexBuffer.Bind();
		glVertexPointer(vertexBuffer.count_per_element, vertexBuffer.datatype, 0, 0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glDrawArrays(GL_POINTS, 0, numGLBufferGoodPoints);
		glDisableClientState(GL_VERTEX_ARRAY);
		vertexBuffer.Unbind();

		glDisableClientState(GL_COLOR_ARRAY);
		colorBuffer.Unbind();

	glPopMatrix();
}

void KeyFrameDisplay::drawGTCam(Sophus::Matrix4f m, float lineWidth, float* color, float sizeFactor)
{
    //if(cx != 0 || cy != 0)
    //   return;

    //printf("cx/cy set\n");

    float sz=sizeFactor;

    glPushMatrix();

    glMultMatrixf((GLfloat*)m.data());

    if(color == 0)
    {
        glColor3f(1,0,0);
    }
    else
        glColor3f(color[0],color[1],color[2]);

    glLineWidth(lineWidth);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(sz*(0-cx)/fx,sz*(0-cy)/fy,sz);

    glEnd();
    glPopMatrix();
}