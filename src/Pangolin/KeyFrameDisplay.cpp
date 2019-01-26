#include "KeyFrameDisplay.h"
#include <stdio.h>
#include <pangolin/pangolin.h>

KeyFrameDisplay::KeyFrameDisplay()
{

}
       
KeyFrameDisplay::~KeyFrameDisplay()
{

}


// copies & filters internal data to GL buffer for rendering. if nothing to do: does nothing.
bool KeyFrameDisplay::refreshPC(bool canRefresh, float scaledTH, float absTH, int mode, float minBS, int sparsity)
{
    return false;
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

		glPointSize(pointSize);


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
    if(cx != 0 || cy != 0)
        return;

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