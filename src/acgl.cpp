/*************
Copyright (c) 2011, Allen Choong
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the project nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

/*
Author:    Allen
Date:    20090223
Description:
acgl.cpp handles the OpenGL related functions
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <GL/gl.h>
#include <GL/glut.h>
#include "opencvar/acgl.h"

using namespace std;

void acGlTexture(unsigned char* data, int width, int height, int depth,
                 int swap) {
    if (depth != 3 && depth != 4)
        perror("acGlTexture: byte either 3 or 4\n");

    GLuint texid[1];
    glGenTextures(1, texid);
    glBindTexture(GL_TEXTURE_2D, texid[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    int size = sizeof(GLubyte) * 1024 * 1024 * depth;
    GLubyte* blank_t = new GLubyte[size];
    memset(blank_t, 0x7f, size);

    if (depth == 4)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1024, 1024, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, blank_t);
    else if (depth == 3 && swap == 0)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, blank_t);
    else if (depth == 3 && swap == 1)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_BGR_EXT,
                     GL_UNSIGNED_BYTE, blank_t);

    delete[] blank_t;

    float u_rt, v_rt;
    u_rt = (float) width / 1024;
    v_rt = (float) height / 1024;

    if (depth == 4)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA,
                        GL_UNSIGNED_BYTE, data);
    else if (depth == 3 && swap == 0)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB,
                        GL_UNSIGNED_BYTE, data);
    else if (depth == 3 && swap == 1)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGR_EXT,
                        GL_UNSIGNED_BYTE, data);

    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-(float) width / height, -1, 0);
    glTexCoord2f(u_rt, 0);
    glVertex3f((float) width / height, -1, 0);
    glTexCoord2f(u_rt, v_rt);
    glVertex3f((float) width / height, 1, 0);
    glTexCoord2f(0, v_rt);
    glVertex3f(-(float) width / height, 1, 0);
    glEnd();

    glDeleteTextures(1, texid);
}

/////////Older GL functions

void acGlGetWindowSize(int* width, int* height) {
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(0, 0); //In the middle

    int rasterPos[4];
    glGetIntegerv(GL_CURRENT_RASTER_POSITION, rasterPos);

    glPopMatrix();

    *width = rasterPos[0] * 2;
    *height = rasterPos[1] * 2;
}

float acGlPixelRasterRatioX() {
    glRasterPos2f(0, 0);

    float rasterPos[4];
    glGetFloatv(GL_CURRENT_RASTER_POSITION, rasterPos);

    glRasterPos2f(1, 0);
    float rasterPos2[4];

    glGetFloatv(GL_CURRENT_RASTER_POSITION, rasterPos2);

    return (rasterPos2[0] - rasterPos[0]);
}

float acGlPixelRasterRatioY() {
    glRasterPos2f(0, 0);

    float rasterPos[4];
    glGetFloatv(GL_CURRENT_RASTER_POSITION, rasterPos);

    glRasterPos2f(0, 1);
    float rasterPos2[4];

    glGetFloatv(GL_CURRENT_RASTER_POSITION, rasterPos2);

    return (rasterPos2[1] - rasterPos[1]);
}

float acGlPixel2RasterX(float input) {
    float ratio = acGlPixelRasterRatioX();

    //Get the middle
    glRasterPos2f(0, 0);
    float raster[4];
    glGetFloatv(GL_CURRENT_RASTER_POSITION, raster);

    return (input - raster[0]) / ratio;
}
float acGlRaster2PixelX(float input) {
    float ratio = acGlPixelRasterRatioX();

    glRasterPos2f(0, 0);
    float raster[4];
    glGetFloatv(GL_CURRENT_RASTER_POSITION, raster);

    return raster[0] + input * ratio;
}

float acGlPixel2RasterY(float input) {
    float ratio = acGlPixelRasterRatioY();

    //Get the middle
    glRasterPos2f(0, 0);
    float raster[4];
    glGetFloatv(GL_CURRENT_RASTER_POSITION, raster);

    return (input - raster[1]) / ratio;
}
float acGlRaster2PixelY(float input) {
    float ratio = acGlPixelRasterRatioY();

    glRasterPos2f(0, 0);
    float raster[4];
    glGetFloatv(GL_CURRENT_RASTER_POSITION, raster);

    return raster[1] + input * ratio;
}

void acGlutFontBitmap(char* str) {
    //Note: Bitmap cannot be rotated

    if (str && strlen(str)) {
        while (*str) {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *str);
            str++;
        }
    }
}

void acGlutFontStroke(char* str) {
    //Note: Stroke can be rotated.

    glScalef(0.002, 0.002, 0.002); //Need to scale it to very small
    if (str && strlen(str)) {
        while (*str) {
            glutStrokeCharacter(GLUT_STROKE_ROMAN, *str);
            str++;
        }
    }
}

void acGlPrintState() {
    cout << "GL_BLEND:\t" << (int) glIsEnabled(GL_BLEND) << endl;
    cout << "GL_DEPTH_TEST:\t" << (int) glIsEnabled(GL_DEPTH_TEST) << endl;
    cout << "GL_CULL_FACE:\t" << (int) glIsEnabled(GL_CULL_FACE) << endl;
    cout << "GL_FOG:\t" << (int) glIsEnabled(GL_FOG) << endl;
    cout << "GL_LIGHTING:\t" << (int) glIsEnabled(GL_LIGHTING) << endl;
    cout << "GL_LINE_SMOOTH:\t" << (int) glIsEnabled(GL_LINE_SMOOTH) << endl;
    cout << "GL_NORMALIZE:\t" << (int) glIsEnabled(GL_NORMALIZE) << endl;
    cout << "GL_STENCIL_TEST:\t" << (int) glIsEnabled(GL_STENCIL_TEST) << endl;
    cout << "GL_TEXTURE_1D:\t" << (int) glIsEnabled(GL_TEXTURE_1D) << endl;
    cout << "GL_TEXTURE_2D:\t" << (int) glIsEnabled(GL_TEXTURE_2D) << endl;
    cout << "GL_COLOR_MATERIAL:\t" << (int) glIsEnabled(GL_COLOR_MATERIAL)
         << endl;
}

void acGlPrintGet(GLenum pname) {
    float temp[16];
    int i, j;
    switch (pname) {
    case GL_VIEWPORT:
        glGetFloatv(pname, temp);
        for (i = 0; i < 4; i++) {
            cout << temp[i] << "\t";
        }
        cout << endl;
        break;
    case GL_PROJECTION_MATRIX:
    case GL_MODELVIEW_MATRIX:
        glGetFloatv(pname, temp);
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                cout << temp[i * 4 + j] << "\t";
            }
            cout << endl;
        }
        break;
    case GL_DEPTH_FUNC:
        glGetFloatv(pname, temp);
        printf("GL_DEPTH_FUNC: ");
        switch ((int) temp[0]) {
        case GL_LEQUAL:
            cout << "GL_LEQUAL" << endl;
            break;
        case GL_LESS:
            cout << "GL_LESS" << endl;
            break;
        case GL_EQUAL:
            cout << "GL_EQUAL" << endl;
            break;
        default:
            cout << temp[0] << endl;
            break;
        }
        break;
    case GL_CURRENT_COLOR:
        glGetFloatv(pname, temp);
        printf("GL_CURRENT_COLOR: ");
        for (i = 0; i < 4; i++) {
            cout << temp[i] << "\t";
        }
        cout << endl;
        break;
    }
}

//For picking or selection, the projection and modelview are defined outside the
// function, so that, it is able to be used in custom environment
int acGlSelect(int x, int y, float* projection, float* modelview,
               void (*draw)()) {
    GLuint buffer[512] = { 0 }; //For selection, initialise
    glSelectBuffer(512, buffer);

    //Change the GL_RENDER mode to GL_SELECT, therefore, the projection need to be
    // set to the same as the GL_RENDER
    (void) glRenderMode(GL_SELECT);

    //From NeHe lesson 32
    glInitNames();
    glPushName(0); //This only works in GL_SELECT mode, this statement must exist

    //{ Matrix setting for GL_PROJECTION
    glMatrixMode(GL_PROJECTION);
    glPushMatrix(); //Must push matrix, so that the setting will not affect the display
    glLoadIdentity();

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    gluPickMatrix((GLdouble) x, (GLdouble) (viewport[3] - y), 1, 1, viewport);

    glMultMatrixf(projection);

    //This part is draw the object which can be selected. However, this
    // draw will only draw in memory, not displayed, therefore, the
    // display looping must draw the same object so that the user can see them
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMultMatrixf(modelview);

    //this->DrawTree();
    draw();

    glPopMatrix();

    //This is only want to pop the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    //}end projection

    //Change back to MODELVIEW
    glMatrixMode(GL_MODELVIEW);

    GLint hits;
    hits = glRenderMode(GL_RENDER); //Return the hits of GL_SELECT mode

    //Loop through the buffer using ptr, because the number of names in the stack
    // will cause the different number of names occured
    GLuint nNames, *ptr; //Number of names and ptr
    ptr = buffer;

    int choose = buffer[3]; // This is to store which object is selected
    int depth = buffer[1];

    if (hits > 0) {

        //cout<<"Number of hit: "<<hits<<endl;
        for (int i = 0; i < hits; i++) {
            if (buffer[i * 4 + 1] < GLuint(depth)) { //Select the front object
            //But in gluPerspective(), the z is almost same
                choose = buffer[i * 4 + 3];
                depth = buffer[i * 4 + 1];
            }
            /*nNames=*ptr;
             cout<<"Number of names in name stack: "<<nNames<<endl; ptr++;
             cout<<"z1: "<<(float)*ptr/0x7fffffff<<", "; ptr++;
             cout<<"z2: "<<(float)*ptr/0x7fffffff<<endl; ptr++;
             for(int j=0;j<nNames;j++) {
             cout<<"Name: "<<*ptr<<" "; ptr++;
             }
             cout<<endl; //*/
        }
    }
    return choose;
}

void acGlTextureProject(void* buffer, int width, int height, int depth,
                        int swap) {
    if (depth != 3 && depth != 4)
        printf("Error: byte either 3 or 4\n");

    GLuint texid[1];
    glGenTextures(1, texid);
    glBindTexture(GL_TEXTURE_2D, texid[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    int size = 1024 * 1024 * depth;
    unsigned char* blank_t = (unsigned char*) malloc(size);
    memset(blank_t, 0x7f, size);

    if (depth == 4)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1024, 1024, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, blank_t);
    else if (depth == 3 && swap == 0)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, blank_t);
    else if (depth == 3 && swap == 1)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_BGR_EXT,
                     GL_UNSIGNED_BYTE, blank_t);

    free(blank_t);

    float u_rt = (float) width / 1024;
    float v_rt = (float) height / 1024;

    if (depth == 4)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGBA,
                        GL_UNSIGNED_BYTE, buffer);
    else if (depth == 3 && swap == 0)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RGB,
                        GL_UNSIGNED_BYTE, buffer);
    else if (depth == 3 && swap == 1)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGR_EXT,
                        GL_UNSIGNED_BYTE, buffer);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
    glTexCoord2f(0, 0);
    glVertex3f(-1, -1, 0);
    glTexCoord2f(u_rt, 0);
    glVertex3f(1, -1, 0);
    glTexCoord2f(u_rt, v_rt);
    glVertex3f(1, 1, 0);
    glTexCoord2f(0, v_rt);
    glVertex3f(-1, 1, 0);
    glEnd();

    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glDeleteTextures(1, texid);
}

int acGlProcessHit(GLint hits, GLuint buffer[]) {
    int choose = -1;
    GLuint nNames, *ptr; //Number of names and ptr
    ptr = buffer;

    if (hits > 0) {
        float depth = 10;

        //printf("Number of hit: %d\n",hits);
        for (int i = 0; i < hits; i++) {
            nNames = *ptr++;
            //printf("Number of names in name stack: %d\n",nNames);

            float z1 = (float) *ptr++ / 0x7fffffff;
            float z2 = (float) *ptr++ / 0x7fffffff;

            //printf("z1: %g\nz2: %g\n",z1,z2);

            if (z1 < depth) {
                depth = z1;
                choose = *ptr;
            }

            for (int j = 0; j < nNames; j++) {
                //printf("Name: %d\t",*ptr);
                ptr++;
            }
            //printf("\n");
        }
        //printf("\n");
    }
    return choose;
}

int acGlSelect2(int x, int y, int width, int height, float* projection,
                float* modelview, void (*draw)(),
                int (*processHit)(GLint hit, GLuint buffer[])) {
    GLuint buffer[512] = { 0 }; //For selection, initialise
    glSelectBuffer(512, buffer);

    glDepthRange(0, 0.5);

    //Change the GL_RENDER mode to GL_SELECT, therefore, the projection need to be
    // set to the same as the GL_RENDER
    glRenderMode(GL_SELECT);

    //From NeHe lesson 32
    glInitNames();
    glPushName(0); //This only works in GL_SELECT mode, this statement must exist

    //Matrix setting for GL_PROJECTION
    glMatrixMode(GL_PROJECTION);
    glPushMatrix(); //Must push matrix, so that the setting will not affect the display
    glLoadIdentity();

    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    gluPickMatrix((GLdouble) x, (GLdouble) (viewport[3] - y), width, height,
                  viewport);

    glMultMatrixf(projection);

    //This part is draw the object which can be selected. However, this
    // draw will only draw in memory, not displayed, therefore, the
    // display looping must draw the same object so that the user can see them
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixf(modelview);

    //Callback
    draw();

    glPopMatrix();

    //This is only want to pop the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    //Change back to MODELVIEW
    glMatrixMode(GL_MODELVIEW);

    GLint hits;
    hits = glRenderMode(GL_RENDER); //Return the hits of GL_SELECT mode

    int choose = processHit(hits, buffer);

    glDepthRange(0, 1);
    return choose;
}

int acGlSelectd(int x, int y, int width, int height, double* projection,
                double* modelview, void (*draw)(), GLint viewport[4],
                int inverse, int (*processHit)(GLint hit, GLuint buffer[])) {
    GLuint buffer[512] = { 0 }; //For selection, initialise
    glSelectBuffer(512, buffer);

    glDepthRange(0, 0.5);

    //Change the GL_RENDER mode to GL_SELECT, therefore, the projection need to be
    // set to the same as the GL_RENDER
    glRenderMode(GL_SELECT);

    //From NeHe lesson 32
    glInitNames();
    glPushName(0); //This only works in GL_SELECT mode, this statement must exist

    //Matrix setting for GL_PROJECTION
    glMatrixMode(GL_PROJECTION);
    glPushMatrix(); //Must push matrix, so that the setting will not affect the display
    glLoadIdentity();

    if (!viewport)
        glGetIntegerv(GL_VIEWPORT, viewport);

    if (inverse)
        gluPickMatrix((GLdouble) x, (GLdouble) (y), width, height, viewport);
    else
        gluPickMatrix((GLdouble) x, (GLdouble) (viewport[3] - y), width, height,
                      viewport);

    glMultMatrixd(projection);

    //This part is draw the object which can be selected. However, this
    // draw will only draw in memory, not displayed, therefore, the
    // display looping must draw the same object so that the user can see them
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(modelview);

    //Callback
    draw();

    glPopMatrix();

    //This is only want to pop the projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    //Change back to MODELVIEW
    glMatrixMode(GL_MODELVIEW);

    GLint hits;
    hits = glRenderMode(GL_RENDER); //Return the hits of GL_SELECT mode

    int choose = processHit(hits, buffer);

    glDepthRange(0, 1);
    return choose;
}

/**
 * @brief Check object (point) is occluded or not
 * @return 0 if no occlusion, else 1.
 */
int acGlIsOccluded(float objX, float objY, float objZ) {
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    GLdouble win[3];

    gluProject(objX, objY, objZ, modelview, projection, viewport, &win[0],
               &win[1], &win[2]);

    GLfloat buffer;
    glReadPixels(win[0], win[1], 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &buffer);

    if (buffer < win[2])
        return 1; //Occluded

    return 0;
}
