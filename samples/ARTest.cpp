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

#include <iostream>
#include <vector>
#include <GL/glut.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencvar/opencvar.h"
#include "opencvar/acmath.h"
#include "opencvar/acgl.h"

CvarCamera camera;
std::vector<CvarTemplate> templates;
CvCapture* capture;
std::vector<CvarMarker> markers;
bool enabledAR = true;

void display() {
    IplImage* frame = cvQueryFrame(capture);
    cvFlip(frame, frame);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glPushMatrix();
    glFrontFace(GL_CCW);
    acGlTextureProject((unsigned char*) frame->imageData, frame->width,
                       frame->height, frame->nChannels, 1);

    cvarArMultRegistration(frame, &markers, templates, &camera);

    glClear(GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    if (enabledAR) {
        for (auto& marker : markers) {
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);

            glLoadMatrixd(marker.glMatrix);
            glRotatef(90, 1, 0, 0);
            glTranslatef(0, 0.5, 0);

            if (marker.score > 0) {
                glFrontFace(GL_CW);
                glutSolidTeapot(1);
            } else {
                glFrontFace(GL_CCW);
                glutSolidCube(1);
            }

            glDisable(GL_LIGHTING);
        }
    }

    glPopMatrix();
    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double projection[16];
    cvarCameraProjection(&camera, projection);
    acMatrixTranspose(projection);
    glLoadMatrixd(projection);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void idle() {
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27:
        cvReleaseCapture(&capture);
        exit(0);
        break;
    case 'a':
        enabledAR = !enabledAR;
        break;
    }
}

int main(int argc, char** argv) {
    if (argc < 3)
        return -1;

    capture = cvCreateCameraCapture(atoi(argv[1]));
    if (!capture) {
        std::cerr << "Create camera capture failed" << std::endl;
        return -1;
    }

    CvarTemplate template_;
    cvarLoadTemplateTag(&template_, argv[2]);
    templates.push_back(template_);

    if (argc < 4) {
        cvarReadCamera(argv[3], &camera);
    } else {
        IplImage* frame = cvQueryFrame(capture);
        cvarReadCamera(NULL, &camera);
        cvarCameraScale(&camera, frame->width, frame->height);
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(640, 480);
    glutCreateWindow("AR");
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);

    glEnable(GL_DEPTH_TEST);
    glutMainLoop();
    cvReleaseCapture(&capture);

    return 0;
}
