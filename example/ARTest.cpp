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
#include <fstream>
#include <vector>
#include <GL/glut.h>
#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencvar/opencvar.h"
#include "opencvar/acmath.h"
#include "opencvar/acgl.h"

using namespace std;

CvarCamera camera;
vector<CvarTemplate> templates;
CvCapture* capture;
vector<CvarMarker> markers;

void display() {
    IplImage* frame = cvQueryFrame(capture);
    cvFlip(frame, frame);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glColor3f(1, 1, 1);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    glPushMatrix();
    acGlTextureProject((unsigned char*) frame->imageData, frame->width,
                       frame->height, frame->nChannels, 1);
    glClear(GL_DEPTH_BUFFER_BIT);

    int ar_detect = cvarArMultRegistration(frame, &markers, templates, &camera,
                                           AC_THRESH_AUTO, 0.8);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    for (int i = 0; i < ar_detect; i++) {
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        if (markers[i].tpl == 0) {

            glLoadMatrixd(markers[i].modelview);
            glRotatef(90, 1, 0, 0);
            glTranslatef(0, 0.5, 0);
            glutSolidTeapot(1);
        }

        glDisable(GL_LIGHTING);
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
    acMatrixTransposed(projection);
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
    }
}

int main(int argc, char** argv) {
    if (argc < 3)
        return -1;

    cvarEnableDebug();
    capture = cvCreateCameraCapture(atoi(argv[1]));
    if (!capture) {
        fprintf(stderr, "Create camera capture failed\n");
        return 1;
    }

    CvarTemplate tpl;
    cvarLoadTemplateTag(&tpl, argv[2]);
    templates.push_back(tpl);

    IplImage* frame = cvQueryFrame(capture);
    cvarReadCamera(NULL, &camera);
    cvarCameraScale(&camera, frame->width, frame->height);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(640, 480);
    glutCreateWindow("ARTest");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutIdleFunc(idle);

    glEnable(GL_DEPTH_TEST);
    glutMainLoop();
    cvReleaseCapture(&capture);

    return 0;
}
