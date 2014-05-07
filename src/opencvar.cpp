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

#include <stdio.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencvar/opencvar.h"
#include "opencvar/acmath.h"

using namespace std;

int AC_CV_DEBUG = 0;

static int g_thresh = -1; // For storing auto threshold value
static int g_needRandom = 1; // For checking need randomise the threshold or not
static int g_isRandom = 0; // Check randomise performed or not

int cvarReadCamera(const char* filename, CvarCamera* pCam) {
    // Use default value
    if (!filename) {
        pCam->width = 320;
        pCam->height = 240;
        pCam->flags = 0;

        pCam->avg_error = 0;
        double mat[] = { 350, 0, 160, 0, 350, 120, 0, 0, 1 };
        memcpy(pCam->matrix, mat, sizeof(double) * 9);

        double distort[] = { 0, 0, 0, 0 };
        memcpy(pCam->distortion, distort, sizeof(double) * 4);
    } else {
        CvFileStorage* file = cvOpenFileStorage(filename, 0, CV_STORAGE_READ);
        if (!file)
            return 0;

        pCam->width = cvReadIntByName(file, 0, "image_width");
        pCam->height = cvReadIntByName(file, 0, "image_height");
        pCam->flags = cvReadIntByName(file, 0, "flags");

        pCam->avg_error = cvReadRealByName(file, 0, "avg_reprojection_error");

        CvMat* camera = (CvMat*) cvRead(
                file, cvGetFileNodeByName(file, 0, "camera_matrix"));
        // Copy camera matrix
        memcpy(pCam->matrix, camera->data.db, sizeof(double) * 9);

        CvMat* distortion = (CvMat*) cvRead(
                file, cvGetFileNodeByName(file, 0, "distortion_coefficients"));
        memcpy(pCam->distortion, distortion->data.db, sizeof(double) * 4);

        cvReleaseFileStorage(&file);
    }

    // Create OpenGL projection
    cvarCameraProjection(pCam, pCam->projection);
    acMatrixTransposed(pCam->projection);

    return 1;
}

void cvarCameraScale(CvarCamera* pCam, int width, int height) {
    // This scaling algorithm refer to ARToolKit arParamChangeSize()

    // Get ratio
    float rt_u = (float) width / pCam->width;
    float rt_v = (float) height / pCam->height;

    // ARToolKit only uses one ratio. But I used two ratio, so that, the data of the matrix
    // need to be scaled separately
    // fx,fy (focal length)
    pCam->matrix[0] *= rt_u;
    pCam->matrix[1 * 3 + 1] *= rt_v;

    // cx,cy (principal point)
    pCam->matrix[2] *= rt_u;
    pCam->matrix[1 * 3 + 2] *= rt_v;

    pCam->width = width;
    pCam->height = height;

    // Recalculate OpenGL projection
    cvarCameraProjection(pCam, pCam->projection);
    acMatrixTransposed(pCam->projection);
}

void cvarCameraProjection(CvarCamera* pCam, double* projection, int glstyle) {
    // The projection should be 4x4 matrix

    // Set the near plane and far plane, based on ARToolKit
    // No more based on ARToolKit
    float nearplane = 0.1f;
    float farplane = 5000.0f;

    // Initialise with 0
    memset(projection, 0, sizeof(double) * 16);

    projection[0] = 2. * pCam->matrix[0] / pCam->width;
    projection[1 * 4 + 1] = 2. * pCam->matrix[1 * 3 + 1] / pCam->height;
    projection[0 * 4 + 2] = 2. * (pCam->matrix[2] / pCam->width) - 1.;
    projection[1 * 4 + 2] = 2. * (pCam->matrix[1 * 3 + 2] / pCam->height) - 1.;
    projection[2 * 4 + 2] = -(farplane + nearplane) / (farplane - nearplane);
    projection[2 * 4 + 3] = -2. * farplane * nearplane / (farplane - nearplane);
    projection[3 * 4 + 2] = -1;

    if (glstyle)
        acMatrixTransposed(projection);
}

/**
 * Convert to OpenGL Matrix
 * @param modelview    16 size
 */
void cvarGlMatrix(double* modelview, CvMat* rotate3, CvMat* translate) {
    memset(modelview, 0, 16 * sizeof(double));
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3; i++) {
            modelview[i * 4 + j] = rotate3->data.db[j * 3 + i];
        }
    }

    double qua[4];
    acMatrixToQuaternion(modelview, qua);
    qua[1] = -qua[1];
    qua[2] = -qua[2];

    acQuaternionToMatrix(qua, modelview);

    modelview[12] = translate->data.db[0];
    modelview[13] = translate->data.db[1];
    modelview[14] = -translate->data.db[2];
    modelview[15] = 1;
}

// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
CvSeq* cvarFindSquares(IplImage* img, CvMemStorage* storage, int threshold,
                       int inner) {
    CvSeq* contours;
    int i, c, l, N = 11;
    CvSize sz = cvSize(img->width & -2, img->height & -2);
    IplImage* timg = cvCloneImage(img); // make a copy of input image
    IplImage* gray = cvCreateImage(sz, 8, 1);
    IplImage* pyr = cvCreateImage(cvSize(sz.width / 2, sz.height / 2), 8, 3);
    IplImage* tgray;
    CvSeq* result;
    double s, t;

    // create empty sequence that will contain points -
    // 4 points per square (the square's vertices)
    CvSeq* squares = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvPoint), storage);

    // select the maximum ROI in the image
    // with the width and height divisible by 2
    cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));

    // down-scale and upscale the image to filter out the noise
    cvPyrDown(timg, pyr, 7);
    cvPyrUp(pyr, timg, 7);
    tgray = cvCreateImage(sz, 8, 1);

    // find squares in every color plane of the image
    for (c = 0; c < 1; c++) {
        // extract the c-th color plane
        // Convert the image to greyscale
        cvCvtColor(timg, tgray, CV_BGR2GRAY);

        // try several threshold levels
        // Modified: using only one threshold value
        for (l = 0; l < 1; l++) {
            cvThreshold(tgray, gray, threshold, 255, CV_THRESH_BINARY);

            if (AC_CV_DEBUG) {
                gray->origin = 1;
                cvNamedWindow("find_d", CV_WINDOW_AUTOSIZE);
                cvShowImage("find_d", gray);
            }

            // find contours and store them all as a list
            cvFindContours(gray, storage, &contours, sizeof(CvContour),
                           CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

            // test each contour
            while (contours) {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                result = cvApproxPoly(contours, sizeof(CvContour), storage,
                                      CV_POLY_APPROX_DP,
                                      cvContourPerimeter(contours) * 0.02, 0);

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                // Modified: only positive contour will be returned

                // Check contour
                int chkContour;
                switch (inner) {
                case 0:
                    chkContour = cvContourArea(result, CV_WHOLE_SEQ) > 500;
                    break;
                case 1:
                    chkContour = cvContourArea(result, CV_WHOLE_SEQ) < -500;
                    break;
                case 2:
                    chkContour = fabs(cvContourArea(result, CV_WHOLE_SEQ)) > 500;
                    break;
                }

                if (result->total == 4 && chkContour
                    && cvCheckContourConvexity(result)) {
                    // Only if the square is smaller than the image
                    // Modified: No need calculation of the angle
                    if (((CvPoint*) cvGetSeqElem(result, 0))->x > 2
                        && ((CvPoint*) cvGetSeqElem(result, 0))->x
                           < img->width - 2
                        && ((CvPoint*) cvGetSeqElem(result, 0))->y > 2
                        && ((CvPoint*) cvGetSeqElem(result, 0))->y
                           < img->height - 2) {
                        for (i = 0; i < 4; i++)
                            cvSeqPush(squares,
                                      (CvPoint*) cvGetSeqElem(result, i));
                    }
                }

                // take the next contour
                contours = contours->h_next;
            }
        }
    }

    // release all the temporary images
    cvReleaseImage(&gray);
    cvReleaseImage(&pyr);
    cvReleaseImage(&tgray);
    cvReleaseImage(&timg);

    return squares;
}

/**
 * Initialise the points of square in 3D
 * @param mat    Must be 4*3 matrix with 64f (double)
 */
void cvarSquareInit(CvMat* mat, float ratio) {
    mat->data.db[0] = -ratio;
    mat->data.db[1] = -1;
    mat->data.db[2] = 0;

    mat->data.db[3] = ratio;
    mat->data.db[4] = -1;
    mat->data.db[5] = 0;

    mat->data.db[6] = ratio;
    mat->data.db[7] = 1;
    mat->data.db[8] = 0;

    mat->data.db[9] = -ratio;
    mat->data.db[10] = 1;
    mat->data.db[11] = 0;
}

void cvarReverseSquare(CvPoint2D32f sq[4]) {
    CvPoint2D32f temp;
    temp = sq[1];
    sq[1] = sq[3];
    sq[3] = temp;
}

/**
 * Find the camera based on the points
 * @param cam    The camera data, in order to get camera matrix and distortion
 * @param objPts    Matrix of object points in 3D, follow the function cvFindExtrinsicCameraParams2
 * @param imgPts    Matrix of image points in 2D, same as above
 * @param modelview    [Out] The model view of OpenGL matrix,
 */
void cvarFindCamera(CvarCamera* cam, CvMat* objPts, CvMat* imgPts,
                    double* modelview) {
    CvMat camera = cvMat(3, 3, CV_64F, cam->matrix);
    CvMat dist = cvMat(1, 4, CV_64F, cam->distortion);

    CvMat* rotate = cvCreateMat(1, 3, CV_64F);
    CvMat* rotate3 = cvCreateMat(3, 3, CV_64F);
    CvMat* translate = cvCreateMat(1, 3, CV_64F);

    cvFindExtrinsicCameraParams2(objPts, imgPts, &camera, &dist, rotate,
                                 translate);
    cvRodrigues2(rotate, rotate3);

    cvarGlMatrix(modelview, rotate3, translate);

    cvReleaseMat(&rotate);
    cvReleaseMat(&rotate3);
    cvReleaseMat(&translate);
}

/*******************
 Augmented Reality
 ****************/

int cvarLoadTemplateTag(CvarTemplate* tpl, const char* filename) {
    IplImage* file = cvLoadImage(filename, CV_LOAD_IMAGE_GRAYSCALE);
    if (!file) {
        return 0;
    }

    // Crop image
    cvSetImageROI(file, cvRect(1, 1, file->width - 2, file->height - 2));

    // Binarise
    IplImage* fileg = cvCreateImage(cvGetSize(file), 8, 1);
    cvCopy(file, fileg);
    cvThreshold(fileg, fileg, 100, 1, CV_THRESH_BINARY);
    cvFlip(fileg, fileg);

    long long int bit;
    acArray2DToBit((unsigned char*) fileg->imageData, fileg->width,
                   fileg->height, &bit);

    cvarLoadTag(tpl, bit, file->width - 2, file->height - 2);

    cvReleaseImage(&fileg);
    cvResetImageROI(file);

    return 1;
}

void cvarLoadTag(CvarTemplate* tpl, long long int bit,
                 int width, int height) {
    tpl->width = width;
    tpl->height = height;

    for (int i = 0; i < 4; i++) {
        tpl->code[i] = bit;
        acBitRotate(&tpl->code[i], i, width, height);
    }
}

/**
 * Compare the square with the four points of the optical flow
 * @param points [in]    Four points from optical flow
 */
int cvarCompareSquare(IplImage* img, CvPoint2D32f* points) {
    CvMemStorage* storage = cvCreateMemStorage();

    // Find the square
    CvSeqReader reader;
    CvSeq* square = cvarFindSquares(img, storage);

    cvStartReadSeq(square, &reader, 0);

    int match = 0;

    for (int i = 0; i < square->total; i += 4) {
        CvPoint pt[4], *rect = pt;
        int count = 4;

        for (int j = 0; j < 4; j++)
            CV_READ_SEQ_ELEM(pt[j], reader);

        // Compare the points
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                AcPointf tPoint;
                tPoint.x = pt[k].x;
                tPoint.y = pt[k].y;

                AcPointf tPoint2;
                tPoint2.x = points[j].x;
                tPoint2.y = points[j].y;

                double dist = acCalcLength(tPoint2, tPoint);

                if (dist < 10) {
                    match++;
                }
            }
        }
    }

    cvReleaseMemStorage(&storage);
    return match;
}

int cvarDrawSquares(IplImage* img, CvSeq* squares, CvPoint2D32f* points,
                    int draw) {
    CvSeqReader reader;
    IplImage* cpy = cvCloneImage(img);
    int i;

    // result
    int res = 0;

    // initialize reader of the sequence
    cvStartReadSeq(squares, &reader, 0);

    // read 4 sequence elements at a time (all vertices of a square)
    for (i = 0; i < squares->total; i += 4) {
        CvPoint pt[4], *rect = pt;
        int count = 4;

        // read 4 vertices
        CV_READ_SEQ_ELEM(pt[0], reader);
        CV_READ_SEQ_ELEM(pt[1], reader);
        CV_READ_SEQ_ELEM(pt[2], reader);
        CV_READ_SEQ_ELEM(pt[3], reader);

        // draw the square as a closed polyline 
        cvPolyLine(cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 1, CV_AA, 0);

        // Copy the points
        for (int j = 0; j < 4; j++) {
            points[j].x = pt[j].x;
            points[j].y = pt[j].y;
        }

        res++;
    }

    if (draw) {
        cvCopy(cpy, img);
    }
    cvReleaseImage(&cpy);

    return res;
}

int cvarGetSquare(IplImage* img, CvSeq* squares, CvPoint2D32f* points) {
    CvSeqReader reader;
    IplImage* cpy = cvCloneImage(img);
    int i;

    // result
    int res = 0;

    // initialize reader of the sequence
    cvStartReadSeq(squares, &reader, 0);

    // read 4 sequence elements at a time (all vertices of a square)
    for (i = 0; i < squares->total; i += 4) {
        CvPoint pt[4], *rect = pt;
        int count = 4;

        // read 4 vertices
        CV_READ_SEQ_ELEM(pt[0], reader);
        CV_READ_SEQ_ELEM(pt[1], reader);
        CV_READ_SEQ_ELEM(pt[2], reader);
        CV_READ_SEQ_ELEM(pt[3], reader);

        // draw the square as a closed polyline 
        cvPolyLine(cpy, &rect, &count, 1, 1, CV_RGB(0,255,0), 1, CV_AA, 0);

        // Copy the points
        for (int j = 0; j < 4; j++) {
            points[j].x = pt[j].x;
            points[j].y = pt[j].y;
        }

        res++;
    }

    if (AC_CV_DEBUG) {
        cvNamedWindow("square_d", CV_WINDOW_AUTOSIZE);
        cvShowImage("square_d", cpy);
    }
    cvReleaseImage(&cpy);

    return res;
}

/**
 * Create square value, no need release
 * @param src    The array
 * @param ccw    Counter clockwise of the point
 */
void cvarSquare(CvPoint2D32f* src, int width, int height, int ccw) {
    // Reverse order is correct way
    if (ccw) {
        src[0].x = 0;
        src[0].y = 0;
        src[1].x = 0;
        src[1].y = height - 1;
        src[2].x = width - 1;
        src[2].y = height - 1;
        src[3].x = width - 1;
        src[3].y = 0;
    } else {
        src[0].x = 0;
        src[0].y = 0;
        src[1].x = width - 1;
        src[1].y = 0;
        src[2].x = width - 1;
        src[2].y = height - 1;
        src[3].x = 0;
        src[3].y = height - 1;
    }
}

/**
 * Rotate square
 * @param rot    The rotation. 1 - 0 degree, 2 - 90 degree ccw, 3 - 180 degree, 4 - 270 degree ccw
 */
void cvarRotSquare(CvPoint2D32f* src, int rot) {
    CvPoint2D32f temp[4];
    for (int i = 0; i < 4; i++) {
        temp[i] = src[i];
    }

    // The fomula come from:
    /* switch(rot) {
     case 1:
     src[0] = temp[0];
     src[1] = temp[1];
     src[2] = temp[2];
     src[3] = temp[3];
     case 2:
     src[1] = temp[0];
     src[2] = temp[1];
     src[3] = temp[2];
     src[0] = temp[3];
     break;
     case 3:
     src[2] = temp[0];
     src[3] = temp[1];
     src[0] = temp[2];
     src[1] = temp[3];
     break;
     case 4:
     src[3] = temp[0];
     src[0] = temp[1];
     src[1] = temp[2];
     src[2] = temp[3];
     break;
     default:
     break;
     }*/
    for (int i = 0; i < 4; i++) {
        src[(rot - 1 + i) % 4] = temp[i];
    }
}

/**
 * Invert perspective view of the image to the 2D view
 * @param input    Input image, the whole image
 * @param output    Output image.
 * @param src    Source 4 points, correspond to input
 * @param dst    Destination 4 points, correspond to output
 */
void cvarInvertPerspective(IplImage* input, IplImage* output, CvPoint2D32f* src,
                           CvPoint2D32f* dst) {
    CvMat* mapMatrix = cvCreateMat(3, 3, CV_32FC1);
    cvGetPerspectiveTransform(src, dst, mapMatrix);
    cvWarpPerspective(input, output, mapMatrix);
    cvReleaseMat(&mapMatrix);
}

/**
 * From the points of the square to OpenGL model view matrix
 * @param points    Points of the square
 * @param cam    Camera
 * @param modelview [out]    Model view matrix
 */
void cvarSquareToMatrix(CvPoint2D32f* points, CvarCamera* cam,
                        double* modelview, float ratio) {
    // Calculate matrix
    CvMat* imgPoint = cvCreateMat(4, 2, CV_64F);
    CvMat* objPoint = cvCreateMat(4, 3, CV_64F);
    cvarSquareInit(objPoint, ratio);

    for (int i = 0; i < 4; i++) {
        imgPoint->data.db[i * 2] = points[i].x;
        imgPoint->data.db[i * 2 + 1] = points[i].y;
    }

    cvarFindCamera(cam, objPoint, imgPoint, modelview);

    cvReleaseMat(&imgPoint);
    cvReleaseMat(&objPoint);
}

/*************
 Debug
 ************/
void cvarEnableDebug() {
    AC_CV_DEBUG = 1;
}

/*************
 Multiple marker registration
 ***********/

CvRect cvarSquare2Rect(CvPoint2D32f pt[4]) {
    int x = 50000, y = 50000, w, h;
    int x2 = -50000, y2 = -50000;
    for (int i = 0; i < 4; i++) {
        if (pt[i].x < x)
            x = pt[i].x;
        if (pt[i].x > x2)
            x2 = pt[i].x;
        if (pt[i].y < y)
            y = pt[i].y;
        if (pt[i].y > y2)
            y2 = pt[i].y;
    }
    w = x2 - x;
    h = y2 - y;
    return cvRect(x, y, w, h);
}

int cvarGetAllSquares(CvSeq* squares, vector<CvPoint2D32f>* pts) {
    CvSeqReader reader;

    int res = 0;

    cvStartReadSeq(squares, &reader, 0);

    for (int i = 0; i < squares->total; i += 4) {
        CvPoint pt[4];

        CV_READ_SEQ_ELEM(pt[0], reader);
        CV_READ_SEQ_ELEM(pt[1], reader);
        CV_READ_SEQ_ELEM(pt[2], reader);
        CV_READ_SEQ_ELEM(pt[3], reader);

        for (int j = 0; j < 4; j++) {
            CvPoint2D32f pt32;
            pt32.x = pt[j].x;
            pt32.y = pt[j].y;
            pts->push_back(pt32);
        }

        res++;
    }

    return res;
}

int cvarTrack(CvPoint2D32f pt1[4], CvPoint2D32f pt2[4]) {
    // Match the point direction
    for (int j = 0; j < 4; j++) { //4 direction
        int res = 0;
        for (int i = 0; i < 4; i++) {
            AcPointf tPoint1;
            tPoint1.x = pt1[i].x;
            tPoint1.y = pt1[i].y;

            AcPointf tPoint2;
            tPoint2.x = pt2[(i + j) % 4].x;
            tPoint2.y = pt2[(i + j) % 4].y;

            if (acCalcLength(tPoint1, tPoint2) < 20) {
                res++;
            }
        }
        if (res == 4) {
            for (int i = 0; i < 4; i++) {
                pt1[i] = pt2[(i + j) % 4];
            }
            return 1;
        }
    }
    return 0;
}

int cvarArMultRegistration(IplImage* img, vector<CvarMarker>* vMarker,
                           vector<CvarTemplate> vTpl, CvarCamera* cam,
                           int thresh, double matchThresh) {
    // Auto thresholding
    int autothresh = 0;
    if (thresh == AC_THRESH_AUTO) {
        autothresh = 1;
        if (g_needRandom) {
            if (g_isRandom == 0) {
                srand(time(NULL));
                g_isRandom = 1;
            }
            g_thresh = rand() % 256;
        }
        thresh = g_thresh;
    }

    CvMemStorage* patStorage = cvCreateMemStorage();

    // Greyscaling
    IplImage* grey = cvCreateImage(cvSize(img->width, img->height), 8, 1);
    cvCvtColor(img, grey, CV_BGR2GRAY);
    // cvThreshold(grey,grey,100,255,CV_THRESH_BINARY);
    cvCvtColor(grey, img, CV_GRAY2BGR);
    cvReleaseImage(&grey);

    // Find all squares
    CvMemStorage* squareStorage = cvCreateMemStorage();
    vector<CvPoint2D32f> vPts;
    int nSquare = cvarGetAllSquares(
            cvarFindSquares(img, squareStorage, thresh, 0), &vPts);

    // Checking for previous marker square
    vector<int> reserve; // For reserving the previous data

    for (int i = 0; i < vMarker->size(); i++) {
        for (int j = 0; j < vPts.size(); j += 4) {
            // Points to array
            CvPoint2D32f arrPoint[4];
            for (int k = 0; k < 4; k++) {
                arrPoint[k] = vPts[j + k];
            }

            // If current is related to previous,
            // that means the previous one can be use, and the current one can be eliminated
            if (cvarTrack((*vMarker)[i].square, arrPoint)) {
                reserve.push_back(i);

                // Directly remove it from the vector, because removing is difficult
                vPts.erase(vPts.begin() + j, vPts.begin() + j + 4); // Remove the 4 vertices

                // Recalculate the modelview
                cvarSquareToMatrix((*vMarker)[i].square, cam,
                                   (*vMarker)[i].modelview,
                                   (*vMarker)[i].ratio);
            }
        }
    }

    // Make a copy of previous data
    vector<CvarMarker> cpy = *vMarker;
    vMarker->clear();

    // Store only the updated
    for (int i = 0; i < reserve.size(); i++) {
        vMarker->push_back(cpy[reserve[i]]);
    }

    // For template matching part
    CvPoint2D32f points[4]; // For calculation

    vector<CvarMarker> vMarker2; // For calculation

    // For each square, check the pattern
    for (int i = 0; i < vPts.size() / 4; i++) {

        // Get the subimage from the square
        // Vector to array
        for (int j = 0; j < 4; j++) {
            points[j] = vPts[i * 4 + j];
        }
        CvRect rect = cvarSquare2Rect(points);
        rect.x -= 5;
        rect.y -= 5;
        rect.width += 10;
        rect.height += 10;

        cvSetImageROI(img, rect);
        IplImage* crop = cvCreateImage(cvGetSize(img), img->depth,
                                       img->nChannels);
        crop->origin = 1;
        cvCopy(img, crop);
        cvResetImageROI(img);

        // Get pattern from within the square
        int pattern = 0;
        CvPoint2D32f patPoint[4] = { 0 };

        // For every template
        for (int j = 0; j < vTpl.size(); j++) {

            pattern = cvarGetSquare(
                    crop, cvarFindSquares(crop, patStorage, thresh, 0), patPoint);

            if (pattern) {
                // Create pattern image
                IplImage* patImage = cvCreateImage(
                        cvSize(vTpl[j].width + 2, vTpl[j].height + 2), 8, 3);
                patImage->origin = 1;

                CvarMarker marker = { 0 }; // Important to initialise especially "match"
                marker.id = i;

                CvPoint2D32f patPointSrc[4];
                cvarSquare(patPointSrc, vTpl[j].width + 2, vTpl[j].height + 2, 0);
                cvarInvertPerspective(crop, patImage, patPoint, patPointSrc);

                // Crop
                CvRect croptag = cvRect(1, 1, vTpl[j].width, vTpl[j].height);
                cvSetImageROI(patImage, croptag);

                // Binarise
                IplImage* patImageg = cvCreateImage(cvGetSize(patImage), 8, 1);
                cvCvtColor(patImage, patImageg, CV_BGR2GRAY);
                cvThreshold(patImageg, patImageg, thresh, 1, CV_THRESH_BINARY);

                // Image to bit
                long long int bit = 0;
                acArray2DToBit((unsigned char*) patImageg->imageData,
                               patImageg->width, patImageg->height, &bit);

                // Get orientation of the bit
                int orient = 0;
                for (int k = 0; k < 4; k++) {
                    if (bit == vTpl[j].code[k]) {
                        orient = k + 1;
                        break;
                    }
                }

                // Match
                if (orient)
                    marker.match = 1; // So that it is the best
                else
                    marker.match = 0;

                // Release
                cvReleaseImage(&patImageg);
                cvResetImageROI(patImage);

                // Record the current template so that later will be pushed into vector
                marker.tpl = j;

                switch (orient) {
                case 4:
                    cvarRotSquare(points, 2);
                    break;
                case 2:
                    cvarRotSquare(points, 4);
                    break;
                }

                // If matched and get orientation, now compare the matched value
                // Matrix calculation
                memcpy(marker.square, points, 4 * sizeof(CvPoint2D32f));

                // Add in ratio
                marker.ratio = (float) vTpl[j].width / vTpl[j].height;

                // Add the marker info
                vMarker2.push_back(marker);

                cvReleaseImage(&patImage);
            }
        }

        cvReleaseImage(&crop);
    }

    // Process detected marker
    // double matchVal=matchThresh;
    for (int i = 0; i < vMarker2.size(); i++) {
        // Compare with the other
        for (int j = 0; j < i; j++) {
            // If same detected marker, and same template, only one will survive
            if (vMarker2[i].id == vMarker2[j].id
                || vMarker2[i].tpl == vMarker2[j].tpl) {
                if (vMarker2[i].match > vMarker2[j].match)
                    vMarker2[j].id = -1;
                else
                    vMarker2[i].id = -1;
            }
        }
    }

    // To output
    for (int i = 0; i < vMarker2.size(); i++) {
        if (vMarker2[i].tpl >= 0 && vMarker2[i].id >= 0) {
            cvarSquareToMatrix(vMarker2[i].square, cam, vMarker2[i].modelview,
                               vMarker2[i].ratio);
            vMarker->push_back(vMarker2[i]);
        }
    }

    cvReleaseMemStorage(&patStorage);
    cvReleaseMemStorage(&squareStorage);

    // Auto threshold
    if (autothresh) {
        if (vMarker->size()) {
            g_needRandom = 0;
        } else {
            g_needRandom = 1;
        }
    }

    return vMarker->size();
}
