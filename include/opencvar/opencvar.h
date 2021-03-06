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

/**
 * @file opencvar.h
 * @brief OpenCV AR (Augmented Reality) related functions.
 *
 * @author Allen
 * @date 20100130
 * @version 0.0.2
 */

#pragma once

#ifdef ACDLL
#      define AC_DLL __declspec(dllexport)
#else
#      define AC_DLL
#endif

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;

/**
 * @brief The data structure, based on OpenCV intrinsic camera parameter
 */
struct CvarCamera {
    int width;
    int height;
    double cameraMatrix[9]; /**< Camera matrix */
    double distCoeffs[5]; /**< Distortion coefficients */
    double glProjection[16]; /**< OpenGL projection */
};

/**
 * @brief Data structure to store the 4 orientation template
 */
struct CvarTemplate {
    int width;
    int height;
    double scale;
    long long int code[4];
};

/**
 * @brief Marker
 */
struct CvarMarker {
    double glMatrix[16]; /**< OpenGL modelview matrix */
    int templateId; /**< Possible matched template */
    int markerId; /**< The index of detected marker in image source */
    double score; /**< Match level in range [0,1] (for comparison) */
    CvPoint2D32f square[4]; /**< Storing square vertices */
    double aspectRatio; /**< For the non-square rectangular, width:height ratio */
};

extern "C" {

/**
 * @defgroup functionList cvar function list
 * @{
 */

/**
 * @brief Read camera.yml generated by calibration
 * @param filename    NULL to load default camera data <br>
 <pre>
 350,    0,    160
 0,    350,    120
 0,    0,    1
 </pre>
 * @param pCam    The CvarCamera structure which store the camera matrix, etc
 */
AC_DLL int cvarReadCamera(const char* filename, CvarCamera* pCam);

/**
 * @brief Scale the camera matrix
 */
AC_DLL void cvarCameraScale(CvarCamera* pCam, int width, int height);

/**
 * @brief Convert the camera matrix to projection matrix (row-major).
 *
 * Transpose is needed for OpenGL
 * @param projection [out]    Store the OpenGL projection matrix
 * @param glstyle    OpenGL column-majored matrix, default=0
 */
AC_DLL void cvarCameraProjection(CvarCamera* pCam, double* projection,
                                 int glstyle = 0);

/**
 * @brief Convert to OpenGL matrix
 * @param modelview    [Out] 16 size
 */
AC_DLL void cvarGlMatrix(double* modelview, CvMat* rotate3, CvMat* translate);

/**
 * @brief This is modified version of findquares4()
 *
 * Only the square in darker will return.
 */
AC_DLL CvSeq* cvarFindSquares(IplImage* img, CvMemStorage* storage);

/**
 * @brief Initialise the points of square in 3D
 * @param mat    Must be 4*3 matrix with 64f (double)
 * @param ratio    Ratio of width : height, it can be square or any rectangular
 */
AC_DLL void cvarSquareInit(CvMat* mat, double ratio = 1);

/**
 * @brief Reverse the order of the square
 *
 * Due to the problem of finding of contour, different sequence will be produced.
 * Thus, the source squae might need to be reversed.
 */
AC_DLL void cvarReverseSquare(CvPoint2D32f sq[4]);

/**
 * @brief Find the camera based on the points
 * @param cam    The camera data, in order to get camera matrix and distortion
 * @param objPts    Matrix of object points in 3D, follow the function cvFindExtrinsicCameraParams2
 * @param imgPts    Matrix of image points in 2D, same as above
 * @param modelview    [Out] The model view of OpenGL matrix,
 */
AC_DLL void cvarFindCamera(CvarCamera* cam, CvMat* objPts, CvMat* imgPts,
                           double* modelview);

/************
 Augmented Reality
 ****************/

/**
 * @brief Load template image and convert to ARTag
 *
 * This function will call cvarLoadTag()
 * @param filename    Image file
 */
AC_DLL int cvarLoadTemplateTag(CvarTemplate* tpl, const char* filename,
                               double scale = 0.01);

/**
 * @brief Load the tag information like ARTag
 *
 * @param bit    64-bit hexadecimal value
 */
AC_DLL void cvarLoadTag(CvarTemplate* tpl, long long int bit,
                        int width = 8, int height = 8, double scale = 0.01);

/**
 * @brief Compare the square with the four points of the optical flow
 * @param points [in]    Four points from optical flow
 */
AC_DLL int cvarCompareSquare(IplImage* img, CvPoint2D32f* points);

/**
 * @brief Draw the sqaure based on the sequence from FindSquare().
 */
AC_DLL int cvarDrawSquares(IplImage* img, CvSeq* squares);

/**
 * @brief Get final square
 */
AC_DLL int cvarGetSquare(CvSeq* squares, CvPoint2D32f* points);

/**
 * @brief Create square value, no need release
 * @param src    The array
 * @param ccw    Counter clockwise of the point
 */
AC_DLL void cvarSquare(CvPoint2D32f* src, int width, int height, int ccw = 1);

/**
 * @brief Rotate square
 * @param rot    The rotation. 1 - 0 degree, 2 - 90 degree ccw, 3 - 180 degree, 4 - 270 degree ccw
 */
AC_DLL void cvarRotSquare(CvPoint2D32f* src, int rot);

/**
 * @brief Invert perspective view of the image to the 2D view
 * @param input    Input image, the whole image
 * @param output    Output image.
 * @param src    Source 4 points, correspond to input
 * @param dst    Destination 4 points, correspond to output
 */
AC_DLL void cvarInvertPerspective(IplImage* input, IplImage* output,
                                  CvPoint2D32f* src, CvPoint2D32f* dst);

/**
 * @brief From the points of the square to OpenGL model view matrix
 * @param points    Points of the square
 * @param cam    Camera
 * @param modelview [out]    Model view matrix
 * @param ratio    Ratio of width:height, 1 = square
 */
AC_DLL void cvarSquareToMatrix(CvPoint2D32f* points, CvarCamera* cam,
                               double* modelview, double ratio = 1);

/**
 * @brief Convert square points to rectangle for cropping
 * @return The rectangle
 */
CvRect cvarSquare2Rect(CvPoint2D32f pt[4]);

/**
 * @brief Get all square from cvarFindSquare()
 *
 * Store all the square points in pts
 * @param squares    Can get the squares from cvarFindSquare()
 * @param pts    A vector of points to store the output
 * @return Number of squares
 */
int cvarGetAllSquares(CvSeq* squares, vector<CvPoint2D32f>* pts);

/**
 * @brief Tracking, check the relationship between two square points
 *
 * After the tracking, if they are related, the pt1 will be updated with pt2
 * @param pt1 The pevious points
 * @param pt2 The current points
 * @return If they are related, return 1, else 0.
 */
int cvarTrack(CvPoint2D32f pt1[4], CvPoint2D32f pt2[4]);

/**
 * @brief Multiple marker registration
 * @param image Incoming image
 * @param markers    Vector of marker, which will be stored with the marker information
 * @param templates    Vector of template for template matching
 * @param camera    Camera data
 */
int cvarArMultRegistration(IplImage* image, vector<CvarMarker>* markers,
                           vector<CvarTemplate> templates, CvarCamera* camera);

} // extern

/** @} end functionList */
