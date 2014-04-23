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
 * \file opencvar.h
 * \brief OpenCV AR (Augmented Reality) related functions.
 *
 * \author Allen
 * \date 20100130
 * \version 0.0.2
 */

#ifndef AC_CV_H
#define AC_CV_H

#ifdef ACDLL
#	define AC_DLL __declspec(dllexport)
#else
#	define AC_DLL
#endif

#ifndef _VECTOR_
# include <vector>
#endif

//OpenCV Header
#ifndef _CV_H
# include <opencv/cv.h>
#endif

#ifndef _HIGH_GUI_
# include <opencv/highgui.h>
#endif


using namespace std;

#define AC_THRESH_AUTO 1000

/**
 * \brief The data structure, based on OpenCV intrinsic camera parameter
 */
struct CvarCamera {
	int width;
	int height;
	int flags;
	double avg_error; /**< Average reprojection error */
	
	double matrix[9]; /**< Camera matrix */
	double distortion[4]; /**< Distortion coefficients */
	double projection[16]; /**< OpenGL projection */
};


/**
 * \brief Data structure to store the 4 orientation template
 */
struct CvarTemplate {
	IplImage *image[4];
	int type; /**< 0 = ARToolKit (the pattern only), 1 = Whole image (with border), 2= ARTag */
	long long int code[4]; //For ARTag
};


/**
 * \brief Marker
 */
struct CvarMarker {
	double modelview[16];	/**< OpenGL modelview matrix */
	int tpl;	/**< Possible matched template */
	int id;	/**< The index of detected marker in image source */
	double match;	/**< Match level in range [0,1] (for comparison) */
	vector<CvPoint2D32f> pt;	/**< The points of the marker */
	CvPoint2D32f square[4];	/**< Storing square vertices */
	
	float ratio;	/**< For the non-square rectangular, width:height ratio */
};


/**
 * \brief SURF
 */
struct CvarSurf {
	IplImage *object;
	CvSeq *objectKeypoints;
	CvSeq *objectDescriptors;	
};


extern "C" {

/**
 * \defgroup functionList cvar function list
 * @{
 */

/**
 * \brief Read camera.yml generated by calibration
 * \param filename	NULL to load default camera data <br>
<pre> 
350,	0,	160
0,	350,	120
0,	0,	1
</pre>
 * \param pCam	The CvarCamera structure which store the camera matrix, etc
 */
AC_DLL int cvarReadCamera(const char* filename,CvarCamera* pCam);

/**
 * \brief Scale the camera matrix
 */
AC_DLL void cvarCameraScale(CvarCamera* pCam,int width,int height);

/**
 * \brief Convert the camera matrix to projection matrix (row-major).
 *
 * Transpose is needed for OpenGL
 * \param projection [out]	Store the OpenGL projection matrix
 * \param glstyle	OpenGL column-majored matrix, default=0
 */
AC_DLL void cvarCameraProjection(CvarCamera* pCam,double* projection,int glstyle=0);


/**
 * \brief
 *
 * OpenCV dummy function. Because of cvLoad() for Haar detection might cause problem,
 * this function can be used.
 */
AC_DLL void cvarDummy();

/**
 * \brief Convert to OpenGL matrix
 * \param modelview	[Out] 16 size
 */
AC_DLL void cvarGlMatrix(double* modelview,CvMat* rotate3,CvMat* translate);

/**
 * \brief This is modified version of findquares4()
 *
 * Only the square in darker will return.
 * \param inner	0 - inner is black, 1 - white, 2 - both
 */
AC_DLL CvSeq* cvarFindSquares( IplImage* img, CvMemStorage* storage,int threshold = 100,int inner=0);

/**
 * \brief Initialise the points of square in 3D
 * \param mat	Must be 4*3 matrix with 64f (double)
 * \param ratio	Ratio of width : height, it can be square or any rectangular
 */
AC_DLL void cvarSquareInit(CvMat* mat,float ratio=1);

/**
 * \brief Reverse the order of the square
 *
 * Due to the problem of finding of contour, different sequence will be produced.
 * Thus, the source squae might need to be reversed.
 */
AC_DLL void cvarReverseSquare(CvPoint2D32f sq[4]);

/**
 * \brief Find the camera based on the points
 * \param cam	The camera data, in order to get camera matrix and distortion
 * \param objPts	Matrix of object points in 3D, follow the function cvFindExtrinsicCameraParams2
 * \param imgPts	Matrix of image points in 2D, same as above
 * \param modelview	[Out] The model view of OpenGL matrix,
 */
AC_DLL void cvarFindCamera(CvarCamera* cam,CvMat* objPts,CvMat* imgPts,double* modelview);


/************
Augmented Reality
****************/

/**
 * \brief Load the template
 * \param tpl [out]	The struct store the template
 * \param type The type of the template, default is like ARToolKit, 1 = Whole image
 * \return 1 = success, 0 = fail
 */
AC_DLL int cvarLoadTemplate(CvarTemplate* tpl,const char* filename,int type=0);

/**
 * \brief Load template image, 10x10 size, and convert to ARTag
 *
 * This function will call cvarLoadTag()
 * \param filename	Image file
 */
AC_DLL int cvarLoadTemplateTag(CvarTemplate* tpl,const char* filename);


/**
 * \brief Load the tag information like ARTag
 *
 * \param bit	64-bit hexadecimal value
 */
AC_DLL void cvarLoadTag(CvarTemplate* tpl,long long int bit);

/**
 * \brief Free memory
 */
AC_DLL void cvarReleaseTemplate(CvarTemplate* tpl);

/**
 * \brief Binarise the template
 *
 * Based on ARToolKit algorithm, the template needs to be binarised. From my
 * experience, the grayscale image will reduce the matching result.
 */
AC_DLL void cvarThresholdTemplate(CvarTemplate* tpl,int threshold=100);

/**
 * \brief Compare the square with the four points of the optical flow
 * \param points [in]	Four points from optical flow
 */
AC_DLL int cvarCompareSquare(IplImage* img,CvPoint2D32f* points);


/**
 * \brief Draw the sqaure based on the sequence from FindSquare().
 *
 * Will only get the points of the final square.
 * \param draw	if==1, draw the square
 */
AC_DLL int cvarDrawSquares( IplImage* img, CvSeq* squares,CvPoint2D32f* points,int draw=1);

/**
 * \brief Get final square
 */
AC_DLL int cvarGetSquare( IplImage* img, CvSeq* squares,CvPoint2D32f* points);

/**
 * \brief Create square value, no need release
 * \param src	The array
 * \param ccw	Counter clockwise of the point
 */
AC_DLL void cvarSquare(CvPoint2D32f* src,int width,int height,int ccw=1);


/**
 * \brief Rotate square
 * \param rot	The rotation. 1 - 0 degree, 2 - 90 degree ccw, 3 - 180 degree, 4 - 270 degree ccw
 */
AC_DLL void cvarRotSquare(CvPoint2D32f* src,int rot);

/**
 * \brief Invert perspective view of the image to the 2D view
 * \param input	Input image, the whole image
 * \param output	Output image.
 * \param src	Source 4 points, correspond to input
 * \param dst	Destination 4 points, correspond to output
 */
AC_DLL void cvarInvertPerspective(IplImage* input,IplImage* output,CvPoint2D32f* src,CvPoint2D32f* dst);

/**
 * \brief Get the orientation of the template
 * Remember that the detection is from left to right, bottom to up.
 * Note: If the pattern left-bottom vertex is detected first, then the return value
 * is 1. If the right-bottom vertex is detected first, then the return value is 4
 * \param input	The input image which has the same size of the template
 * \param tpl	The template with four orientation
 * \param match [out]	Template matching value
 * \param thres	Threshold for the sum of square different normalised. If don't want
 * get result, use NULL
 * \return	If match, > 0; else 0. 1 for 1st template, 2 for 2nd, ...
 */
AC_DLL int cvarGetOrientation(IplImage* input,CvarTemplate tpl,double* match=NULL,double thres = 0.98);


/**
 * \brief From the points of the square to OpenGL model view matrix
 * \param points	Points of the square
 * \param cam	Camera
 * \param modelview [out]	Model view matrix
 * \param ratio	Ratio of width:height, 1 = square
 */
AC_DLL void cvarSquareToMatrix(CvPoint2D32f* points,CvarCamera* cam,double* modelview,float ratio=1);


/**
 * \brief Calculate the model view matrix from square of the image.
 * \param img	Image
 * \param points [out]	4 points of the square
 * \param matchThresh	Template matching thresholding value, range: [0,1]
 * \param draw	Draw detected square
 * \return 1 if found, else 0
 */
AC_DLL int cvarArRegistration(IplImage* img,CvPoint2D32f* points,CvarTemplate tpl,int thresh=100,double matchThresh=0.98);


/**
 * \brief Enable debug mode
 *
 * To display some detailed information for debugging
 */
AC_DLL void cvarEnableDebug();

/**
 * \brief Convert square points to rectangle for cropping
 * \return The rectangle
 */
CvRect cvarSquare2Rect(CvPoint2D32f pt[4]);

/**
 * \brief Get all square from cvarFindSquare()
 *
 * Store all the square points in pts
 * \param squares	Can get the squares from cvarFindSquare()
 * \param pts	A vector of points to store the output
 * \return Number of squares
 */
int cvarGetAllSquares(CvSeq* squares,vector<CvPoint2D32f>* pts);


/**
 * \brief This is prototype for multiple marker registration without tracking
 */
int cvarArMultRegNoTrack(IplImage* img,vector<CvarMarker>* vMarker,vector<CvarTemplate> vTpl,CvarCamera* cam,int thresh,double matchThresh);

/**
 * \brief Tracking, check the relationship between two square points
 *
 * After the tracking, if they are related, the pt1 will be updated with pt2
 * \param pt1 The pevious points
 * \param pt2 The current points
 * \return If they are related, return 1, else 0.
 */
int cvarTrack(CvPoint2D32f pt1[4],CvPoint2D32f pt2[4]);


/**
 * \brief Multiple marker registration
 * \param img Incoming image
 * \param vMarker	Vector of marker, which will be stored with the marker information
 * \param vTemplate	Vector of template for template matching
 * \param cam	Camera data
 * \param thresh	Theshold value for binarisation. Can receive AC_THRESH_AUTO for auto-thresholding
 * \param matchThresh	Theshold value for template matching in range [0,1]
 */
int cvarArMultRegistration(IplImage* img,vector<CvarMarker>* vMarker,vector<CvarTemplate> vTpl,CvarCamera* cam,int thresh,double matchThresh);

/**
 * \brief Convert RGB to Gray
 *
 * This function is written because of unknown problem for the function cvCvtColor() in grayscale
 */
void cvarRgb2Gray(IplImage* src,IplImage* dst);

/**
 * \brief Skin segmentation
 */
AC_DLL void acSkinSegmentation(unsigned char* imageData,int width,int height);

/**
 * \brief Finger tracking
 * \param finger	Finger template
 * \param x	Output of x-coordinate
 * \param y Output of y-coordinate
 * \param draw Mark the finger with green point
 */
AC_DLL void acFingerTracking(IplImage* finger,unsigned char* data,int width,int height,int* x,int* y,int draw=1);


///////SURF

/**
 * \brief SURF matching
 */
void cvarSurfMatch(IplImage *image,IplImage* object,
	CvSeq *objectKeypoints,CvSeq *objectDescriptors,
	CvarCamera* cam,double* modelview,
	IplImage* result); 

} //extern


/** @} end functionList */

/**
 * Haar object, which is used for face detection, etc
 */
class AC_DLL CvarHaar {
public:
	CvarHaar();
	/**
	 * \brief Load the XML file
	 */
	CvarHaar(const char* filename);
	virtual ~CvarHaar();
	
	/**
	 * \brief Load the XML file
	 */
	void Load(const char* filename);
	
	
	/**
	 * \brief Clear storage memory
	 */
	void ClearMemory();
	
	/**
	 * \brief Detect object, follow the functions cvHaarDetectObjects
	 */
	CvSeq* DetectObjects(IplImage* image,
		double scale_factor = 1.1,
		int min_neighbors=2,
		int flags=0,
		CvSize min_size=cvSize(30,30));
	
	CvSeq* object; /**< Detected object*/
	CvHaarClassifierCascade* cascade;
	
	CvMemStorage* storage;
};

/*********
Optical flow
************/
class AC_DLL CvarOpticalFlow {
public:
	CvarOpticalFlow();
	virtual ~CvarOpticalFlow();
	
	/**
	 * \brief Initialise with the image size and number of points
	 * \param nPoint	Number of points
	 */
	void Init(IplImage* img,int nPoint,CvPoint2D32f* pts);
	
	/**
	 * \brief Destroy
	 */
	void Destroy();
	
	/**
	 * \brief Update with the image and number of points
	 * \param nPoint	Number of points
	 * \param pts	[out] Points
	 */
	int Update(IplImage* img,int nPoint,CvPoint2D32f* pts,int draw = 0);
	
	IplImage *grey, *prevGrey, *pyramid, *prevPyramid, *swapTemp;
	CvPoint2D32f *points[2], *swapPoints;
	int flags;
	char* status;

private:
	int m_bInit;
};


/***************
AR
**************/

/**
 * AR class
 */
class AC_DLL CvarAr {
public:
	CvarAr();
	virtual ~CvarAr();
	
	/**
	 * \brief Load template file
	 * 
	 * When: Beginning
	 * \param filename	Filename of the template
	 * \return 1 = sucess, 0 = fail
	 */
	int LoadTemplate(char* filename);
	
	/**
	 * \brief Load camera parameter
	 *
	 * When: Beginning
	 * \param filename	Filename of the camera data
	 * \return 1 = success, 0 = fail
	 */
	int LoadCamera(char* filename);
	
	/**
	 * \brief Resize camera parameter based on the camera size
	 *
	 * When: Beginning
	 * \param width	Width of the new camera
	 * \param height 	Height of the new camera
	 */
	void ResizeCamera(int width,int height);
	
	/** 
	 * \brief Get OpenGL projection matrix
	 *
	 * When: Reize callback
	 */
	void GetGlProjection(double* projection);
	
	/**
	 * \brief Detect the marker and get the model view matrix for OpenGL
	 * \param imageData	Pointer to the image data
	 * \param width	Width of the image
	 * \param height	Height of the image
	 * \param modelview	[out] Model view matrix
	 * \param thresh	Threshold value for the image processing
	 * \param matchThresh	Threshold value for the template matching, range [0,1]
	 */
	void DetectMarker(unsigned char* imageData,int width,int height,double* modelview,int thresh=100,double matchThresh=0.98);
	
	/**
	 * \brief Another algorithm for marker detection for lighting problem
	 * \param imageData	Pointer to the image data
	 * \param width	Width of the image
	 * \param height	Height of the image
	 * \param modelview	[out] Model view matrix
	 * \param thresh	Threshold value for the image processing
	 * \param matchThresh	Threshold value for the template matching, range [0,1]
	 */
	void DetectMarkerLight(unsigned char* imageData,int width,int height,double *modelview,int thresh=100,double matchThresh=0.98);

	/**
	 * \brief Get the detected state
	 *
	 * When: Before draw AR object
	 * \return	0 if no marker detected, else 1.
	 */
	int GetState();
	
	CvarCamera* GetCamera();
	CvarTemplate* getTemplate();
	
private:
	CvarTemplate marker;
	CvarCamera camera;
	CvarOpticalFlow *flow;
	
	int state;
};

#endif
