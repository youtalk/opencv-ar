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
 * \file acgl.h
 * \brief Header acgl
 *
 * Description: Several OpenGL functions, for easier use.
 * 20100226 Add acGlSelectd() because of OpenSceneGraph
 * \author	Allen
 * \date	20080904
 * \version	0.1.0
 *
 * \deprecated acGlSelect() 
 * Superceded by acGlSelect2()
 */

#pragma once

#ifdef ACDLL
#      define AC_DLL __declspec(dllexport)
#else
#      define AC_DLL
#endif

/**
 * \defgroup functionGl acgl functions
 * @{
 */
 
extern "C" {
/**
 * \brief Create image texture mapping
 *
 * For the video, with the ratio height : width = 1: width/height
 * \param depth	How many bytes per pixel
 * \param swap	1 for swapping RGB, 0 no swapping needed.
 */
AC_DLL void acGlTexture(unsigned char* data,int width,int height,int depth,int swap = 0);

/**
 * The definition of this function is written again and again. Therefore, just make it
 * into a function. The disadvantage of this function is generate the texture and delete
 * the texture every time render.
 * This function is used for video (or image), despise the ratio of width and height
 * Therefore, it is very good in GL_PROJECTION.
 * The function is specifically to map the whole window with the texture. Therefore,
 * remember to use GL_TEXTURE_2D
 */
AC_DLL void acGlTextureProject(void* buffer,int width,int height,int depth,int swap = 0);

/**
 * \brief Selection/picking
 *
 * For picking or selection, the projection and modelview are defined outside the
 * function, so that, it is able to be used in custom environment <br>
 * Note that, glLoadName() is needed in the draw() callback function so that the value
 * of the "name" will be returned.
 * \param x	Coordinate-x
 * \param y Coordinate-y
 * \param projection	OpenGL projection matrix
 * \param modelview	OpenGL modelview matrix
 * \param draw	Callback function to draw the selectable/pickable object
 * \return Return the "name" of the object, if no object selected, return 0.
 */
AC_DLL int acGlSelect(int x,int y,float* projection,float* modelview,void (*draw)());

/**
 * Not sure it will work in all windows or not
 */
AC_DLL void acGlGetWindowSize(int* width,int* height);

/**
 * Get the ratio of "raster float" to "window pixel"
 */
AC_DLL float acGlPixelRasterRatioX();
AC_DLL float acGlPixelRasterRatioY();

/**
 * Super power conversion function. Actually, the pixel refers to the 3D transformation unit. Not the windows 2D pixel unit.
 * This is because, the pixels will be influenced by the 3D transformation.
 * Note: These functions should not be used after glRasterPos(), cause it will reset the state
 */
AC_DLL float acGlPixel2RasterX(float input); //For raster, convert the value of glGet() to glRasterPos()
AC_DLL float acGlRaster2PixelX(float input);
AC_DLL float acGlPixel2RasterY(float input);
AC_DLL float acGlRaster2PixelY(float input);


/**
 * GLUT font functions
 */
AC_DLL void acGlutFontBitmap(char* str);
AC_DLL void acGlutFontStroke(char* str);



/**
 * Print OpenGL state
 */
AC_DLL void acGlPrintState();

/**
 * Print OpenGL glGet*()
 */
AC_DLL void acGlPrintGet(GLenum pname);

/**
 * \brief Default process hit function
 * \return Front object
 */
AC_DLL int acGlProcessHit(GLint hits,GLuint buffer[]);

/**
 * \brief Select OpenGL matrix
 * \param processHit	Callback function
 * \return If nothing, return -1
 */
AC_DLL int acGlSelect2(int x,int y,int width,int height,float* projection,float* modelview,void (*draw)(),
		int (*processHit)(GLint hit,GLuint buffer[]) = acGlProcessHit);

/**
 * \brief Select OpenGL matrix in double
 * \param processHit	Callback function
 * \param viewport	If want get self-defined viewport
 * \param inverse	Inverse the y-coordinate
 * \return If nothing, return -1
 */
AC_DLL int acGlSelectd(int x,int y,int width,int height,double* projection,double* modelview,void (*draw)(),
		GLint viewport[4]=0,
		int inverse=0,
		int (*processHit)(GLint hit,GLuint buffer[]) = acGlProcessHit);

		
/**
 * \brief Check object (point) is occluded or not
 * \return 0 if no occlusion, else 1.
 */
AC_DLL int acGlIsOccluded(float objX,float objY,float objZ);

}//extern

/** @} end group */

/**************
Object font
**************/

/**
 * \brief WGL bitmap font
 */
class AC_DLL acWglFont {
public:
	acWglFont();
	virtual ~acWglFont();
	
	/**
	 * \brief Initialization
	 */
	void init(char* szWindow,char* szTypeface,int size=12,int bold=0,int italic=0,
		int underline=0,int strikeout=0);
	
	void print(const char* str,...);
	void printMulti(int height,const char* str,...);
	
	int isInit();
	void destroy();
	
private:
	char* m_szWindow;
	GLuint m_uBase;
	int m_bInit;
};
