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
 * @file acgl.h
 * @brief Header acgl
 *
 * Description: Several OpenGL functions, for easier use.
 * 20100226 Add acGlSelectd() because of OpenSceneGraph
 * @author    Allen
 * @date    20080904
 * @version    0.1.0
 */

#pragma once

#ifdef ACDLL
#      define AC_DLL __declspec(dllexport)
#else
#      define AC_DLL
#endif

/**
 * @defgroup functionGl acgl functions
 * @{
 */

extern "C" {
/**
 * @brief Create image texture mapping
 *
 * For the video, with the ratio height : width = 1: width/height
 * @param depth    How many bytes per pixel
 * @param swap    1 for swapping RGB, 0 no swapping needed.
 */
AC_DLL void acGlTexture(unsigned char* data, int width, int height, int depth,
                        int swap = 0);

/**
 * The definition of this function is written again and again. Therefore, just make it
 * into a function. The disadvantage of this function is generate the texture and delete
 * the texture every time render.
 * This function is used for video (or image), despise the ratio of width and height
 * Therefore, it is very good in GL_PROJECTION.
 * The function is specifically to map the whole window with the texture. Therefore,
 * remember to use GL_TEXTURE_2D
 */
AC_DLL void acGlTextureProject(void* buffer, int width, int height, int depth,
                               int swap = 0);

/**
 * Not sure it will work in all windows or not
 */
AC_DLL void acGlGetWindowSize(int* width, int* height);

/**
 * Get the ratio of "raster double" to "window pixel"
 */
AC_DLL double acGlPixelRasterRatioX();
AC_DLL double acGlPixelRasterRatioY();

/**
 * Super power conversion function. Actually, the pixel refers to the 3D transformation unit. Not the windows 2D pixel unit.
 * This is because, the pixels will be influenced by the 3D transformation.
 * Note: These functions should not be used after glRasterPos(), cause it will reset the state
 */
AC_DLL double acGlPixel2RasterX(double input); // For raster, convert the value of glGet() to glRasterPos()
AC_DLL double acGlRaster2PixelX(double input);
AC_DLL double acGlPixel2RasterY(double input);
AC_DLL double acGlRaster2PixelY(double input);

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
 * @brief Default process hit function
 * @return Front object
 */
AC_DLL int acGlProcessHit(GLint hits, GLuint buffer[]);

/**
 * @brief Select OpenGL matrix
 * @param processHit    Callback function
 * @return If nothing, return -1
 */
AC_DLL int acGlSelect(
        int x, int y, int width, int height, double* projection,
        double* modelview, void (*draw)(),
        int (*processHit)(GLint hit, GLuint buffer[]) = acGlProcessHit);

/**
 * @brief Check object (point) is occluded or not
 * @return 0 if no occlusion, else 1.
 */
AC_DLL int acGlIsOccluded(double objX, double objY, double objZ);

} // extern

/** @} end group */
