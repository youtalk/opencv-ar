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
 * @file acmath.h
 * @brief Math module
 *
 * These are the old functions that I used. The primary purpose is for 3D computer graphic,
 * thus, the vector is 3, matrix is 4x4, and also based on OpenGL, column-major matrix.
 * @author Allen
 * @date 20090507
 * @todo Decompose the transformation matrix.
 * Add in 3x3 matrix inversion, 2x2.
 */

#pragma once

#ifdef ACDLL
#      define AC_DLL __declspec(dllexport)
#else
#      define AC_DLL
#endif

/**
 * This struct follows OpenCV point structure. So that, without OpenCV,
 * it is also possible to use my own strucutre
 */
struct AcPointi {
    int x;
    int y;
};

/**
 * Same as above
 */
struct AcPointf {
    double x;
    double y;
};

extern "C" {

/**
 * @defgroup mathFunc    Math module
 *
 * Related to math
 * @{
 */

AC_DLL void acVectorPrint(double *v);
AC_DLL void acVectorAdd(double *v1, double *v2, double *vOut);
AC_DLL void acVectorDeduct(double *v1, double *v2, double *vOut);
AC_DLL void acVectorCrossProduct(double *v1, double *v2, double *product);

/**
 * Description:
 * v1, v2, and v3, should be vertices, with x,y,z for each vertex.
 * They are input value.
 * "normal" will be the output value. It must be a vector with x,y,z also
 */
AC_DLL void acVectorNormal(double *v1, double *v2, double *v3, double *normal);
AC_DLL double acVectorMagnitude(double *v);
AC_DLL void acVectorNormalise(double *vIn, double *vOut);

/**
 * Get the normalised normal
 */
AC_DLL void acVectorNormal2(double *v1, double *v2, double *v3, double *nv);

AC_DLL double acRad2Deg(double rad);
AC_DLL double acDeg2Rad(double deg);

/**
 * This coding is according to the calculation of OpenGL, therefore the output matrix is also 
 * according to OpenGL matrix
 */
AC_DLL void acMatrixRotate(double deg, double x, double y, double z, double *m);
AC_DLL void acMatrixTranslate(double x, double y, double z, double *m);
AC_DLL void acMatrixScale(double x, double y, double z, double *m);
AC_DLL void acMatrixIdentity(double *m);

/**
 * Dot product base on row-majored matrix
 * Therefore, m1 and m2 should be row-majored matrix
 */
AC_DLL double acMatrixDotProduct(double *m1, double *m2, int col, int row);

/**
 * Based on row-majored matrix
 * m1, m2 and mOut should be row-majored
 */
AC_DLL void acMatrixMultiply(double *m1, double *m2, double *mOut);
AC_DLL void acMatrixPrint(double *m);
AC_DLL void acMatrixTranspose(double *m);

/**
 * The algorithm come from http://www.j3d.org/matrix_faq/matrfaq_latest.html
 * @param q Quaternion with w,x,y,z
 */
AC_DLL void acMatrixToQuaternion(double* m, double* q);

/**
 * Convert quaternion to matrix in 4x4, the data of the matrix will be
 * overwritten.
 */
AC_DLL void acQuaternionToMatrix(double* q, double* m);

/**
 * This algorithm is from OpenCV square example
 */
AC_DLL double acAngle(AcPointi* pt1, AcPointi* pt2, AcPointi* pt0);

/**
 * Calculate length between two vertices
 */
AC_DLL double acCalcLength(AcPointf pt1, AcPointf pt2);

/**
 * @brief Invert modelview matrix 4x4
 *
 * From http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
 */
AC_DLL void acMatrix4Invert(double m[]);

/**
 * @brief Get determinant of 4x4 matrix
 *
 ** From http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
 */
AC_DLL double acMatrix4GetDeterminant(double m[]);

/**
 * @brief decompose the 4x4 matrix
 *
 * Algorithm from http://www.gamedev.net/community/forums/topic.asp?topic_id=441695
 * @param m    [in] 4x4 row-majored transformation matrix
 * @param t    [out]    Translation 3x1 vector
 * @param s    [out] Scale 3x1 vector
 * @param r    [out] Rotation 4x4 matrix
 */
AC_DLL void acMatrixDecompose(double m[], double t[], double s[], double r[]);

/************
 Bit operation and 2D square array, related to image processing
 ***********/

/**
 * @brief Rotate square array in unsigned byte
 * @param rot 1 = 90 deg clockwise, 2 = 180 deg clockwise, 3 = 270 deg clockwise
 */
AC_DLL void acArray2DRotateub(unsigned char* arr, int w, int h, int rot);

/**
 * @brief Print array 2D
 */
AC_DLL void acArray2DPrintub(unsigned char* arr, int w, int h);

/**
 * @brief Convert array 2D to bit
 *
 * The array data must be binary. The bit data is like the following feature <br>
 * Array sequence:
 * 1248 1248 (bit)
 * 0000 0000 => 0 0 <br>
 * 0001 1000 => 8 1<br>
 * 0011 1100 => C 3<br>
 * 0111 1110 => E 7<br>
 * 1000 0001 => 1 8<br>
 * 0100 0010 => 2 4<br>
 * 0010 0100 => 4 2<br>
 * Result: 0x00183C7E814224 <br>
 * And remember that, the image should be flipped vertically, that means printing the array will always
 * flipped from the image, because the printing is print bottom-up. <br>
 * If read the hex from left to right, then, the image should be read from right to left yet bottom-up, and vice versa. <br>
 * Thus, if a person want to convert the image code to hex manually, write the hex from left to right, then he/she need to read
 * the image from right to left and bottom-up.
 * @param bit    Store the output, the size of bit must be
 * large enough to hold the data, such as "long long" for 8-byte (or 64bit)
 */
AC_DLL void acArray2DToBit(unsigned char* arr, int w, int h,
                           long long int* bit);

/**
 * @brief Convert bit to 2D array
 */
AC_DLL void acBitToArray2D(long long int bit, unsigned char* arr, int w, int h);

/**
 * @brief Rotate bit
 *
 * Convert the bit to 2D square array, then rotate, then convert to bit again
 * Expected size of bit is 64-bit.
 * @param rot 1=90 degree clockwise, 2=180, 3=270
 */
AC_DLL void acBitRotate(long long int* bit, int rot, int w, int h);

/** @} end mathFunc */

} // extern
