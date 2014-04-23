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


#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>


#include "opencvar/acmath.h"

#define AC_PI 3.1415927


using namespace std;


void acVectorPrint(float *v) {
	for(int i=0;i<3;i++) {
		cout<<v[i]<<"\t";
	}
	cout<<endl;
}

void acVectorAdd(float *v1,float *v2,float *vOut) {
	vOut[0]=v1[0]+v2[0];
	vOut[1]=v1[1]+v2[1];
	vOut[2]=v1[2]+v2[2];
}

void acVectorDeduct(float *v1,float *v2,float *vOut) {
	vOut[0]=v1[0]-v2[0];
	vOut[1]=v1[1]-v2[1];
	vOut[2]=v1[2]-v2[2];
}

void acVectorCrossProduct(float *v1,float *v2,float *product) {
	product[0]=v1[1]*v2[2] - v1[2]*v2[1];
	product[1]=-(v2[2]*v1[0] - v2[0]*v1[2]);
	product[2]=v1[0]*v2[1] - v1[1]*v2[0];
}

void acVectorNormal(float *v1,float *v2,float *v3,float *normal) {
	//Description:
	//v1, v2, and v3, should be vertices, with x,y,z for each vertex.
	//They are input value.
	//"normal" will be the output value. It must be a vector with x,y,z also

	float temp[3],temp2[3];
	acVectorDeduct(v2,v1,temp);
	acVectorDeduct(v3,v1,temp2);
	acVectorCrossProduct(temp,temp2,normal);
}

float acVectorMagnitude(float *v) {
	return (float)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void acVectorNormalise(float *vIn,float *vOut) {
	float mag=acVectorMagnitude(vIn);
	vOut[0]=vIn[0]/mag;
	vOut[1]=vIn[1]/mag;
	vOut[2]=vIn[2]/mag;
}

void acVectorNormal2(float *v1,float *v2,float *v3,float *nv) {
	float temp[3];
	acVectorNormal(v1,v2,v3,temp);
	acVectorNormalise(temp,nv);
}


float acRad2Deg(float rad) {
	return (rad*180)/AC_PI;
}
float acDeg2Rad(float deg) {
	return (deg*AC_PI)/180;
}

void acMatrixRotate(float deg,float x,float y,float z,float *m) {
	//This coding is according to the calculation of OpenGL, therefore the output matrix is also 
	// according to OpenGL matrix

	float temp[16];
	float c=cos(acDeg2Rad(deg));
	float s=sin(acDeg2Rad(deg));

	//Normalise the x,y,z
	float mag=sqrt(x*x+y*y+z*z);
	x=x/mag;
	y=y/mag;
	z=z/mag;

	temp[0]=(x*x*(1-c))+c;	temp[4]=(x*y*(1-c))-(z*s);	temp[8]=(x*z*(1-c))+(y*s);	temp[12]=0;
	temp[1]=(y*x*(1-c))+(z*s);	temp[5]=(y*y*(1-c))+c;	temp[9]=(y*z*(1-c))-(x*s);	temp[13]=0;
	temp[2]=(x*z*(1-c))-(y*s);	temp[6]=(y*z*(1-c))+(x*s);	temp[10]=(z*z*(1-c))+c;	temp[14]=0;
	temp[3]=0;	temp[7]=0;	temp[11]=0;	temp[15]=1;

	acMatrixMultiply(temp,m,m);
}

void acMatrixTranslate(float x,float y,float z,float *m) {
	float temp[16];

	temp[0]=1;	temp[4]=0;	temp[8]=0;	temp[12]=x;
	temp[1]=0;	temp[5]=1;	temp[9]=0;	temp[13]=y;
	temp[2]=0;	temp[6]=0;	temp[10]=1;	temp[14]=z;
	temp[3]=0;	temp[7]=0;	temp[11]=0;	temp[15]=1;

	acMatrixMultiply(temp,m,m); //Because of the multiplication is using row-majored
}

void acMatrixScale(float x,float y,float z,float *m) {
	float temp[16];

	temp[0]=x;	temp[4]=0;	temp[8]=0;	temp[12]=0;
	temp[1]=0;	temp[5]=y;	temp[9]=0;	temp[13]=0;
	temp[2]=0;	temp[6]=0;	temp[10]=z;	temp[14]=0;
	temp[3]=0;	temp[7]=0;	temp[11]=0;	temp[15]=1;

	acMatrixMultiply(temp,m,m);
}

void acMatrixIdentity(float *m) {
	for(int i=0;i<16;i++) {
		m[i]=0;
	}
	m[0]=1;
	m[5]=1;
	m[10]=1;
	m[15]=1;
}

float acMatrixDotProduct(float *m1,float *m2,int col,int row) {
	//Dot product base on row-majored matrix
	//Therefore, m1 and m2 should be row-majored matrix

	float ret=0;
	for(int i=0;i<4;i++) {
		ret+=m1[row*4+i]*m2[i*4+col];
	}
	return ret;
}

void acMatrixMultiply(float *m1,float *m2,float *mOut) {
	//Based on row-majored matrix
	//m1, m2 and mOut should be row-majored
	float temp[16]; //Must use the temporary, because if the address of mOut is same
		//as m1 or m2, the DotProduct value will be altered.
	for(int j=0;j<4;j++) {
		for(int i=0;i<4;i++) {
			temp[j*4+i]=acMatrixDotProduct(m1,m2,i,j);
		}
	}
	memcpy(mOut,temp,16*sizeof(float));
}

void acMatrixPrint(float *m) {
	for(int j=0;j<4;j++) {
		for(int i=0;i<4;i++) {
			printf("%f\t",m[j*4+i]);
		}
		printf("\n");
	}
	printf("\n");
}

void acMatrixTranspose(float *m) {
	float temp[16];
	memcpy(temp,m,16*sizeof(float));
	for(int i=0;i<4;i++) {
		for(int j=0;j<4;j++) {
			m[j*4+i]=temp[i*4+j];
		}
	}
}


/**
 * Transpose matrix in double type
 */
void acMatrixTransposed(double* m) {
	double temp[16];
	memcpy(temp,m,16*sizeof(double));
	for(int j=0;j<4;j++) {
		for(int i=0;i<4;i++)
			m[j*4+i] = temp[i*4+j];
	}
}

/**
 * The algorithm come from http://www.j3d.org/matrix_faq/matrfaq_latest.html
 * \param q Quaternion with w,x,y,z
 */
void acMatrixToQuaternion(double* m,double* q) {
	double x,y,z,w,s;
	double t = 1 + m[0] + m[5] + m[10];
	if(t > 0.00000001) {
		s = sqrt(t) * 2;
		x = (m[9] - m[6]) / s; //Because of OpenGL uses column-major matrix, mat[6]-mat[9] is used
		y = (m[2] - m[8]) /s;
		z = (m[4]-m[1])/s;
		w = 0.25 * s;
	}
	else if(m[0] > m[5] && m[0] > m[10]) {
		s = sqrt(1 + m[0] - m[5] - m[10]) * 2;
		x = 0.25 * s;
		y = (m[4] + m[1]) /s;
		z = (m[2]+m[8]) /s;
		w = (m[9] - m[6]) /s;
	}
	else if(m[5]> m[10]) {
		s = sqrt(1+m[5] - m[0] - m[10])*2;
		x = (m[4] +m[1]) /s;
		y = 0.25*s;
		z = (m[9] + m[6])/s;
		w = (m[2] - m[8])/s;
	}
	else {
		s = sqrt(1 +m[10] - m[0] - m[5])*2;
		x = (m[2] + m[8])/s;
		y = (m[9] + m[6])/s;
		z = 0.25*s;
		w = (m[4] - m[1])/s;
	}
	q[0] = w;
	q[1] = x;
	q[2] = y;
	q[3] = z;
}

/**
 * Convert quaternion to matrix in 4x4, the data of the matrix will be
 * overwritten.
 */
void acQuaternionToMatrix(double* q,double* m) {
	double xx,xy,xz,xw,yy,yz,yw,zz,zw;
	xx = q[1] * q[1];
	xy = q[1] * q[2];
	xz = q[1] * q[3];
	xw = q[1] * q[0];
	yy = q[2] * q[2];
	yz = q[2] * q[3];
	yw = q[2] * q[0];
	zz = q[3] * q[3];
	zw = q[3] * q[0];
	
	m[0] = 1 - 2 * (yy + zz);
	m[1] = 2 * (xy-zw);
	m[2] = 2 * (xz+yw);
	
	m[4] = 2 * (xy+zw);
	m[5] = 1 - 2 * (xx + zz);
	m[6] = 2 * (yz - xw);
	
	m[8] = 2 * (xz - yw);
	m[9] = 2 *( yz + xw);
	m[10] = 1 - 2 * (xx+yy);
}


/**
 * This algorithm is from OpenCV square example
 */
double acAngle( AcPointi* pt1, AcPointi* pt2, AcPointi* pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

/**
 * Calculate length between two vertices
 */
double acCalcLength(AcPointf pt1,AcPointf pt2) {
	double dx = pt1.x - pt2.x;
	double dy = pt1.y - pt2.y;
	
	return sqrt(dx*dx + dy*dy);
}

/**
 * \brief Get determinant of 4x4 matrix
 *
 ** From http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
 */
float acMatrix4GetDeterminant(float m[]) {
	float value = 
		m[0*4+3] * m[1*4+2] * m[2*4+1] * m[3*4+0]-m[0*4+2] * m[1*4+3] * m[2*4+1] * m[3*4+0]-m[0*4+3] * m[1*4+1] * m[2*4+2] * m[3*4+0]+m[0*4+1] * m[1*4+3]    * m[2*4+2] * m[3*4+0]+
		m[0*4+2] * m[1*4+1] * m[2*4+3] * m[3*4+0]-m[0*4+1] * m[1*4+2] * m[2*4+3] * m[3*4+0]-m[0*4+3] * m[1*4+2] * m[2*4+0] * m[3*4+1]+m[0*4+2] * m[1*4+3]    * m[2*4+0] * m[3*4+1]+
		m[0*4+3] * m[1*4+0] * m[2*4+2] * m[3*4+1]-m[0*4+0] * m[1*4+3] * m[2*4+2] * m[3*4+1]-m[0*4+2] * m[1*4+0] * m[2*4+3] * m[3*4+1]+m[0*4+0] * m[1*4+2]    * m[2*4+3] * m[3*4+1]+
		m[0*4+3] * m[1*4+1] * m[2*4+0] * m[3*4+2]-m[0*4+1] * m[1*4+3] * m[2*4+0] * m[3*4+2]-m[0*4+3] * m[1*4+0] * m[2*4+1] * m[3*4+2]+m[0*4+0] * m[1*4+3]    * m[2*4+1] * m[3*4+2]+
		m[0*4+1] * m[1*4+0] * m[2*4+3] * m[3*4+2]-m[0*4+0] * m[1*4+1] * m[2*4+3] * m[3*4+2]-m[0*4+2] * m[1*4+1] * m[2*4+0] * m[3*4+3]+m[0*4+1] * m[1*4+2]    * m[2*4+0] * m[3*4+3]+
		m[0*4+2] * m[1*4+0] * m[2*4+1] * m[3*4+3]-m[0*4+0] * m[1*4+2] * m[2*4+1] * m[3*4+3]-m[0*4+1] * m[1*4+0] * m[2*4+2] * m[3*4+3]+m[0*4+0] * m[1*4+1]    * m[2*4+2] * m[3*4+3];
	
	return value;
}

/**
 * \brief Invert modelview matrix 4x4
 *
 * From http://www.euclideanspace.com/maths/algebra/matrix/functions/inverse/fourD/index.htm
 */
void acMatrix4Invert(float m[]) {
	float res[16] = {0};
	res[0*4+0] = m[1*4+2]*m[2*4+3]*m[3*4+1] - m[1*4+3]*m[2*4+2]*m[3*4+1] + m[1*4+3]*m[2*4+1]*m[3*4+2] - m[1*4+1]*m[2*4+3]*m[3*4+2] - m[1*4+2]*m[2*4+1]*m[3*4+3] + m[1*4+1]*m[2*4+2]*m[3*4+3];
	res[0*4+1] = m[0*4+3]*m[2*4+2]*m[3*4+1] - m[0*4+2]*m[2*4+3]*m[3*4+1] - m[0*4+3]*m[2*4+1]*m[3*4+2] + m[0*4+1]*m[2*4+3]*m[3*4+2] + m[0*4+2]*m[2*4+1]*m[3*4+3] - m[0*4+1]*m[2*4+2]*m[3*4+3];
	res[0*4+2] = m[0*4+2]*m[1*4+3]*m[3*4+1] - m[0*4+3]*m[1*4+2]*m[3*4+1] + m[0*4+3]*m[1*4+1]*m[3*4+2] - m[0*4+1]*m[1*4+3]*m[3*4+2] - m[0*4+2]*m[1*4+1]*m[3*4+3] + m[0*4+1]*m[1*4+2]*m[3*4+3];
	res[0*4+3] = m[0*4+3]*m[1*4+2]*m[2*4+1] - m[0*4+2]*m[1*4+3]*m[2*4+1] - m[0*4+3]*m[1*4+1]*m[2*4+2] + m[0*4+1]*m[1*4+3]*m[2*4+2] + m[0*4+2]*m[1*4+1]*m[2*4+3] - m[0*4+1]*m[1*4+2]*m[2*4+3];
	res[1*4+0] = m[1*4+3]*m[2*4+2]*m[3*4+0] - m[1*4+2]*m[2*4+3]*m[3*4+0] - m[1*4+3]*m[2*4+0]*m[3*4+2] + m[1*4+0]*m[2*4+3]*m[3*4+2] + m[1*4+2]*m[2*4+0]*m[3*4+3] - m[1*4+0]*m[2*4+2]*m[3*4+3];
	res[1*4+1] = m[0*4+2]*m[2*4+3]*m[3*4+0] - m[0*4+3]*m[2*4+2]*m[3*4+0] + m[0*4+3]*m[2*4+0]*m[3*4+2] - m[0*4+0]*m[2*4+3]*m[3*4+2] - m[0*4+2]*m[2*4+0]*m[3*4+3] + m[0*4+0]*m[2*4+2]*m[3*4+3];
	res[1*4+2] = m[0*4+3]*m[1*4+2]*m[3*4+0] - m[0*4+2]*m[1*4+3]*m[3*4+0] - m[0*4+3]*m[1*4+0]*m[3*4+2] + m[0*4+0]*m[1*4+3]*m[3*4+2] + m[0*4+2]*m[1*4+0]*m[3*4+3] - m[0*4+0]*m[1*4+2]*m[3*4+3];
	res[1*4+3] = m[0*4+2]*m[1*4+3]*m[2*4+0] - m[0*4+3]*m[1*4+2]*m[2*4+0] + m[0*4+3]*m[1*4+0]*m[2*4+2] - m[0*4+0]*m[1*4+3]*m[2*4+2] - m[0*4+2]*m[1*4+0]*m[2*4+3] + m[0*4+0]*m[1*4+2]*m[2*4+3];
	res[2*4+0] = m[1*4+1]*m[2*4+3]*m[3*4+0] - m[1*4+3]*m[2*4+1]*m[3*4+0] + m[1*4+3]*m[2*4+0]*m[3*4+1] - m[1*4+0]*m[2*4+3]*m[3*4+1] - m[1*4+1]*m[2*4+0]*m[3*4+3] + m[1*4+0]*m[2*4+1]*m[3*4+3];
	res[2*4+1] = m[0*4+3]*m[2*4+1]*m[3*4+0] - m[0*4+1]*m[2*4+3]*m[3*4+0] - m[0*4+3]*m[2*4+0]*m[3*4+1] + m[0*4+0]*m[2*4+3]*m[3*4+1] + m[0*4+1]*m[2*4+0]*m[3*4+3] - m[0*4+0]*m[2*4+1]*m[3*4+3];
	res[2*4+2] = m[0*4+1]*m[1*4+3]*m[3*4+0] - m[0*4+3]*m[1*4+1]*m[3*4+0] + m[0*4+3]*m[1*4+0]*m[3*4+1] - m[0*4+0]*m[1*4+3]*m[3*4+1] - m[0*4+1]*m[1*4+0]*m[3*4+3] + m[0*4+0]*m[1*4+1]*m[3*4+3];
	res[2*4+3] = m[0*4+3]*m[1*4+1]*m[2*4+0] - m[0*4+1]*m[1*4+3]*m[2*4+0] - m[0*4+3]*m[1*4+0]*m[2*4+1] + m[0*4+0]*m[1*4+3]*m[2*4+1] + m[0*4+1]*m[1*4+0]*m[2*4+3] - m[0*4+0]*m[1*4+1]*m[2*4+3];
	res[3*4+0] = m[1*4+2]*m[2*4+1]*m[3*4+0] - m[1*4+1]*m[2*4+2]*m[3*4+0] - m[1*4+2]*m[2*4+0]*m[3*4+1] + m[1*4+0]*m[2*4+2]*m[3*4+1] + m[1*4+1]*m[2*4+0]*m[3*4+2] - m[1*4+0]*m[2*4+1]*m[3*4+2];
	res[3*4+1] = m[0*4+1]*m[2*4+2]*m[3*4+0] - m[0*4+2]*m[2*4+1]*m[3*4+0] + m[0*4+2]*m[2*4+0]*m[3*4+1] - m[0*4+0]*m[2*4+2]*m[3*4+1] - m[0*4+1]*m[2*4+0]*m[3*4+2] + m[0*4+0]*m[2*4+1]*m[3*4+2];
	res[3*4+2] = m[0*4+2]*m[1*4+1]*m[3*4+0] - m[0*4+1]*m[1*4+2]*m[3*4+0] - m[0*4+2]*m[1*4+0]*m[3*4+1] + m[0*4+0]*m[1*4+2]*m[3*4+1] + m[0*4+1]*m[1*4+0]*m[3*4+2] - m[0*4+0]*m[1*4+1]*m[3*4+2];
	res[3*4+3] = m[0*4+1]*m[1*4+2]*m[2*4+0] - m[0*4+2]*m[1*4+1]*m[2*4+0] + m[0*4+2]*m[1*4+0]*m[2*4+1] - m[0*4+0]*m[1*4+2]*m[2*4+1] - m[0*4+1]*m[1*4+0]*m[2*4+2] + m[0*4+0]*m[1*4+1]*m[2*4+2];
	
	float deter = acMatrix4GetDeterminant(m);
	for(int i=0;i<16;i++) {
		m[i] = (1.0/deter) * res[i];
	}
}

/**
 * \brief decompose the 4x4 matrix
 *
 * Algorithm from http://www.gamedev.net/community/forums/topic.asp?topic_id=441695
 * \param m	[in] 4x4 row-majored transformation matrix
 * \param t	[out]	Translation 3x1 vector
 * \param s	[out] Scale 3x1 vector
 * \param r	[out] Rotation 4x4 matrix
 */
void acMatrixDecompose(float m[],float t[],float s[],float r[]) {
	//Translation
	t[0] = m[0*4+3];
	t[1] = m[1*4+3];
	t[2] = m[2*4+3];
	
	//Scale
	s[0] = sqrt( pow(m[0*4+0],2) + pow(m[0*4+1],2) + pow(m[0*4+2],2) );
	s[1] = sqrt( pow(m[1*4+0],2) + pow(m[1*4+1],2) + pow(m[1*4+2],2) );
	s[2] = sqrt( pow(m[2*4+0],2) + pow(m[2*4+1],2) + pow(m[2*4+2],2) );
	
	//Rotation
	float rot[16] = {
			m[0*4+0] / s[0],	m[0*4+1] / s[0],	m[0*4+2] / s[0],	0,
			m[1*4+0] / s[1],	m[1*4+1] / s[1],	m[1*4+2] / s[1],	0,
			m[2*4+0] / s[2],	m[2*4+1] / s[2],	m[2*4+2] / s[2],	0,
			0, 0, 0, 1
		};
	memcpy(r,rot,sizeof(float) * 16);
}


/************
Bit operation and 2D square array, related to image processing
***********/

/**
 * \brief Rotate square array in unsigned byte
 * \param rot 1 = 90 deg clockwise, 2 = 180 deg clockwise, 3 = 270 deg clockwise
 */
void acArray2DRotateub(unsigned char* arr,int w,int h,int rot) {
	unsigned char* temp = (unsigned char*)malloc(w*h);
	memcpy(temp,arr,w*h);
	
	/*
	rotation 1:
	left top => right top
	0,0 => 7,0
	0,1 => 6,0
	0,2 => 5,0
	
	rotation 2:
	0,0 => 7,7
	0,1 => 6,7
	
	rotation 3:
	left top => left bottom
	0,0 => 0,7
	0,1 => 1,7
	0,2 => 2,7
	*/
	
	for(int i=0;i<h;i++) {
		for(int j=0;j<w;j++) {
			switch(rot) {
			case 1:
				arr[i*w+j] = temp[(h-1-j)*w+i];
				break;
			case 2:
				arr[i*w+j] = temp[(h-1-i)*w + (h-1-j)];
				break;
			case 3:
				arr[i*w+j] = temp[j*w+(h-1-i)];
			}
		}
	}

	
	free(temp);
}

/**
 * \brief Print array 2D
 */
void acArray2DPrintub(unsigned char* arr,int w,int h) {
	for(int i=0;i<h;i++) {
		for(int j=0;j<w;j++) {
			printf("%d ",arr[i*w+j]);
		}
		printf("\n");
	}
}

/**
 * \brief Convert array 2D to bit
 *
 * The array data must be binary
 * \param bit	Store the output, the size of bit must be 
 * large enough to hold the data, such as "long long" for 8-byte (or 64bit)
 */
void acArray2DToBit(unsigned char* arr,int w,int h,long long int* bit) {
	*bit = 0;
	for(int i=0;i<h;i++) {
		for(int j=w-1;j>=0;j--) {
			*bit = *bit<<1;
			*bit = *bit|arr[i*w+j];
		}
	}
}

/**
 * \brief Convert bit to 2D array
 */
void acBitToArray2D(long long int bit,unsigned char* arr,int w,int h) {
	for(int i=h-1;i>=0;i--) {
		for(int j=0;j<w;j++) {
			arr[i*w+j] = 1 & bit;
			bit = bit >> 1;
		}
	}
}

/**
 * \brief Rotate bit
 *
 * Convert the bit to 2D square array, then rotate, then convert to bit again
 * Expected size of bit is 64-bit.
 * \param rot 1=90 degree clockwise, 2=180, 3=270
 */
void acBitRotate(long long int* bit,int rot) {
	unsigned char arr[64];
	acBitToArray2D(*bit,arr,8,8);
	acArray2DRotateub(arr,8,8,rot);
	acArray2DToBit(arr,8,8,bit);
}


