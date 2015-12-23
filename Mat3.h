#ifndef DGP_MAT3_H
#define DGP_MAT3_H

#include"Vec3.h"

class Mat3 {
public:
	double m11, m12, m13;
	double m21, m22, m23;
	double m31, m32, m33;

	Mat3() {};

	Mat3(Vec3 in) { 
		m11=0; m12=-in.z; m13=in.y;
		m21=in.z; m22=0; m23=-in.x;
		m31=-in.y; m32=in.x; m33=0;
	};

	Mat3(double m11_in, double m12_in, double m13_in, double m21_in, double m22_in, double m23_in, double m31_in, double m32_in, double m33_in){
		m11 = m11_in;
		m12 = m12_in;
		m13 = m13_in;
		m21 = m21_in;
		m22 = m22_in;
		m23 = m23_in;
		m31 = m31_in;
		m32 = m32_in;
		m33 = m33_in;
	};

	Mat3 operator*( double in ){ return Mat3(m11*in, m12*in, m13*in, m21*in, m22*in, m23*in, m31*in, m32*in, m33*in ); }
	Mat3 operator/( double in ){ return Mat3(m11/in, m12/in, m13/in, m21/in, m22/in, m23/in, m31/in, m32/in, m33/in ); }
	Mat3 operator+( Mat3 in ){ return Mat3(m11+in.m11, m12+in.m12, m13+in.m13, m21+in.m21, m22+in.m22, m23+in.m23, m31+in.m31, m32+in.m32, m33+in.m33 ); }

	Mat3 operator*( Mat3 in) {
		Mat3 out;
		out.m11 = m11*in.m11 + m12*in.m21 + m13*in.m31;
		out.m12 = m11*in.m12 + m12*in.m22 + m13*in.m32;
		out.m13 = m11*in.m13 + m12*in.m23 + m13*in.m33;
		out.m21 = m21*in.m11 + m22*in.m21 + m23*in.m31;
		out.m22 = m21*in.m12 + m22*in.m22 + m23*in.m32;
		out.m23 = m21*in.m13 + m22*in.m23 + m23*in.m33;
		out.m31 = m31*in.m11 + m32*in.m21 + m33*in.m31;
		out.m32 = m31*in.m12 + m32*in.m22 + m33*in.m32;
		out.m33 = m31*in.m13 + m32*in.m23 + m33*in.m33;
		return out;
	};

	Vec3 operator*( Vec3 in) {
		
		double x = m11*in.x + m12*in.y + m13*in.z;
		double y = m21*in.x + m22*in.y + m23*in.z;
		double z = m31*in.x + m32*in.y + m33*in.z;		
		return Vec3(x, y, z);
	};
	

};

#endif