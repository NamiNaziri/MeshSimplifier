#pragma once
#include <iostream>   
#include <string>  
// http://stackoverflow.com/questions/2270726/how-to-determine-the-size-of-an-array-of-strings-in-c
template <typename T, size_t N>
char (&static_sizeof_array( T(&)[N] ))[N]; // declared, not defined
#define SIZEOF_ARRAY( x ) sizeof(static_sizeof_array(x))
#define M_PI 3.14159

namespace CustomMath
{
	inline FW::Mat4f Rotation(const FW::Vec3f& axis, FW::F32 angle)
	{
		FW::Mat4f R;
		FW::F32 cosa = cosf(angle);
		FW::F32 sina = sinf(angle);
		R(0, 0) = cosa + FW::sqr(axis.x) * (1.0f - cosa);			R(0, 1) = axis.x * axis.y * (1.0f - cosa) - axis.z * sina;	R(0, 2) = axis.x * axis.z * (1.0f - cosa) + axis.y * sina; R(0, 3) = 0;
		R(1, 0) = axis.x * axis.y * (1.0f - cosa) + axis.z * sina;	R(1, 1) = cosa + FW::sqr(axis.y) * (1.0f - cosa);			R(1, 2) = axis.y * axis.z * (1.0f - cosa) - axis.x * sina; R(1, 3) = 0;
		R(2, 0) = axis.z * axis.x * (1.0f - cosa) - axis.y * sina;	R(2, 1) = axis.z * axis.y * (1.0f - cosa) + axis.x * sina;	R(2, 2) = cosa + FW::sqr(axis.z) * (1.0f - cosa); R(1, 3) = 0;
		R.setRow(3, FW::Vec4f(0.f, 0.f, 0.f, 1.f));
		return R;

	}

	inline FW::Mat3f FromMat4(const FW::Mat4f& from)
	{
		FW::Mat3f m3f;

		m3f(0, 0) = from(0, 0);	m3f(0, 1) = from(0, 1);   	m3f(0, 2) = from(0, 2);
		m3f(1, 0) = from(1, 0); 	m3f(1, 1) = from(1, 1);	m3f(1, 2) = from(1, 2);
		m3f(2, 0) = from(2, 0);	m3f(2, 1) = from(2, 1);  	m3f(2, 2) = from(2, 2);

		return m3f;
	}

	inline FW::Mat4f ToMat4(const FW::Mat3f& from)
	{
		FW::Mat4f C;

		C.setCol(0, FW::Vec4f(from.getCol(0), 0));
		C.setCol(1, FW::Vec4f(from.getCol(1), 0));
		C.setCol(2, FW::Vec4f(from.getCol(2), 0));
		C.setCol(3, FW::Vec4f(0, 0, 0, 1));

		return C;
	}

	inline float ToRadian(const float& angle_in_degree)
	{
		return (angle_in_degree / 180.f)* M_PI;
	}
}


namespace DebugHelpper
{
	inline std::string GetString(const FW::Vec2f& in)
	{
		return " (" + std::to_string(in.x) + ", " + std::to_string(in.y) + ") ";
	}

	inline std::string GetString(const FW::Vec3f& in)
	{
		return " (" + std::to_string(in.x) + ", " + std::to_string(in.y) + ", " + std::to_string(in.z) + ") ";
	}
}