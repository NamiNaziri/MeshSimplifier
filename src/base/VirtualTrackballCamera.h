#pragma once
#include "base/Math.hpp"



struct Hemisphere
{
	Hemisphere(float width, float height) :
	width(width),
	height(height)
	{
		
	}
	float width, height;
};

class VirtualTrackballCamera
{
public:
	VirtualTrackballCamera();
	VirtualTrackballCamera(Hemisphere h, FW::Vec2f mouse_pose);
	const FW::Mat4f& GetCameraTransform();
	void UpdateCamera(FW::Vec2f new_mouse_pose);
	void ResetCamera(FW::Vec2f mouse_pose);

private:
	FW::Mat4f camera_transformation_;
	FW::Mat3f camera_transformation_2;
	FW::Vec3f last_projected_position_;
	FW::Vec2f mlast;
	Hemisphere h_= {1.f,1.f};
private:
	FW::Vec3f ProjectToHemisphere(FW::Vec2f mouse_pos);

};

