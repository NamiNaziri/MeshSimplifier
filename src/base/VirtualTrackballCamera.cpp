#include "VirtualTrackballCamera.h"
#include "src/base/utility.hpp"
#define M_PI 3.14159

VirtualTrackballCamera::VirtualTrackballCamera(Hemisphere h) :
	h_(h)
{
}

const FW::Mat4f& VirtualTrackballCamera::GetCameraTransform()
{
	return camera_transformation_;
}

void VirtualTrackballCamera::UpdateCamera(FW::Vec2f new_mouse_pose)
{
	
	const FW::Vec3f new_projected_position = ProjectToHemisphere(new_mouse_pose);
	const FW::Vec3f delta_pos = new_projected_position - last_projected_position_;

	if(delta_pos.x || delta_pos.y || delta_pos.z) //todo ?????
	{
		float angle = 90.0 * FW::length(delta_pos);
		FW::Vec3f axis = FW::cross(last_projected_position_, new_projected_position);
		camera_transformation_ = CustomMath::ToMat4(FW::Mat3f::rotation(axis, angle));
	}
	
	//camera_transformation_ = Mat3f
}

FW::Vec3f VirtualTrackballCamera::ProjectToHemisphere(const FW::Vec2f& mouse_pos)
{
	float d; //, a;
	FW::Vec3f projected_point;

	projected_point.x = (2 * mouse_pos.x - h_.width) / h_.width;
	projected_point.z = (h_.height - 2.0 * mouse_pos.y)/h_.height;
	d = FW::length(mouse_pos);
	projected_point.y = cos((M_PI / 2.f) * ((d < 1.0) ? d : 1.0));
	//a = 1.0 / FW::length(projected_point);
	return projected_point.normalized();
}
