
#include "src/base/VirtualTrackballCamera.h"
#include "src/base/utility.hpp"
#include <iostream>
#define M_PI 3.14159

VirtualTrackballCamera::VirtualTrackballCamera()
{
}

VirtualTrackballCamera::VirtualTrackballCamera(Hemisphere h, FW::Vec2f mouse_pose) :
	h_(h),
	camera_transformation_(FW::Mat4f())
{
	ResetCamera(mouse_pose);
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
		float angle = 90 * FW::length(delta_pos);

		FW::Vec3f axis = FW::cross(last_projected_position_, new_projected_position);
		camera_transformation_ = CustomMath::ToMat4(FW::Mat3f::rotation(axis.normalized(), angle * M_PI / 180.f)) * camera_transformation_;
		last_projected_position_ = new_projected_position;
	}
	
}

void VirtualTrackballCamera::ResetCamera(FW::Vec2f mouse_pose)
{
	last_projected_position_ = ProjectToHemisphere(mouse_pose);
}

FW::Vec3f VirtualTrackballCamera::ProjectToHemisphere(FW::Vec2f mouse_pos)
{
	
	float d; //, a;
	FW::Vec3f projected_point;
	projected_point.x = (2.0f * mouse_pos.x - h_.width) / h_.width;
	projected_point.y = (h_.height - 2.0F * mouse_pos.y)/ h_.height;
	d = FW::sqrt((projected_point.x * projected_point.x) + (projected_point.y * projected_point.y));
	projected_point.z = cos((M_PI / 2.0f) * ((d < 1.0f) ? d : 1.0f));

	return projected_point.normalized();
}
