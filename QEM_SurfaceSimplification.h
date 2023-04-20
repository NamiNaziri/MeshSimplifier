#pragma once
#include <array>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include "base/Math.hpp"


class QEM_SurfaceSimplification
{
public:
	QEM_SurfaceSimplification(
		std::vector<std::array<unsigned, 6>> faces,
		std::vector<FW::Vec3f> positions,
		std::vector<FW::Vec3f> normals
	);
	QEM_SurfaceSimplification();
	void Simplify(int itrNum);

	// sets of valid pairs 
	std::vector<std::pair<unsigned, unsigned>> validPairs;

public:
	void SetVariables(std::vector<std::array<unsigned, 6>> faces,
		std::vector<FW::Vec3f> positions,
		std::vector<FW::Vec3f> normals);
	void ComputeQMatrix();
	void ComputePlaneArray();

	void ComputeValidPairs();

	// computes V.transpose * Q * V
	float ComputeSingleCost(FW::Vec4f v, FW::Mat4f Q);

	void ComputeCostOfContractionTarget();

	// computes a plane using 3 points and return a vec4 of [a,b,c,d] which is ax + by + cz + d = 0
	FW::Vec4f ComputeSinglePlane(FW::Vec3f p1, FW::Vec3f p2, FW::Vec3f p3);

	FW::Vec4f FindNewLocation(unsigned v1, unsigned v2, FW::Mat4f Q);


	std::vector<std::array<unsigned, 6>> faces;
	std::vector<FW::Vec3f> positions, normals;

	std::vector<FW::Mat4f> Q;
	std::vector<FW::Vec4f> plane;
	std::vector<FW::Mat4f> Kp;

	// threshold for find pairs (this is the threshold squered)
	float hSqr;

	// a heap based on cost. the first float is the cost the the second pair is the valid pair 
	std::vector<std::pair<float, std::pair<unsigned, unsigned>>> costHeap;

	 // // a map from vertex index to the index of vertices which are neighbour with this
	std::map<unsigned, std::set<unsigned>> neighbours;
};


struct Comp
{

	bool operator()(const std::pair<float, std::pair<unsigned, unsigned>>& s1,
		const std::pair<float, std::pair<unsigned, unsigned>>& s2)
	{
		return s1.first > s2.first;
	}
};

struct custom_comparator {
	bool operator()(const std::pair<int, int>& a,
		const std::pair<int, int>& b) const
	{
		return less_comparator(std::minmax(a.first, a.second),
			std::minmax(b.first, b.second));
	}

	std::less<std::pair<int, int>> less_comparator;
};