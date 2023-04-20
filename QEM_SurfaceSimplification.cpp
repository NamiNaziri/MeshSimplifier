#include "QEM_SurfaceSimplification.h"

#include <algorithm>
#include <exdisp.h>
#include <iostream>
#include "src/base/utility.hpp"

QEM_SurfaceSimplification::QEM_SurfaceSimplification(
	std::vector<std::array<unsigned, 6>> faces,
	std::vector<FW::Vec3f> positions,
	std::vector<FW::Vec3f> normals)
{
	this->faces = faces;
	this->positions = positions;
	this->normals = normals;

	this->Q.resize(positions.size(), FW::Mat4f(0));
	this->plane.resize(faces.size());
	this->Kp.resize(faces.size());
	hSqr = 0.05 * 0.05;

	ComputePlaneArray();

	//Compute the Q matrices for all the initial vertices.
	ComputeQMatrix();

	//Select all valid pairs
	ComputeValidPairs();

	/*
	 Compute the optimal contraction target v� for each valid pair
	(v1, v2).The error v�T(Q1 + Q2)v� of this target vertex becomes
		the cost of contracting that pair
	*/
	ComputeCostOfContractionTarget();
}


void QEM_SurfaceSimplification::SetVariables(std::vector<std::array<unsigned, 6>> faces, std::vector<FW::Vec3f> positions, std::vector<FW::Vec3f> normals)
{
	this->faces = faces;
	this->positions = positions;
	this->normals = normals;

	this->Q.resize(positions.size(), FW::Mat4f(0));
	this->plane.resize(faces.size());
	this->Kp.resize(faces.size(), FW::Mat4f(0));
	hSqr = 0.01 * 0.01;

	ComputePlaneArray();

	//Compute the Q matrices for all the initial vertices.
	ComputeQMatrix();

	//Select all valid pairs
	ComputeValidPairs();

	/*
	 Compute the optimal contraction target v� for each valid pair
	(v1, v2).The error v�T(Q1 + Q2)v� of this target vertex becomes
		the cost of contracting that pair
	*/
	ComputeCostOfContractionTarget();
}

QEM_SurfaceSimplification::QEM_SurfaceSimplification()
{
}

void QEM_SurfaceSimplification::Simplify(int itrNum)
{
	//Steps
	if (faces.size() < itrNum)
	{
		std::cout << "Number of faces is less than requested number" << std::endl;
		return;
	}

	std::cout << "Curent Count of Faces " << faces.size() << std::endl;
	std::cout << "Removing " << faces.size() - itrNum << " Faces" << std::endl;
	std::cout << "Simplifying " << std::endl;

	while (faces.size() > itrNum)
	{
		//  Place all the pairs in a heap keyed on cost with the minimum cost pair at the top.
		std::pair<float, std::pair<unsigned, unsigned>> leastCost = costHeap.front();
		std::pair<unsigned, unsigned> leastCostVertex = leastCost.second;
		FW::Mat4f newQ = Q[leastCostVertex.first] + Q[leastCostVertex.second];
		Q[leastCostVertex.first] = newQ;

		// erase the pair that is contracted from validPairs.
		for (int i = 0; i < validPairs.size(); i++)
		{
			if (validPairs[i].first == leastCostVertex.first && validPairs[i].second == leastCostVertex.second)
			{
				validPairs.erase(std::next(validPairs.begin(), i));
				break; 
			}
		}

		// also erase it from costHeap; note that this pair is now in the front. so just pop the first one
		costHeap.erase(costHeap.begin());

		//change location of v1
		FW::Vec3f newPosition = (FindNewLocation(leastCostVertex.first, leastCostVertex.second, newQ)).toCartesian();
		this->positions[leastCostVertex.first] = newPosition;

		// replace all the v2 with new v1 in costHeap; 
		for (auto& [cost, vp] : costHeap)
		{
			if (vp.first == leastCostVertex.second)
			{
				vp.first = leastCostVertex.first;
			}
			else if (vp.second == leastCostVertex.second)
			{
				vp.second = leastCostVertex.first;
			}
		}

		//update cost
		for (auto& [cost, validPair] : costHeap)
		{
			if (validPair.first == leastCostVertex.first || validPair.second == leastCostVertex.first)
			{
				FW::Mat4f Qbar = Q[validPair.first] + Q[validPair.second];
				FW::Vec4f newPosition = FindNewLocation(validPair.first, validPair.second, Qbar);
				cost = ComputeSingleCost(newPosition, Qbar);

			}
		}
		std::make_heap(costHeap.begin(), costHeap.end(), Comp());

		//update faces
		for (auto& face : faces)
		{
			if (face[0] == leastCostVertex.second)
			{
				face[0] = leastCostVertex.first;
			}
			else if (face[2] == leastCostVertex.second)
			{
				face[2] = leastCostVertex.first;
			}
			else if (face[4] == leastCostVertex.second)
			{
				face[4] = leastCostVertex.first;
			}
		}
		for (std::vector<std::array<unsigned, 6>>::iterator it = faces.begin(); it != faces.end();)
		{
			auto f = (*it);

			if (f[0] == f[2] || f[0] == f[4] || f[2] == f[4])
			{
				//std::cout << "Deleted Face" << std::endl;
				it = faces.erase(it);
			}

			else
				++it;
		}



	}
	std::cout << "Simplifying Ended" << std::endl;
}

float QEM_SurfaceSimplification::ComputeSingleCost(FW::Vec4f v, FW::Mat4f Q)
{
	FW::Vec4f temp(
		FW::dot(v, Q.getCol(0)),
		FW::dot(v, Q.getCol(1)),
		FW::dot(v, Q.getCol(2)),
		FW::dot(v, Q.getCol(3))
	);

	return FW::dot(temp, v);
}


void QEM_SurfaceSimplification::ComputeQMatrix()
{

	for (int i = 0; i < faces.size(); i++)
	{

		Q[faces[i][0]] += Kp[i];
		Q[faces[i][2]] += Kp[i];
		Q[faces[i][4]] += Kp[i];
	}
}


void QEM_SurfaceSimplification::ComputePlaneArray()
{
	for (int i = 0; i < faces.size(); i++)
	{
		std::array<unsigned, 6> f = faces[i];
		FW::Vec4f p = ComputeSinglePlane(positions[f[0]], positions[f[2]], positions[f[4]]);
		plane[i] = p;

		FW::Mat4f Kp;
		Kp.setRow(0, FW::Vec4f(p[0] * p[0], p[0] * p[1], p[0] * p[2], p[0] * p[3]));
		Kp.setRow(1, FW::Vec4f(p[0] * p[1], p[1] * p[1], p[1] * p[2], p[1] * p[3]));
		Kp.setRow(2, FW::Vec4f(p[0] * p[2], p[1] * p[2], p[2] * p[2], p[2] * p[3]));
		Kp.setRow(3, FW::Vec4f(p[0] * p[3], p[1] * p[3], p[2] * p[3], p[3] * p[3]));

		this->Kp[i] = Kp;
	}
}


FW::Vec4f QEM_SurfaceSimplification::ComputeSinglePlane(FW::Vec3f p1, FW::Vec3f p2, FW::Vec3f p3)
{
	const FW::Vec3f normal = FW::cross((p2 - p1), (p3 - p1)).normalized();
	const float d = -1 * (FW::dot(p1, normal));
	return FW::Vec4f(normal, d);
}

FW::Vec4f QEM_SurfaceSimplification::FindNewLocation(unsigned v1, unsigned v2, FW::Mat4f Q)
{
	FW::Mat4f newQ;
	newQ.setRow(0, FW::Vec4f(Q(0, 0), Q(0, 1), Q(0, 2), Q(0, 3)));
	newQ.setRow(1, FW::Vec4f(Q(0, 1), Q(1, 1), Q(1, 2), Q(1, 3)));
	newQ.setRow(2, FW::Vec4f(Q(0, 2), Q(1, 2), Q(2, 2), Q(2, 3)));
	newQ.setRow(3, FW::Vec4f(0, 0, 0, 1));
	FW::Vec4f newPosition;
	if (newQ.det() == 0.f) // matrix is not invertible
	{
		newPosition = ((this->positions[v1] + this->positions[v2]) / 2).toHomogeneous();
	}
	else
	{
		newPosition = newQ.inverted() * FW::Vec4f(0, 0, 0, 1);
	}
	newPosition = ((this->positions[v1] + this->positions[v2]) / 2).toHomogeneous();
	return newPosition;
}


void QEM_SurfaceSimplification::ComputeValidPairs()
{
	for (auto& face : faces)
	{
		validPairs.emplace_back(face[0], face[2]);
		validPairs.emplace_back(face[2], face[4]);
		validPairs.emplace_back(face[4], face[0]);
	}
	int k = 0;
	for (int i = 0; i < positions.size(); i++)
	{
		for (int j = i + 1; j < positions.size(); j++)
		{
			FW::Vec3f dis = (positions[i] - positions[j]);
			if (dis.lenSqr() < hSqr)
			{
				k++;
				validPairs.emplace_back(i, j);
			}
		}
	}
	std::cout << "number of pairs added using threshold: " << k << std::endl;
	// Specify custom comparator for the set
	std::set<std::pair<int, int>, custom_comparator> unique;

	// Fill the set
	for (const auto& p : validPairs) {
		unique.insert(p);
	}

	validPairs.clear();
	validPairs = std::vector<std::pair<unsigned, unsigned>>(unique.begin(), unique.end());
}

void QEM_SurfaceSimplification::ComputeCostOfContractionTarget()
{
	for (auto validPair : validPairs)
	{
		FW::Mat4f Qbar = Q[validPair.first] + Q[validPair.second];
		FW::Vec4f newPosition = FindNewLocation(validPair.first, validPair.second, Qbar);
		float cost = ComputeSingleCost(newPosition, Qbar);
		costHeap.emplace_back(cost, validPair);
	}
	std::make_heap(costHeap.begin(), costHeap.end(), Comp());
}
