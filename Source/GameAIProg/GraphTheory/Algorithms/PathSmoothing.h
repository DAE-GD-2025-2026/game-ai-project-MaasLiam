#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		
		//For each node received, get it's corresponding line
		
			//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point

			//Store portal

		//Add degenerate portal to force end evaluation

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals(std::vector<NavLine> const& Portals, TriPolygon const& NavPoly)
{
	std::vector<FVector2D> Path{};

	if (Portals.empty())
		return Path;

	FVector2D apexPoint = Portals[0].P1;
	FVector2D leftLeg = Portals[0].P2 - apexPoint;
	FVector2D rightLeg = Portals[0].P1 - apexPoint;

	int apexIndex = 0;
	int leftLegIndex = 0;
	int rightLegIndex = 0;

	Path.push_back(apexPoint);

	for (int portalIdx = 1; portalIdx < static_cast<int>(Portals.size()); ++portalIdx)
	{
		const NavLine& portal = Portals[portalIdx];

		// --- RIGHT CHECK ---
		FVector2D newRightLeg = portal.P1 - apexPoint;
		
		if (FVector2D::CrossProduct(rightLeg, newRightLeg) <= 0.f)
		{
			if (FVector2D::CrossProduct(leftLeg, newRightLeg) > 0.f)
			{
				rightLeg = newRightLeg;
				rightLegIndex = portalIdx;
			}
			else
			{
				apexPoint = apexPoint + leftLeg;
				apexIndex = leftLegIndex;
				
				portalIdx = leftLegIndex + 1;
				leftLegIndex = portalIdx;
				rightLegIndex = portalIdx;

				Path.push_back(apexPoint);

				if (portalIdx < static_cast<int>(Portals.size()))
				{
					leftLeg = Portals[portalIdx].P2 - apexPoint;
					rightLeg = Portals[portalIdx].P1 - apexPoint;
				}

				--portalIdx;
				continue;
			}
		}

		// --- LEFT CHECK ---
		FVector2D newLeftLeg = portal.P2 - apexPoint;
		
		if (FVector2D::CrossProduct(leftLeg, newLeftLeg) >= 0.f)
		{
			if (FVector2D::CrossProduct(rightLeg, newLeftLeg) < 0.f)
			{
				leftLeg = newLeftLeg;
				leftLegIndex = portalIdx;
			}
			else
			{
				apexPoint = apexPoint + rightLeg;
				apexIndex = rightLegIndex;
				
				portalIdx = rightLegIndex + 1;
				leftLegIndex = portalIdx;
				rightLegIndex = portalIdx;

				Path.push_back(apexPoint);

				if (portalIdx < static_cast<int>(Portals.size()))
				{
					leftLeg = Portals[portalIdx].P2 - apexPoint;
					rightLeg = Portals[portalIdx].P1 - apexPoint;
				}

				--portalIdx;
				continue;
			}
		}
	}
	Path.push_back(Portals.back().P1);

	return Path;
}
private:
	SSFA() {};
	~SSFA() {};
};
}
