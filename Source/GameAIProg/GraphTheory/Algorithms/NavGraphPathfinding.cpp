
#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	if (pNavGraph == nullptr || pNavGraph->GetNavPolygon() == nullptr)
	{
		return finalPath;
	}

	//Get the start and endTriangle
	const TriPolygon* pNavPoly = pNavGraph->GetNavPolygon();
	const TriPolygon::Triangle* startTriangle = pNavPoly->GetTriangleAtPosition(startPos, true);
	const TriPolygon::Triangle* endTriangle = pNavPoly->GetTriangleAtPosition(endPos, true);
	
	if (startTriangle == nullptr || endTriangle == nullptr)
	{
		return finalPath;
	}
	
	if (*startTriangle == *endTriangle)
	{
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}


	//We have valid start/end triangles and they are not the same
	//=> Start looking for a path
	//Copy the graph
	std::unique_ptr<NavGraph> pGraphCopy = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position
	const int startNodeId = pGraphCopy->AddNode(std::make_unique<NavGraphNode>(startPos, -1));

	for (const TriPolygon::Edge& edge : startTriangle->GetEdges())
	{
		const int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
		const int nodeId = pGraphCopy->GetNodeIdFromEdgeIndex(edgeIdx);

		if (nodeId != Graphs::InvalidNodeId)
		{
			pGraphCopy->AddConnection(startNodeId, nodeId);

			const float cost =
				(pGraphCopy->GetNode(startNodeId)->GetPosition() -
				 pGraphCopy->GetNode(nodeId)->GetPosition()).Length();

			if (Connection* pConnection = pGraphCopy->FindConnection(startNodeId, nodeId))
			{
				pConnection->SetWeight(cost);
			}
			if (Connection* pConnection = pGraphCopy->FindConnection(nodeId, startNodeId))
			{
				pConnection->SetWeight(cost);
			}
		}
	}
	//Create extra node for the endNode
	const int endNodeId = pGraphCopy->AddNode(std::make_unique<NavGraphNode>(endPos, -1));

	for (const TriPolygon::Edge& edge : endTriangle->GetEdges())
	{
		const int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
		const int nodeId = pGraphCopy->GetNodeIdFromEdgeIndex(edgeIdx);

		if (nodeId != Graphs::InvalidNodeId)
		{
			pGraphCopy->AddConnection(endNodeId, nodeId);

			const float cost =
				(pGraphCopy->GetNode(endNodeId)->GetPosition() -
				 pGraphCopy->GetNode(nodeId)->GetPosition()).Length();

			if (Connection* pConnection = pGraphCopy->FindConnection(endNodeId, nodeId))
			{
				pConnection->SetWeight(cost);
			}
			if (Connection* pConnection = pGraphCopy->FindConnection(nodeId, endNodeId))
			{
				pConnection->SetWeight(cost);
			}
		}
	}

	//Run A star on new graph
	AStar aStar{ pGraphCopy.get(), HeuristicFunctions::Euclidean };
	std::vector<Node*> nodePath =
		aStar.FindPath(pGraphCopy->GetNode(startNodeId).get(), pGraphCopy->GetNode(endNodeId).get());

	//Debug Visualisation
	for (Node* pNode : nodePath)
	{
		debugNodePositions.push_back(pNode->GetPosition());
		finalPath.push_back(pNode->GetPosition());
	}

	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	// debugPortals = SSFA::FindPortals(nodes, *pNavGraph->GetNavPolygon());
	// finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}