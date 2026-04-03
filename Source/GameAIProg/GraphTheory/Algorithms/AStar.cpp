#include "AStar.h"

#include <algorithm>

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*> AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};

	if (pGraph == nullptr || pStartNode == nullptr || pGoalNode == nullptr)
	{
		return path;
	}

	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	openList.push_back(startRecord);

	NodeRecord currentNodeRecord{};
	bool foundGoal = false;

	while (!openList.empty())
	{
		auto bestRecordIt = std::min_element(openList.begin(), openList.end(),
			[](NodeRecord const& lhs, NodeRecord const& rhs)
			{
				return lhs.estimatedTotalCost < rhs.estimatedTotalCost;
			});

		currentNodeRecord = *bestRecordIt;

		if (currentNodeRecord.pNode == pGoalNode)
		{
			foundGoal = true;
			break;
		}

		for (Connection* const pConnection : pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId()))
		{
			Node* const pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			float const newCostSoFar = currentNodeRecord.costSoFar + pConnection->GetWeight();

			auto closedIt = std::find_if(closedList.begin(), closedList.end(),
				[pNextNode](NodeRecord const& record)
				{
					return record.pNode == pNextNode;
				});

			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= newCostSoFar)
				{
					continue;
				}

				closedList.erase(closedIt);
			}

			auto openIt = std::find_if(openList.begin(), openList.end(),
				[pNextNode](NodeRecord const& record)
				{
					return record.pNode == pNextNode;
				});

			if (openIt != openList.end())
			{
				if (openIt->costSoFar <= newCostSoFar)
				{
					continue;
				}

				openList.erase(openIt);
			}

			NodeRecord newRecord{};
			newRecord.pNode = pNextNode;
			newRecord.pConnection = pConnection;
			newRecord.costSoFar = newCostSoFar;
			newRecord.estimatedTotalCost = newCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);

			openList.push_back(newRecord);
		}

		auto currentOpenIt = std::find_if(openList.begin(), openList.end(),
			[&currentNodeRecord](NodeRecord const& record)
			{
				return record.pNode == currentNodeRecord.pNode;
			});

		if (currentOpenIt != openList.end())
		{
			openList.erase(currentOpenIt);
		}

		closedList.push_back(currentNodeRecord);
	}

	if (!foundGoal)
	{
		return path;
	}

	while (currentNodeRecord.pNode != pStartNode)
	{
		path.push_back(currentNodeRecord.pNode);

		if (currentNodeRecord.pConnection == nullptr)
		{
			return {};
		}

		int const previousNodeId = currentNodeRecord.pConnection->GetFromId();

		auto previousRecordIt = std::find_if(closedList.begin(), closedList.end(),
			[previousNodeId](NodeRecord const& record)
			{
				return record.pNode != nullptr && record.pNode->GetId() == previousNodeId;
			});

		if (previousRecordIt == closedList.end())
		{
			return {};
		}

		currentNodeRecord = *previousRecordIt;
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination =
		pGraph->GetNode(pEndNode->GetId())->GetPosition() -
		pGraph->GetNode(pStartNode->GetId())->GetPosition();

	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}