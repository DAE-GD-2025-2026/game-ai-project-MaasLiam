#include "BFS.h"

#include <map>
#include <queue>
#include <algorithm>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path{};

	if (pGraph == nullptr || pStartNode == nullptr || pDestinationNode == nullptr)
	{
		return path;
	}

	if (pStartNode == pDestinationNode)
	{
		path.push_back(pStartNode);
		return path;
	}

	std::queue<Node*> openQueue{};
	std::map<int, int> parentByNodeId{};
	std::map<int, bool> visited{};

	openQueue.push(pStartNode);
	visited[pStartNode->GetId()] = true;

	bool foundGoal = false;

	while (!openQueue.empty())
	{
		Node* const pCurrentNode = openQueue.front();
		openQueue.pop();

		if (pCurrentNode == pDestinationNode)
		{
			foundGoal = true;
			break;
		}

		for (Connection* const pConnection : pGraph->FindConnectionsFrom(pCurrentNode->GetId()))
		{
			Node* const pNextNode = pGraph->GetNode(pConnection->GetToId()).get();

			if (pNextNode == nullptr || visited[pNextNode->GetId()])
			{
				continue;
			}

			visited[pNextNode->GetId()] = true;
			parentByNodeId[pNextNode->GetId()] = pCurrentNode->GetId();
			openQueue.push(pNextNode);
		}
	}

	if (!foundGoal)
	{
		return {};
	}

	Node* pCurrentNode = pDestinationNode;

	while (pCurrentNode != nullptr && pCurrentNode != pStartNode)
	{
		path.push_back(pCurrentNode);

		auto parentIt = parentByNodeId.find(pCurrentNode->GetId());

		if (parentIt == parentByNodeId.end())
		{
			return {};
		}

		pCurrentNode = pGraph->GetNode(parentIt->second).get();
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}