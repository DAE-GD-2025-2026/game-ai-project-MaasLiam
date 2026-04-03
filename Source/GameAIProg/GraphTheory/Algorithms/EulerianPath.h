#pragma once
#include <stack>
#include "Shared/Graph/Graph.h"

namespace GameAI
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	class EulerianPath final
	{
	public:
		EulerianPath(Graph* const pGraph);

		Eulerianity IsEulerian() const;
		std::vector<Node*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(const std::vector<Node*>& pNodes, std::vector<bool>& visited, int startIndex) const;
		bool IsConnected() const;

		Graph* m_pGraph;
	};

	inline EulerianPath::EulerianPath(Graph* const pGraph)
		: m_pGraph(pGraph)
	{
	}

	inline Eulerianity EulerianPath::IsEulerian() const
	{
		if (!IsConnected())
		{
			return Eulerianity::notEulerian;
		}

		int oddDegreeNodeCount = 0;
		for (Node* node : m_pGraph->GetActiveNodes())
		{
			const int degree = static_cast<int>(m_pGraph->FindConnectionsFrom(node->GetId()).size());
			if (degree % 2 != 0)
			{
				++oddDegreeNodeCount;
			}
		}
		
		if (oddDegreeNodeCount > 2)
		{
			return Eulerianity::notEulerian;
		}
		
		if (oddDegreeNodeCount == 2)
		{
			return Eulerianity::semiEulerian;
		}
		
		if (oddDegreeNodeCount == 0)
		{
			return Eulerianity::eulerian;
		}

		return Eulerianity::notEulerian;
	}


	inline std::vector<Node*> EulerianPath::FindPath(Eulerianity& eulerianity) const
	{
		// Get a copy of the graph because this algorithm involves removing edges
		Graph graphCopy = m_pGraph->Clone();
		std::vector<Node*> Path = {};
		std::vector<Node*> Nodes = graphCopy.GetActiveNodes();
		int currentNodeId{ Graphs::InvalidNodeId };
		
		eulerianity = IsEulerian();
		if (eulerianity == Eulerianity::notEulerian || Nodes.empty())
		{
			return Path;
		}
		
		if (eulerianity == Eulerianity::semiEulerian)
		{
			for (Node* node : Nodes)
			{
				const int degree = static_cast<int>(graphCopy.FindConnectionsFrom(node->GetId()).size());
				if (degree % 2 != 0)
				{
					currentNodeId = node->GetId();
					break;
				}
			}
		}
		else
		{
			currentNodeId = Nodes[0]->GetId();
		}
		
		std::stack<int> nodeStack;

		while (currentNodeId != Graphs::InvalidNodeId &&
			   (!nodeStack.empty() || !graphCopy.FindConnectionsFrom(currentNodeId).empty()))
		{
			std::vector<Connection*> connections = graphCopy.FindConnectionsFrom(currentNodeId);

			if (!connections.empty())
			{
				nodeStack.push(currentNodeId);

				Connection* chosenConnection = connections[0];
				const int nextNodeId = chosenConnection->GetToId();

				graphCopy.RemoveConnection(chosenConnection);
				currentNodeId = nextNodeId;
			}
			else
			{
				Path.push_back(m_pGraph->GetNode(currentNodeId).get());
				currentNodeId = nodeStack.top();
				nodeStack.pop();
			}
		}
		
		if (currentNodeId != Graphs::InvalidNodeId)
		{
			Path.push_back(m_pGraph->GetNode(currentNodeId).get());
		}

		std::reverse(Path.begin(), Path.end());
		return Path;
	}

	inline void EulerianPath::VisitAllNodesDFS(const std::vector<Node*>& Nodes, std::vector<bool>& visited, int startIndex) const
	{
		visited[startIndex] = true;

		const int currentNodeId = Nodes[startIndex]->GetId();
		std::vector<Connection*> connections = m_pGraph->FindConnectionsFrom(currentNodeId);

		for (Connection* connection : connections)
		{
			const int connectedNodeId = connection->GetToId();

			for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
			{
				if (Nodes[i]->GetId() == connectedNodeId && !visited[i])
				{
					VisitAllNodesDFS(Nodes, visited, i);
					break;
				}
			}
		}
	}

	inline bool EulerianPath::IsConnected() const
	{
		std::vector<Node*> Nodes = m_pGraph->GetActiveNodes();
		if (Nodes.size() == 0)
			return false;
		
		int startIndex = 0;
		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			if (!m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty())
			{
				startIndex = i;
				break;
			}
		}

		std::vector<bool> visited(Nodes.size(), false);
		
		VisitAllNodesDFS(Nodes, visited, startIndex);

		for (int i = 0; i < static_cast<int>(Nodes.size()); ++i)
		{
			const bool hasConnections = !m_pGraph->FindConnectionsFrom(Nodes[i]->GetId()).empty();
			if (hasConnections && !visited[i])
			{
				return false;
			}
		}

		return true;
	}
}