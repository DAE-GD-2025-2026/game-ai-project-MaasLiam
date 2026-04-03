#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	const auto& Edges = pNavPoly->GetEdges();
	const auto& Triangles = pNavPoly->GetTriangles();

	for (int edgeIdx = 0; edgeIdx < static_cast<int>(Edges.size()); ++edgeIdx)
	{
		const TriPolygon::Edge& edge = Edges[edgeIdx];

		int connectedTriangleCount = 0;
		for (const TriPolygon::Triangle& triangle : Triangles)
		{
			if (triangle.HasEdge(edge))
			{
				++connectedTriangleCount;
			}
		}
		if (connectedTriangleCount >= 2)
		{
			const FVector p1 = edge.GetP1(*pNavPoly);
			const FVector p2 = edge.GetP2(*pNavPoly);
			const FVector2D middle = FVector2D{(p1 + p2) * 0.5f};

			AddNode(std::make_unique<NavGraphNode>(middle, edgeIdx));
		}
	}
	for (const TriPolygon::Triangle& triangle : Triangles)
	{
		std::vector<int> nodeIds{};

		for (const TriPolygon::Edge& edge : triangle.GetEdges())
		{
			const int edgeIdx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
			const int nodeId = GetNodeIdFromEdgeIndex(edgeIdx);

			if (nodeId != Graphs::InvalidNodeId)
			{
				nodeIds.push_back(nodeId);
			}
		}

		if (nodeIds.size() == 2)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
		}
		else if (nodeIds.size() == 3)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[2]);
			AddConnection(nodeIds[2], nodeIds[0]);
		}
	}
	SetConnectionCostsToDistances();
}