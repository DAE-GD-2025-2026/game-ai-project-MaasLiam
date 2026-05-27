#pragma once
#include "Shared/Graph/Graph.h"
#include <map>
#include <set>


namespace GameAI
{
	class GraphColoring final
	{
	public:
		explicit GraphColoring(Graph* const pGraph)
			: m_pGraph(pGraph)
		{
		}

		std::vector<std::pair<int, FColor>> ColorGraph() const
		{
			std::vector<std::pair<int, FColor>> Result{};
			if (!m_pGraph) return Result;

			const std::vector<FColor> Colors =
			{
				FColor::Red,
				FColor::Green,
				FColor::Blue,
				FColor::Yellow,
				FColor::Cyan,
				FColor::Magenta,
				FColor::Orange,
				FColor::Purple
			};

			std::map<int, int> NodeToColorIndex;

			for (Node* Node : m_pGraph->GetActiveNodes())
			{
				std::set<int> UsedNeighbourColors;

				for (Connection* Connection : m_pGraph->FindConnectionsWith(Node->GetId()))
				{
					int OtherNodeId =
						Connection->GetFromId() == Node->GetId()
						? Connection->GetToId()
						: Connection->GetFromId();

					if (NodeToColorIndex.contains(OtherNodeId))
					{
						UsedNeighbourColors.insert(NodeToColorIndex[OtherNodeId]);
					}
				}

				int ChosenColor = 0;
				while (UsedNeighbourColors.contains(ChosenColor))
				{
					++ChosenColor;
				}

				NodeToColorIndex[Node->GetId()] = ChosenColor;
				Result.emplace_back(Node->GetId(), Colors[ChosenColor % Colors.size()]);
			}

			return Result;
		}

	private:
		Graph* m_pGraph;
	};
}