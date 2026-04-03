#include "SpacePartitioning.h"
#include <algorithm>
#include "DrawDebugHelpers.h"

// --- Cell ---
// ------------
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);

	//calculate bounds of a cell
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;
	
	CellOrigin = FVector2D(-SpaceWidth * 0.5f, -SpaceHeight * 0.5f);
	
	Cells.reserve(NrOfRows * NrOfCols);
	for (int r = 0; r < NrOfRows; ++r)
	{
		for (int c = 0; c < NrOfCols; ++c)
		{
			const float left = CellOrigin.X + c * CellWidth;
			const float bottom = CellOrigin.Y + r * CellHeight;
			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	const int idx = PositionToIndex(Agent.GetPosition());
	Cells[idx].Agents.push_back(&Agent);
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	const int oldIdx = PositionToIndex(OldPos);
	const int newIdx = PositionToIndex(Agent.GetPosition());

	if (oldIdx == newIdx)
		return;

	Cells[oldIdx].Agents.remove(&Agent);
	
	Cells[newIdx].Agents.push_back(&Agent);
}

void CellSpace::RegisterNeighbors(ASteeringAgent& Agent, float QueryRadius)
{
	NrOfNeighbors = 0;

	const FVector2D agentPos = Agent.GetPosition();
	const float r2 = QueryRadius * QueryRadius;
	
	FRect queryRect;
	queryRect.Min = { agentPos.X - QueryRadius, agentPos.Y - QueryRadius };
	queryRect.Max = { agentPos.X + QueryRadius, agentPos.Y + QueryRadius };
	
	auto ToRowCol = [this](const FVector2D& p, int& outRow, int& outCol)
	{
		const float x = p.X - CellOrigin.X;
		const float y = p.Y - CellOrigin.Y;

		outCol = static_cast<int>(x / CellWidth);
		outRow = static_cast<int>(y / CellHeight);

		outCol = FMath::Clamp(outCol, 0, NrOfCols - 1);
		outRow = FMath::Clamp(outRow, 0, NrOfRows - 1);
	};

	int minRow{}, minCol{}, maxRow{}, maxCol{};
	ToRowCol(queryRect.Min, minRow, minCol);
	ToRowCol(queryRect.Max, maxRow, maxCol);

	for (int r = minRow; r <= maxRow; ++r)
	{
		for (int c = minCol; c <= maxCol; ++c)
		{
			Cell& cell = Cells[r * NrOfCols + c];
			
			if (!DoRectsOverlap(cell.BoundingBox, queryRect))
				continue;

			for (ASteeringAgent* other : cell.Agents)
			{
				if (!IsValid(other)) continue;
				if (other == &Agent) continue;

				const FVector2D d = other->GetPosition() - agentPos;
				if (d.SquaredLength() <= r2)
				{
					Neighbors[NrOfNeighbors] = other;
					++NrOfNeighbors;
				}
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	for (const Cell& cell : Cells)
	{
		const auto pts = cell.GetRectPoints();
		for (int i = 0; i < 4; ++i)
		{
			const FVector2D a = pts[i];
			const FVector2D b = pts[(i + 1) % 4];
			DrawDebugLine(
				pWorld,
				FVector(a.X, a.Y, 80.f),
				FVector(b.X, b.Y, 80.f),
				FColor::Cyan,
				false,
				0.f,
				0,
				1.f
			);
		}

		const FVector2D center = (cell.BoundingBox.Min + cell.BoundingBox.Max) * 0.5f;
		const int count = static_cast<int>(cell.Agents.size());

		DrawDebugString(
			pWorld,
			FVector(center.X, center.Y, 100.f),
			FString::FromInt(count),
			nullptr,
			FColor::White,
			0.f,
			true
		);
	}
}

int CellSpace::PositionToIndex(FVector2D const& Pos) const
{
	const float x = Pos.X - CellOrigin.X;
	const float y = Pos.Y - CellOrigin.Y;

	int col = static_cast<int>(x / CellWidth);
	int row = static_cast<int>(y / CellHeight);

	col = FMath::Clamp(col, 0, NrOfCols - 1);
	row = FMath::Clamp(row, 0, NrOfRows - 1);

	return row * NrOfCols + col;
}

bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}
