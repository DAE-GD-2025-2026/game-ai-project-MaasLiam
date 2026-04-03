#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "Shared/ImGuiHelpers.h"
#include "DrawDebugHelpers.h"

static FVector ToWorld3D(const FVector2D& p, float z = 90.f)
{
	return FVector(p.X, p.Y, z);
}

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const InAgentToEvade,
	bool bTrimWorld)
	: pWorld{ pWorld }
	, FlockSize{ FlockSize }
	, pAgentToEvade{ InAgentToEvade }
{
	Agents.SetNum(FlockSize);

	for (int i = 0; i < FlockSize; ++i)
	{
		const float x = FMath::FRandRange(-WorldSize, WorldSize);
		const float y = FMath::FRandRange(-WorldSize, WorldSize);

		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			FVector{ x, y, 90.f },
			FRotator::ZeroRotator);

		if (IsValid(Agents[i]))
		{
			Agents[i]->SetIsAutoOrienting(true);
			Agents[i]->SetDebugRenderingEnabled(false);
		}
	}
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	OldPositions.SetNum(Agents.Num());
	const float SpaceDim = WorldSize * 2.f;
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		SpaceDim, SpaceDim,
		NrOfCellsY, NrOfCellsX,
		Agents.Num()
	);

	for (int i = 0; i < Agents.Num(); ++i)
	{
		if (!IsValid(Agents[i])) continue;
		OldPositions[i] = Agents[i]->GetPosition();
		pPartitionedSpace->AddAgent(*Agents[i]);
	}
#else
	Neighbors.SetNum(FlockSize);
	NrOfNeighbors = 0;
#endif
	
	if (!IsValid(pAgentToEvade))
	{
		SpawnedEvadeAgent = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass,
			FVector{ 0.f, 0.f, 90.f },
			FRotator::ZeroRotator);

		if (IsValid(SpawnedEvadeAgent))
		{
			SpawnedEvadeAgent->SetIsAutoOrienting(true);
			SpawnedEvadeAgent->SetDebugRenderingEnabled(false);

			pEvadeAgentWander = std::make_unique<Wander>();
			SpawnedEvadeAgent->SetSteeringBehavior(pEvadeAgentWander.get());
		}

		pAgentToEvade = SpawnedEvadeAgent;
	}
	
	pSeparationBehavior = std::make_unique<Separation>(this);
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pFlockWanderBehavior = std::make_unique<Wander>();

	std::vector<BlendedSteering::WeightedBehavior> blended{};
	blended.emplace_back(pSeparationBehavior.get(), 0.40f);
	blended.emplace_back(pCohesionBehavior.get(),   0.25f);
	blended.emplace_back(pVelMatchBehavior.get(),   0.25f);
	blended.emplace_back(pSeekBehavior.get(),       0.05f);
	blended.emplace_back(pFlockWanderBehavior.get(),0.05f);

	pBlendedSteering = std::make_unique<BlendedSteering>(blended);

	// Priority steering
	pEvadeBehavior = std::make_unique<EvadeWithRadius>();
	pEvadeBehavior->SetEvadeRadius(EvadeRadius);

	std::vector<ISteeringBehavior*> priority{};
	priority.push_back(pEvadeBehavior.get());
	priority.push_back(pBlendedSteering.get());
	pPrioritySteering = std::make_unique<PrioritySteering>(priority);

	for (ASteeringAgent* a : Agents)
	{
		if (IsValid(a))
		{
			a->SetSteeringBehavior(pPrioritySteering.get());
			a->SetIsAutoOrienting(true);
		}
	}

	(void)bTrimWorld;
}

Flock::~Flock()
{
	for (ASteeringAgent* agent : Agents)
	{
		if (IsValid(agent))
			agent->Destroy();
	}
	Agents.Empty();

	if (IsValid(SpawnedEvadeAgent))
	{
		SpawnedEvadeAgent->Destroy();
		SpawnedEvadeAgent = nullptr;
	}
}

void Flock::Tick(float DeltaTime)
{
	for (ASteeringAgent* a : Agents)
	{
		if (IsValid(a))
			a->SetDebugRenderingEnabled(DebugRenderSteering);
	}
	if (IsValid(SpawnedEvadeAgent))
		SpawnedEvadeAgent->SetDebugRenderingEnabled(DebugRenderSteering);
	
	if (IsValid(SpawnedEvadeAgent))
	{
		SpawnedEvadeAgent->Tick(DeltaTime);
	}
	
	if (pEvadeBehavior)
	{
		if (IsValid(pAgentToEvade))
		{
			FTargetData target{};
			target.Position = pAgentToEvade->GetPosition();
			target.Orientation = pAgentToEvade->GetRotation();
			target.LinearVelocity = pAgentToEvade->GetLinearVelocity();
			target.AngularVelocity = pAgentToEvade->GetAngularVelocity();

			pEvadeBehavior->SetTarget(target);
			pEvadeBehavior->SetHasTarget(true);
			pEvadeBehavior->SetEvadeRadius(EvadeRadius);
		}
		else
		{
			pEvadeBehavior->SetHasTarget(false);
		}
	}
	
	for (int i = 0; i < Agents.Num(); ++i)
	{
		ASteeringAgent* agent = Agents[i];
		if (!IsValid(agent)) continue;

#ifdef GAMEAI_USE_SPACE_PARTITIONING
		if (pPartitionedSpace)
			pPartitionedSpace->RegisterNeighbors(*agent, NeighborhoodRadius);

		const FVector2D oldPos = OldPositions[i];

		agent->Tick(DeltaTime);

		if (pPartitionedSpace)
			pPartitionedSpace->UpdateAgentCell(*agent, oldPos);

		OldPositions[i] = agent->GetPosition();
#else
		RegisterNeighbors(agent);
		agent->Tick(DeltaTime);
#endif
	}

	if (DebugRenderNeighborhood)
		RenderNeighborhood();
}

void Flock::RenderDebug()
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions && pPartitionedSpace)
		pPartitionedSpace->RenderCells();
#endif

	if (DebugRenderNeighborhood)
		RenderNeighborhood();
	
	if (pWorld && IsValid(pAgentToEvade))
	{
		const FVector2D p2 = pAgentToEvade->GetPosition();
		DrawDebugCircle(
			pWorld,
			ToWorld3D(p2),
			30.f,
			16,
			FColor::Red,
			false,
			0.f,
			0,
			3.f,
			FVector(1, 0, 0),
			FVector(0, 1, 0),
			false
		);
	}
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	{
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		ImGui::Text("Flocking");
		ImGui::Checkbox("Debug Steering", &DebugRenderSteering);
		ImGui::Checkbox("Debug Neighborhood", &DebugRenderNeighborhood);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
		ImGui::Checkbox("Debug Partitions", &DebugRenderPartitions);
#endif
		ImGui::SliderFloat("Neighborhood Radius", &NeighborhoodRadius, 50.f, 1000.f, "%.0f");
		ImGui::SliderFloat("Evade Radius", &EvadeRadius, 50.f, 1500.f, "%.0f");

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

		if (pBlendedSteering)
		{
			if (auto* w = pBlendedSteering->GetWeight(pSeparationBehavior.get()))
				ImGui::SliderFloat("Separation", w, 0.f, 1.f, "%.2f");

			if (auto* w = pBlendedSteering->GetWeight(pCohesionBehavior.get()))
				ImGui::SliderFloat("Cohesion", w, 0.f, 1.f, "%.2f");

			if (auto* w = pBlendedSteering->GetWeight(pVelMatchBehavior.get()))
				ImGui::SliderFloat("Alignment", w, 0.f, 1.f, "%.2f");

			if (auto* w = pBlendedSteering->GetWeight(pSeekBehavior.get()))
				ImGui::SliderFloat("Seek", w, 0.f, 1.f, "%.2f");

			if (auto* w = pBlendedSteering->GetWeight(pFlockWanderBehavior.get()))
				ImGui::SliderFloat("Wander", w, 0.f, 1.f, "%.2f");
		}

		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	if (!pWorld) return;
	if (Agents.Num() == 0 || !IsValid(Agents[0])) return;

	ASteeringAgent* agent0 = Agents[0];

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (!pPartitionedSpace) return;
	pPartitionedSpace->RegisterNeighbors(*agent0, NeighborhoodRadius);
	const int n = pPartitionedSpace->GetNrOfNeighbors();
	const auto& neighbors = pPartitionedSpace->GetNeighbors();
#else
	RegisterNeighbors(agent0);
	const int n = NrOfNeighbors;
	const auto& neighbors = Neighbors;
#endif

	const FVector2D center2D = agent0->GetPosition();
	const FVector center3D = ToWorld3D(center2D);

	DrawDebugCircle(
		pWorld,
		center3D,
		NeighborhoodRadius,
		32,
		FColor::Green,
		false,
		0.f,
		0,
		2.f,
		FVector(1, 0, 0),
		FVector(0, 1, 0),
		false
	);

	for (int i = 0; i < n; ++i)
	{
		ASteeringAgent* nb = neighbors[i];
		if (!IsValid(nb)) continue;
		DrawDebugLine(pWorld, center3D, ToWorld3D(nb->GetPosition()), FColor::Green, false, 0.f, 0, 1.5f);
	}
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const Agent)
{
	NrOfNeighbors = 0;
	if (!IsValid(Agent)) return;

	const FVector2D agentPos = Agent->GetPosition();
	const float r2 = NeighborhoodRadius * NeighborhoodRadius;

	for (ASteeringAgent* other : Agents)
	{
		if (!IsValid(other)) continue;
		if (other == Agent) continue;

		const FVector2D d = other->GetPosition() - agentPos;
		if (d.SquaredLength() <= r2)
		{
			Neighbors[NrOfNeighbors] = other;
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	const int n = GetNrOfNeighbors();
	const auto& neighbors = GetNeighbors();
	if (n <= 0) return FVector2D::ZeroVector;

	FVector2D avg = FVector2D::ZeroVector;
	for (int i = 0; i < n; ++i)
	{
		if (!IsValid(neighbors[i])) continue;
		avg += neighbors[i]->GetPosition();
	}
	return avg / float(n);
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	const int n = GetNrOfNeighbors();
	const auto& neighbors = GetNeighbors();
	if (n <= 0) return FVector2D::ZeroVector;

	FVector2D avg = FVector2D::ZeroVector;
	for (int i = 0; i < n; ++i)
	{
		if (!IsValid(neighbors[i])) continue;
		avg += neighbors[i]->GetLinearVelocity();
	}
	return avg / float(n);
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
	if (!pSeekBehavior) return;

	FTargetData td{};
	td.Position = Target.Position;
	td.Orientation = Target.Orientation;
	td.LinearVelocity = Target.LinearVelocity;
	td.AngularVelocity = Target.AngularVelocity;

	pSeekBehavior->SetTarget(td);
}
