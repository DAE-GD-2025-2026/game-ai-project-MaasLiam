#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	if (!pFlock || pFlock->GetNrOfNeighbors() == 0)
	{
		Steering.IsValid = false;
		return Steering;
	}

	FTargetData TargetData{};
	TargetData.Position = pFlock->GetAverageNeighborPos();
	SetTarget(TargetData);

	Steering = Seek::CalculateSteering(deltaT, pAgent);
	Steering.IsValid = true;
	return Steering;
}


//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	Steering.IsValid = false;

	if (!pFlock) return Steering;

	const int NeighbourCount = pFlock->GetNrOfNeighbors();
	if (NeighbourCount <= 0) return Steering;

	const auto& Neighbors = pFlock->GetNeighbors();
	const FVector2D AgentPos = pAgent.GetPosition();

	FVector2D Force = FVector2D::ZeroVector;

	for (int i = 0; i < NeighbourCount; ++i)
	{
		ASteeringAgent* Neighbor = Neighbors[i];
		if (!IsValid(Neighbor)) continue;

		const FVector2D toAgent = AgentPos - Neighbor->GetPosition();
		const float distSq = toAgent.SizeSquared();
		if (distSq <= KINDA_SMALL_NUMBER) continue;
		
		Force += toAgent / distSq;
	}

	if (Force.IsNearlyZero())
		return Steering;

	Force.Normalize();
	Steering.LinearVelocity = Force * pAgent.GetMaxLinearSpeed();
	Steering.AngularVelocity = 0.f;
	Steering.IsValid = true;
	return Steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Steering.IsValid = false;

	if (!pFlock) return Steering;
	if (pFlock->GetNrOfNeighbors() <= 0) return Steering;

	FVector2D desiredVelocity = pFlock->GetAverageNeighborVelocity();
	if (desiredVelocity.IsNearlyZero()) return Steering;

	desiredVelocity = desiredVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());

	Steering.LinearVelocity = desiredVelocity;
	Steering.AngularVelocity = 0.f;
	Steering.IsValid = true;
	return Steering;
}
