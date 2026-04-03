
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput blended = {};
	blended.IsValid = false;

	float totalWeight = 0.f;

	for (const WeightedBehavior& wb : WeightedBehaviors)
	{
		if (!wb.pBehavior || wb.Weight <= 0.f)
			continue;

		SteeringOutput s = wb.pBehavior->CalculateSteering(DeltaT, Agent);

		if (!s.IsValid)
			continue;

		blended.LinearVelocity += s.LinearVelocity * wb.Weight;
		blended.AngularVelocity += s.AngularVelocity * wb.Weight;

		totalWeight += wb.Weight;
		blended.IsValid = true;
	}
	
	if (blended.IsValid && totalWeight > 0.f)
	{
		blended.LinearVelocity /= totalWeight;
		blended.AngularVelocity /= totalWeight;
	}

	blended.LinearVelocity = blended.LinearVelocity.GetClampedToMaxSize(Agent.GetMaxLinearSpeed());
	blended.AngularVelocity = FMath::Clamp(blended.AngularVelocity, -Agent.GetMaxAngularSpeed(), Agent.GetMaxAngularSpeed());

	return blended;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	auto it = find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}
	return Steering;
}