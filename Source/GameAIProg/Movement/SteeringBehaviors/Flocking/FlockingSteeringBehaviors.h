#pragma once
#include "Movement/SteeringBehaviors/SteeringAgent.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
class Flock;

//COHESION - FLOCKING
//*******************
class Cohesion final : public Seek
{
public:
	Cohesion(Flock* const pFlock) :pFlock(pFlock) {};

	//Cohesion Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};

//SEPARATION - FLOCKING
//*********************
class Separation final : public ISteeringBehavior
{
public:
	Separation(Flock* const pFlock) :pFlock(pFlock) {};

	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************
class VelocityMatch final : public ISteeringBehavior
{
public:
	VelocityMatch(Flock* const pFlock) :pFlock(pFlock) {};

	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};
class EvadeWithRadius final : public Evade
{
public:
	void SetEvadeRadius(float r) { Radius = r; }
	void SetHasTarget(bool b) { bHasTarget = b; }

	SteeringOutput CalculateSteering(float dt, ASteeringAgent& agent) override
	{
		SteeringOutput s{};
		if (!bHasTarget)
		{
			s.IsValid = false;
			return s;
		}

		const float dist = (Target.Position - agent.GetPosition()).Length();
		if (dist > Radius)
		{
			s.IsValid = false;
			return s;
		}

		s = Evade::CalculateSteering(dt, agent);
		s.IsValid = true;
		return s;
	}

private:
	float Radius = 400.f;
	bool bHasTarget = false;
};
