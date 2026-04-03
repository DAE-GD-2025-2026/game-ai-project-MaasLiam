// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>
#include "CoreMinimal.h"
#include "CombinedSteeringBehaviors.h"
#include "GameAIProg/Shared/Level_Base.h"
#include "GameAIProg/Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "Level_CombinedSteering.generated.h"


class EvadeRadius final : public Evade
{
public:
	void SetEvadeRadius(float r) { Radius = r; }

	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override
	{
		SteeringOutput s = Evade::CalculateSteering(DeltaT, Agent);

		const float dist = (Target.Position - Agent.GetPosition()).Length();
		if (dist > Radius)
		{
			s.IsValid = false;
		}
		return s;
	}

private:
	float Radius = 400.f;
};

UCLASS()
class GAMEAIPROG_API ALevel_CombinedSteering : public ALevel_Base
{
	GENERATED_BODY()

public:
	ALevel_CombinedSteering();
	
	virtual void Tick(float DeltaTime) override;

protected:
	virtual void BeginPlay() override;

	virtual void BeginDestroy() override;

private:
	// Agents
	ASteeringAgent* DrunkAgent = nullptr;
	ASteeringAgent* EvadingAgent = nullptr;
	
	std::unique_ptr<Seek> pSeek = nullptr;
	std::unique_ptr<Wander> pWander = nullptr;
	std::unique_ptr<BlendedSteering> pBlendedSteering = nullptr;
	
	std::unique_ptr<EvadeRadius> pEvade = nullptr;
	std::unique_ptr<Wander> pEvadeWander = nullptr;
	std::unique_ptr<PrioritySteering> pPrioritySteering = nullptr;
	
	bool UseMouseTarget = true;
	bool CanDebugRender = false;

	float EvadeRadiusValue = 400.f;
	
	void HandleMouseClickTarget();
	void UpdateCombinedTargets();
	void ApplyDebugRendering();

	
};

