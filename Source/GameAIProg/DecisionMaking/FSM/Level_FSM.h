// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "Shared/Level_Base.h"
#include "Level_FSM.generated.h"

class ISteeringBehavior;
class Seek;
class Arrive;
class Wander;

UCLASS()
class GAMEAIPROG_API ALevel_FSM : public ALevel_Base
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ALevel_FSM();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

private:
	UPROPERTY()
	ASteeringAgent* Guard{nullptr};

	UPROPERTY()
	ASteeringAgent* Thief{nullptr};

	std::unique_ptr<Arrive> GuardArriveBehavior{};
	std::unique_ptr<Wander> GuardWanderBehavior{};
	std::unique_ptr<Seek> ThiefSeekBehavior{};

	bool bWasLeftMouseDown{false};
};
