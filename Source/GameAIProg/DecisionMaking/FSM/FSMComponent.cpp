// Fill out your copyright notice in the Description page of Project Settings.

#include "FSMComponent.h"

#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "FSM.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

UFSMComponent::UFSMComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	FSMInstance = std::make_unique<GameAI::FSM::FSM>();
}

GameAI::FSM::State* UFSMComponent::AddState(std::unique_ptr<GameAI::FSM::State>&& NewState)
{
	return FSMInstance ? FSMInstance->AddState(std::move(NewState)) : nullptr;
}

void UFSMComponent::AddTransition(GameAI::FSM::State* From, GameAI::FSM::State* To, std::function<bool()> EvalFunc) const
{
	if (FSMInstance)
	{
		FSMInstance->AddTransition(From, To, std::move(EvalFunc));
	}
}

void UFSMComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UFSMComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (bIsRunning && FSMInstance)
	{
		FSMInstance->Tick(DeltaTime);
	}
}

void UFSMComponent::StartLogic()
{
	Super::StartLogic();

	if (!FSMInstance) return;

	AAIController* Controller = Cast<AAIController>(GetOwner());
	GameAI::FSM::Context Context{};
	Context.Controller = Controller;
	Context.Agent = Controller ? Cast<ASteeringAgent>(Controller->GetPawn()) : nullptr;
	Context.UnrealBlackboard = Controller ? Controller->GetBlackboardComponent() : nullptr;

	FSMInstance->SetContext(Context);
	bIsRunning = true;
	FSMInstance->Start();
}

void UFSMComponent::StopLogic(const FString& Reason)
{
	if (FSMInstance)
	{
		FSMInstance->Stop();
	}

	bIsRunning = false;
	Super::StopLogic(Reason);
}

bool UFSMComponent::IsRunning() const
{
	return bIsRunning;
}
