// Fill out your copyright notice in the Description page of Project Settings.

#include "Level_FSM.h"

#include "AIController.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "DecisionMaking/GameAIController.h"
#include "DrawDebugHelpers.h"
#include "FSM.h"
#include "FSMComponent.h"
#include "InputCoreTypes.h"
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
#include "Movement/SteeringBehaviors/SteeringAgent.h"

namespace
{
	constexpr float DetectionRadius = 650.f;
	constexpr float PatrolPointReachedRadius = 130.f;
	constexpr float SearchTime = 6.f;

	FVector2D To2D(const FVector& V)
	{
		return FVector2D{V.X, V.Y};
	}

	bool HasLineOfSight(UWorld* World, ASteeringAgent* Guard, ASteeringAgent* Target)
	{
		if (!World || !Guard || !Target) return false;

		FHitResult Hit{};
		FCollisionQueryParams Params{};
		Params.AddIgnoredActor(Guard);

		const FVector Start = Guard->GetActorLocation() + FVector{0.f, 0.f, 40.f};
		const FVector End = Target->GetActorLocation() + FVector{0.f, 0.f, 40.f};
		const bool bHit = World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params);

		return !bHit || Hit.GetActor() == Target;
	}

	bool IsTargetVisible(UWorld* World, ASteeringAgent* Guard, ASteeringAgent* Target, GameAI::FSM::FSM* FSM)
	{
		if (!World || !Guard || !Target || !FSM) return false;

		const float Distance = FVector::Dist2D(Guard->GetActorLocation(), Target->GetActorLocation());
		const bool bVisible = Distance <= DetectionRadius && HasLineOfSight(World, Guard, Target);

		FSM->GetBlackboard().Set<bool>("TargetVisible", bVisible);
		FSM->GetBlackboard().Set<FVector>("TargetLocation", Target->GetActorLocation());

		if (bVisible)
		{
			FSM->GetBlackboard().Set<FVector>("LastKnownTargetLocation", Target->GetActorLocation());
		}

		return bVisible;
	}

	class PatrolState final : public GameAI::FSM::State
	{
	public:
		PatrolState(Arrive* InArrive, std::vector<FVector> InPatrolPath)
			: ArriveBehavior(InArrive), PatrolPath(std::move(InPatrolPath)) {}

		void OnEnter(GameAI::FSM::Context& Ctx) override
		{
			if (!Ctx.Agent || !ArriveBehavior || PatrolPath.empty()) return;

			Ctx.Agent->SetSteeringBehavior(ArriveBehavior);
			TargetIndex = Ctx.Blackboard ? Ctx.Blackboard->GetOr<int>("PatrolIndex", 0) : 0;

			TimeWithoutProgress = 0.f;
			LastDistanceToTarget = TNumericLimits<float>::Max();

			SetCurrentTarget();
		}

		void Tick(GameAI::FSM::Context& Ctx, float DeltaTime) override
		{
			if (!Ctx.Agent || PatrolPath.empty()) return;

			const FVector2D AgentPos = Ctx.Agent->GetPosition();
			const FVector2D TargetPos = To2D(PatrolPath[TargetIndex]);
			const float Distance = FVector2D::Distance(AgentPos, TargetPos);

			if (Distance <= PatrolPointReachedRadius)
			{
				GoToNextPoint(Ctx);
				return;
			}

			// If the guard is stuck against a wall and no longer getting closer,
			// skip this patrol point instead of staying stuck forever.
			if (Distance < LastDistanceToTarget - 5.f)
			{
				TimeWithoutProgress = 0.f;
				LastDistanceToTarget = Distance;
			}
			else
			{
				TimeWithoutProgress += DeltaTime;
			}

			if (TimeWithoutProgress >= 1.5f)
			{
				GoToNextPoint(Ctx);
			}
		}

	private:
		void GoToNextPoint(GameAI::FSM::Context& Ctx)
		{
			TargetIndex = (TargetIndex + 1) % PatrolPath.size();

			if (Ctx.Blackboard)
			{
				Ctx.Blackboard->Set<int>("PatrolIndex", static_cast<int>(TargetIndex));
			}

			TimeWithoutProgress = 0.f;
			LastDistanceToTarget = TNumericLimits<float>::Max();

			SetCurrentTarget();
		}

		void SetCurrentTarget() const
		{
			if (!ArriveBehavior || PatrolPath.empty()) return;
			ArriveBehavior->SetTarget(FTargetData{To2D(PatrolPath[TargetIndex])});
		}

		Arrive* ArriveBehavior{nullptr};
		std::vector<FVector> PatrolPath{};
		size_t TargetIndex{0};

		float TimeWithoutProgress{0.f};
		float LastDistanceToTarget{0.f};
	};

	class ChaseState final : public GameAI::FSM::State
	{
	public:
		explicit ChaseState(Arrive* InArrive) : ArriveBehavior(InArrive) {}

		void OnEnter(GameAI::FSM::Context& Ctx) override
		{
			UE_LOG(LogTemp, Warning, TEXT("FSM: ENTER CHASE"));
			if (Ctx.Agent && ArriveBehavior)
			{
				Ctx.Agent->SetSteeringBehavior(ArriveBehavior);
			}
		}

		void Tick(GameAI::FSM::Context& Ctx, float DeltaTime) override
		{
			if (!Ctx.Blackboard || !ArriveBehavior) return;
			const FVector TargetLocation = Ctx.Blackboard->GetOr<FVector>("TargetLocation", FVector::ZeroVector);
			ArriveBehavior->SetTarget(FTargetData{To2D(TargetLocation)});
		}

	private:
		Arrive* ArriveBehavior{nullptr};
	};

	class SearchState final : public GameAI::FSM::State
	{
	public:
		SearchState(Arrive* InArrive, Wander* InWander)
			: ArriveBehavior(InArrive), WanderBehavior(InWander) {}

		void OnEnter(GameAI::FSM::Context& Ctx) override
		{
			UE_LOG(LogTemp, Warning, TEXT("FSM: ENTER SEARCH"));
			SearchElapsed = 0.f;
			bReachedLastKnownLocation = false;

			if (!Ctx.Agent || !Ctx.Blackboard || !ArriveBehavior) return;

			const FVector LastKnownLocation = Ctx.Blackboard->GetOr<FVector>("LastKnownTargetLocation", Ctx.Agent->GetActorLocation());
			Ctx.Agent->SetSteeringBehavior(ArriveBehavior);
			ArriveBehavior->SetTarget(FTargetData{To2D(LastKnownLocation)});
		}

		void Tick(GameAI::FSM::Context& Ctx, float DeltaTime) override
		{
			SearchElapsed += DeltaTime;
			if (Ctx.Blackboard)
			{
				Ctx.Blackboard->Set<float>("SearchElapsed", SearchElapsed);
			}

			if (!Ctx.Agent || !Ctx.Blackboard || bReachedLastKnownLocation) return;

			const FVector LastKnownLocation = Ctx.Blackboard->GetOr<FVector>("LastKnownTargetLocation", Ctx.Agent->GetActorLocation());
			if (FVector2D::Distance(Ctx.Agent->GetPosition(), To2D(LastKnownLocation)) <= PatrolPointReachedRadius)
			{
				bReachedLastKnownLocation = true;
				if (WanderBehavior)
				{
					Ctx.Agent->SetSteeringBehavior(WanderBehavior);
				}
			}
		}

	private:
		Arrive* ArriveBehavior{nullptr};
		Wander* WanderBehavior{nullptr};
		float SearchElapsed{0.f};
		bool bReachedLastKnownLocation{false};
	};
}

ALevel_FSM::ALevel_FSM()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_FSM::BeginPlay()
{
	Super::BeginPlay();

	if (!SteeringAgentClass)
	{
		UE_LOG(LogTemp, Error, TEXT("Level_FSM: SteeringAgentClass is not set."));
		return;
	}

	Thief = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0.f, -300.f, 90.f}, FRotator::ZeroRotator);
	Guard = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0.f, 300.f, 90.f}, FRotator::ZeroRotator);

	if (!Thief || !Guard) return;

	Thief->SetDebugRenderingEnabled(false);
	Guard->SetDebugRenderingEnabled(false);
	Thief->SetMaxLinearSpeed(450.f);
	Guard->SetMaxLinearSpeed(350.f);

	ThiefSeekBehavior = std::make_unique<Seek>();
	ThiefSeekBehavior->SetTarget(FTargetData{Thief->GetPosition()});
	Thief->SetSteeringBehavior(ThiefSeekBehavior.get());

	GuardArriveBehavior = std::make_unique<Arrive>();
	GuardArriveBehavior->SetTargetRadius(PatrolPointReachedRadius);
	GuardWanderBehavior = std::make_unique<Wander>();
	GuardWanderBehavior->SetWanderOffset(300.f);
	GuardWanderBehavior->SetWanderRadius(150.f);
	GuardWanderBehavior->SetMaxAngleChange(0.6f);

	// Runtime-spawned characters are sometimes not automatically possessed,
	// so create and possess the guard controller explicitly.
	AGameAIController* AIController = GetWorld()->SpawnActor<AGameAIController>(AGameAIController::StaticClass());
	if (AIController)
	{
		AIController->Possess(Guard);
	}

	if (AIController)
	{
		UFSMComponent* FSMComponent = AIController->FindComponentByClass<UFSMComponent>();
		if (FSMComponent)
		{
			std::vector<FVector> PatrolPath{
				FVector{-150.f, 250.f, 90.f},
				FVector{350.f, 250.f, 90.f},
				FVector{350.f, -250.f, 90.f},
				FVector{-150.f, -250.f, 90.f}
			};

			GameAI::FSM::State* Patrol = FSMComponent->AddState(std::make_unique<PatrolState>(GuardArriveBehavior.get(), PatrolPath));
			GameAI::FSM::State* Chase = FSMComponent->AddState(std::make_unique<ChaseState>(GuardArriveBehavior.get()));
			GameAI::FSM::State* Search = FSMComponent->AddState(std::make_unique<SearchState>(GuardArriveBehavior.get(), GuardWanderBehavior.get()));

			GameAI::FSM::FSM* FSM = FSMComponent->GetFSM();
			FSMComponent->AddTransition(Patrol, Chase, [this, FSM]()
			{
				return IsTargetVisible(GetWorld(), Guard, Thief, FSM);
			});
			FSMComponent->AddTransition(Chase, Search, [this, FSM]()
			{
				return !IsTargetVisible(GetWorld(), Guard, Thief, FSM);
			});
			FSMComponent->AddTransition(Search, Chase, [this, FSM]()
			{
				return IsTargetVisible(GetWorld(), Guard, Thief, FSM);
			});
			FSMComponent->AddTransition(Search, Patrol, [FSM]()
			{
				return FSM && FSM->GetBlackboard().GetOr<float>("SearchElapsed", 0.f) >= SearchTime;
			});

			AIController->RunFiniteStateMachine();
			UE_LOG(LogTemp, Warning, TEXT("Level_FSM: Guard FSM started."));
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("Level_FSM: Guard controller has no FSMComponent."));
		}
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Level_FSM: Failed to spawn GameAIController for guard."));
	}
}

void ALevel_FSM::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (PlayerController && Thief && ThiefSeekBehavior)
	{
		const bool bLeftMouseDown = PlayerController->IsInputKeyDown(EKeys::LeftMouseButton);
		if (bLeftMouseDown && !bWasLeftMouseDown)
		{
			ThiefSeekBehavior->SetTarget(FTargetData{To2D(LatestMouseWorldPos)});
		}
		bWasLeftMouseDown = bLeftMouseDown;
	}
}
