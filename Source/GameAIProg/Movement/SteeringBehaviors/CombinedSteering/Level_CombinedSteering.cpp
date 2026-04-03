#include "Level_CombinedSteering.h"

#include "imgui.h"
#include "Kismet/GameplayStatics.h"


ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	PrimaryActorTick.bCanEverTick = true;
}

void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();
	if (TrimWorld)
	{
		TrimWorld->SetTrimWorldSize(3000.f);
		TrimWorld->bShouldTrimWorld = true;
	}
	
	DrunkAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0, 0, 90}, FRotator::ZeroRotator);
	
	EvadingAgent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{300, 0, 90}, FRotator::ZeroRotator);

	pSeek = std::make_unique<Seek>();
	pWander = std::make_unique<Wander>();

	std::vector<BlendedSteering::WeightedBehavior> drunkBehaviors;
	drunkBehaviors.emplace_back(pSeek.get(), 0.5f);
	drunkBehaviors.emplace_back(pWander.get(), 0.5f);

	pBlendedSteering = std::make_unique<BlendedSteering>(drunkBehaviors);
	
	pEvade = std::make_unique<EvadeRadius>();
	pEvadeWander = std::make_unique<Wander>();

	std::vector<ISteeringBehavior*> evadeBehaviors;
	evadeBehaviors.push_back(pEvade.get());
	evadeBehaviors.push_back(pEvadeWander.get());

	pPrioritySteering = std::make_unique<PrioritySteering>(evadeBehaviors);
	
	if (IsValid(DrunkAgent))
	{
		DrunkAgent->SetSteeringBehavior(pBlendedSteering.get());
		DrunkAgent->SetIsAutoOrienting(true);
	}

	if (IsValid(EvadingAgent))
	{
		EvadingAgent->SetSteeringBehavior(pPrioritySteering.get());
		EvadingAgent->SetIsAutoOrienting(true);
	}
	
	UpdateCombinedTargets();
	ApplyDebugRendering();
}

void ALevel_CombinedSteering::BeginDestroy()
{
	if (IsValid(DrunkAgent))   DrunkAgent->Destroy();
	if (IsValid(EvadingAgent)) EvadingAgent->Destroy();
	
	Super::BeginDestroy();
}

void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();
	
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();
	
		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();
		
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("Combined Steering");
		ImGui::Spacing();

		// Debug rendering toggle
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			ApplyDebugRendering();
		}

		// Trim world
		if (TrimWorld)
		{
			ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
			if (TrimWorld->bShouldTrimWorld)
			{
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
					TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
					[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
			}
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("DrunkAgent (Blended) Weights");

		if (pBlendedSteering)
		{
			auto& w = pBlendedSteering->GetWeightedBehaviorsRef();
			
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
				w[0].Weight, 0.f, 1.f,
				[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");

			ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
				w[1].Weight, 0.f, 1.f,
				[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
		}

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		// Evade radius slider
		ImGui::Text("EvadingAgent (Priority)");
		if (ImGui::SliderFloat("Evade Radius", &EvadeRadiusValue, 100.f, 1500.f, "%.0f"))
		{
			if (pEvade) pEvade->SetEvadeRadius(EvadeRadiusValue);
		}
	
		// ImGui::Spacing();
		// ImGui::Separator();
		// ImGui::Spacing();
		// ImGui::Spacing();
	 //
		// ImGui::Text("Flocking");
		// ImGui::Spacing();
		// ImGui::Spacing();
	 //
		// if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		// {
  //  // TODO: Handle the debug rendering of your agents here :)
		// }
		// ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		// if (TrimWorld->bShouldTrimWorld)
		// {
		// 	ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
		// 		TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
		// 		[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		// }
		//
		// ImGui::Spacing();
		// ImGui::Spacing();
		// ImGui::Spacing();
	 //
		// ImGui::Text("Behavior Weights");
		// ImGui::Spacing();


		// ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
		// 	pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
		// 	[this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		//
		// ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
		// pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
		// [this](float InVal) { pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
	
		//End
		ImGui::End();
	}
#pragma endregion
	
	// Combined Steering Update
 // TODO: implement handling mouse click input for seek
 // TODO: implement Make sure to also evade the wanderer
	HandleMouseClickTarget();
	UpdateCombinedTargets();
}

void ALevel_CombinedSteering::HandleMouseClickTarget()
{
	if (!UseMouseTarget) return;

	APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
	if (!PC) return;

	if (PC->WasInputKeyJustPressed(EKeys::LeftMouseButton))
	{
		FHitResult hit;
		const bool bHit = PC->GetHitResultUnderCursorByChannel(UEngineTypes::ConvertToTraceType(ECC_Visibility), true, hit);

		if (bHit)
		{
			MouseTarget.Position = FVector2D(hit.Location.X, hit.Location.Y);
			MouseTarget.LinearVelocity = FVector2D::ZeroVector;
			MouseTarget.Orientation = 0.f;
			MouseTarget.AngularVelocity = 0.f;
		}
	}
}

void ALevel_CombinedSteering::UpdateCombinedTargets()
{
	// DrunkAgent's Seek should go to MouseTarget
	if (pSeek)
	{
		pSeek->SetTarget(MouseTarget);
	}

	// EvadingAgent's Evade target should be DrunkAgent
	if (pEvade && IsValid(DrunkAgent))
	{
		FTargetData target;
		target.Position = DrunkAgent->GetPosition();
		target.Orientation = DrunkAgent->GetRotation();
		target.LinearVelocity = DrunkAgent->GetLinearVelocity();
		target.AngularVelocity = DrunkAgent->GetAngularVelocity();

		pEvade->SetTarget(target);
		pEvade->SetEvadeRadius(EvadeRadiusValue);
	}
}

void ALevel_CombinedSteering::ApplyDebugRendering()
{
	if (IsValid(DrunkAgent))   DrunkAgent->SetDebugRenderingEnabled(CanDebugRender);
	if (IsValid(EvadingAgent)) EvadingAgent->SetDebugRenderingEnabled(CanDebugRender);
}

