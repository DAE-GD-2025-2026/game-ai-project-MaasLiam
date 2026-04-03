// Fill out your copyright notice in the Description page of Project Settings.


#include "Level_GraphTheory.h"

#include "Algorithms/EulerianPath.h"
#include "Shared/GameAISpectator.h"

using namespace GameAI;

// Sets default values
ALevel_GraphTheory::ALevel_GraphTheory()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_GraphTheory::BeginPlay()
{
	Super::BeginPlay();
	Renderer = MakeUnique<GameAI::GraphRenderer>(GetWorld());	
	
	// Add the graph editor to our player
	if (PlayerController = Cast<APlayerController>(GetWorld()->GetFirstLocalPlayerFromController()->PlayerController); 
		GraphEditorClass && PlayerController)
	{
		PlayerGraphEditor = NewObject<UGraphEditorComponent>(PlayerController->GetPawn(), GraphEditorClass);
		PlayerGraphEditor->RegisterComponent();
		PlayerGraphEditor->SetEditedGraph(&Graph);
		PlayerGraphEditor->SetNodeFactory(&NodeFactory);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Unable to get PlayerController from LocalPlayer or GraphEditorClass is null"))
		return;
	}
	
	// Make the view orthogonal for less perspective issues
	if (AGameAISpectator* Player = Cast<AGameAISpectator>(PlayerController->GetPawnOrSpectator()); Player)
	{
		Player->SetCameraProjection(ECameraProjectionMode::Orthographic);
	}
	
	const int n0 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{200.f, 200.f}));
	const int n1 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{500.f, 200.f}));
	const int n2 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{500.f, 500.f}));
	const int n3 = Graph.AddNode(NodeFactory.CreateNode(FVector2D{200.f, 500.f}));

	Graph.AddConnection(n0, n1);
	Graph.AddConnection(n1, n2);
	Graph.AddConnection(n2, n3);
	Graph.AddConnection(n3, n0);

	Graph.SetConnectionCostsToDistances();
	
	// Spawn the Agent
	Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, 
	FVector{0,0,90}, FRotator::ZeroRotator);
	Agent->SetSteeringBehavior(&PathFollow);
}

void ALevel_GraphTheory::BeginDestroy()
{
	Super::BeginDestroy();
}

void ALevel_GraphTheory::Tick(float DeltaTime)
{
	if (Renderer)
	{
		Renderer->RenderGraph(Graph);
	}
	
#pragma region UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
		ImGui::SetWindowFocus();
		ImGui::PushItemWidth(70);
		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
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

		ImGui::Text("Graph Theory");
		ImGui::Spacing();
		ImGui::Spacing();

		//End
		ImGui::End();
	}
#pragma endregion UI
	
	Renderer->RenderGraph(Graph);
	
	if (PlayerGraphEditor && PlayerGraphEditor->HasGraphUpdated())
	{
		EulerianPath eulerianPath{ &Graph };
		Eulerianity eulerianity = Eulerianity::notEulerian;
		
		std::vector<Node*> trail = eulerianPath.FindPath(eulerianity);
		UpdateAgentPath(trail);
	}
}

void ALevel_GraphTheory::UpdateAgentPath(std::vector<Node*> const& Trail)
{
	std::vector<FVector2D> path{};
	
	for (Node* node : Trail)
	{
		if (node != nullptr)
		{
			path.push_back(node->GetPosition());
		}
	}

	PathFollow.SetPath(path);
	if (Agent && !path.empty())
	{
		Agent->SetPosition(path[0]);
	}
}





