#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	FColor ColorGreen = FColor::Green;
	FColor ColorMagenta = FColor::Magenta;
	FColor ColorBlue = FColor::Blue;
	FVector AgentLocation{Agent.GetPosition().X, Agent.GetPosition().Y, 0};
	FVector AgentVelocity{Agent.GetVelocity().X, Agent.GetVelocity().Y, 0};
	//Fvector AgentRotationVelocity{Agent.GetAngularVelocity(), Agent.GetRotation().Y, 0};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	//Add debug rendering for grades!
	//DrawVelocity
	DrawDebugLine(Agent.GetWorld(), AgentLocation, AgentLocation + AgentVelocity, ColorGreen);
	//DrawForward
	DrawDebugLine(Agent.GetWorld(), AgentLocation, AgentLocation +( Agent.GetActorForwardVector() * Agent.GetLinearVelocity().Length()), ColorMagenta);
	
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = -Target.Position;
	Steering.LinearVelocity.Normalize();
	//Add debug rendering for grades!
	
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	FColor Red = FColor::Red;
	
	
	Steering.LinearVelocity = -Target.Position;
	Steering.LinearVelocity.Normalize();
	float OuterRadius{100.f};
	float InnerRadius{25.f};
	FVector AgentPosition{};
	AgentPosition.X = Agent.GetPosition().X;
	AgentPosition.Y = Agent.GetPosition().Y;
	
	DrawCircle(Agent.GetWorld(), Agent.GetActorLocation(), AgentPosition, AgentPosition, Red, InnerRadius, 6, false, -1.f, 0.f, 1.f); 
	//Add debug rendering for grades!

	return Steering;
}