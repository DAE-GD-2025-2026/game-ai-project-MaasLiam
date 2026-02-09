#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"

void DrawDebug(float DeltaT, ASteeringAgent & Agent)
{
	FColor ColorGreen = FColor::Green;
	FColor ColorMagenta = FColor::Magenta;
	FColor ColorBlue = FColor::Blue;
	FVector AgentLocation{Agent.GetPosition().X, Agent.GetPosition().Y, 0};
	FVector AgentVelocity{Agent.GetVelocity().X, Agent.GetVelocity().Y, 0};
	
	float Dot = FVector::DotProduct( Agent.GetActorForwardVector().GetSafeNormal(), AgentVelocity.GetSafeNormal());
	Dot = FMath::Clamp(Dot, -1.f, 1.f);
	float Sign = FMath::Sign(FVector::CrossProduct( Agent.GetActorForwardVector(), AgentVelocity.GetSafeNormal()).Z);
	float SignedAngleRad = FMath::Acos(Dot) * Sign;
	float SignedAngleDeg = FMath::RadiansToDegrees(SignedAngleRad);
	FVector Right = Agent.GetActorRightVector();
	float TurnVizScale = 20.0f;
	float  AngleLength = FMath::Abs(SignedAngleDeg) * TurnVizScale;
	FVector AngleDir = (SignedAngleDeg >= 0.f) ? (-Right) : (Right);
	
	//DrawVelocity
	DrawDebugLine(Agent.GetWorld(), AgentLocation, AgentLocation + AgentVelocity, ColorGreen);
	//DrawForward
	DrawDebugLine(Agent.GetWorld(), AgentLocation, AgentLocation +( Agent.GetActorForwardVector() * Agent.GetLinearVelocity().Length()), ColorMagenta);
	//DrawAngle
	DrawDebugLine(Agent.GetWorld(), AgentLocation, AgentLocation + AngleDir *  AngleLength, FColor::Cyan, false,0.f,0,2.f);
}
//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	//Add debug rendering for grades!
	DrawDebug(DeltaT, Agent);
	
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = -Target.Position;
	Steering.LinearVelocity.Normalize();
	//Add debug rendering for grades!
	DrawDebug(DeltaT, Agent);
	
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent & Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	
	//DrawDebug
	FColor Red = FColor::Red;
	FColor Blue = FColor::Blue;
	float OuterRadius{400.f};
	float InnerRadius{100.f};
	FVector AgentPosition{};
	AgentPosition.X = Agent.GetPosition().X;
	AgentPosition.Y = Agent.GetPosition().Y;
	
	//Slow
	DrawDebugCircle(Agent.GetWorld(), AgentPosition, OuterRadius, 20, Blue,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
	//Stop
	DrawDebugCircle(Agent.GetWorld(), AgentPosition, InnerRadius, 20, Red,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
	
	return Steering;
}