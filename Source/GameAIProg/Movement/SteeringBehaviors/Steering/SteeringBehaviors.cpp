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
	
	Steering.LinearVelocity = Agent.GetPosition() - Target.Position;
	//Add debug rendering for grades!
	DrawDebug(DeltaT, Agent);
	
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const float SlowRadius = 400.f;
	const float TargetRadius = 100.f;

	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D toTarget = Target.Position - agentPos;
	const float distance = toTarget.Length();
	
	float speedFactor = 1.f;
	if (distance < SlowRadius)
	{
		speedFactor = (distance - TargetRadius) / (SlowRadius - TargetRadius);
		speedFactor = FMath::Clamp(speedFactor, 0.f, 1.f);
	}

	FVector2D dir = toTarget;
	Steering.LinearVelocity = dir * speedFactor;

	// Debug circles around agent
	const FVector center(agentPos.X, agentPos.Y, 0.f);
	DrawDebugCircle(Agent.GetWorld(), center, SlowRadius, 32, FColor::Blue, false, 0.f, 0, 2.f,
					FVector(1,0,0), FVector(0,1,0), false);
	DrawDebugCircle(Agent.GetWorld(), center, TargetRadius, 32, FColor::Red, false, 0.f, 0, 2.f,
					FVector(1,0,0), FVector(0,1,0), false);

	DrawDebug(DeltaT, Agent);
	return Steering;
}
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D toTarget = Target.Position - agentPos;

	if (toTarget.IsNearlyZero())
		return Steering;
	
	const float desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(toTarget.Y, toTarget.X));
	const float currentYaw = Agent.GetRotation();
	const float deltaYaw = FMath::FindDeltaAngleDegrees(currentYaw, desiredYaw);
	
	const float MaxTurnSpeed = 180.f;
	
	float angularVel = deltaYaw * 5.f;
	angularVel = FMath::Clamp(angularVel, -MaxTurnSpeed, MaxTurnSpeed);

	Steering.AngularVelocity = angularVel;

	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;


	const FVector2D targetVel = Target.LinearVelocity; 
	const FVector2D toTarget = targetPos - agentPos;
	const float distance = toTarget.Length();
	
	float pursuerSpeed = Agent.GetVelocity().Length();
	if (pursuerSpeed < 1.f)
	{
		pursuerSpeed = Agent.GetLinearVelocity().Length();
		if (pursuerSpeed < 1.f)
			pursuerSpeed = 150.f; 
	}

	const float predictionTime = distance / pursuerSpeed;
	const FVector2D predictedPos = targetPos + targetVel * predictionTime;

	Steering.LinearVelocity = predictedPos - agentPos;
	Steering.LinearVelocity.Normalize();

	DrawDebug(DeltaT, Agent);
	return Steering;
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;
	const FVector2D targetVel = Target.LinearVelocity;

	const FVector2D toTarget = targetPos - agentPos;
	const float distance = toTarget.Length();

	float pursuerSpeed = Agent.GetVelocity().Length();
	if (pursuerSpeed < 1.f)
	{
		pursuerSpeed = Agent.GetLinearVelocity().Length();
		if (pursuerSpeed < 1.f)
			pursuerSpeed = 150.f;
	}

	const float predictionTime = distance / pursuerSpeed;

	const FVector2D predictedPos = targetPos + targetVel * predictionTime;

	Steering.LinearVelocity = agentPos - predictedPos;
	Steering.LinearVelocity.Normalize();

	DrawDebug(DeltaT, Agent);
	return Steering;
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	m_WanderAngle += FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);
	m_WanderAngle = FMath::UnwindRadians(m_WanderAngle); // keep stable range
	
	const FVector2D agentPos = Agent.GetPosition();

	const FVector fwd3 = Agent.GetActorForwardVector().GetSafeNormal();
	const FVector2D forward(fwd3.X, fwd3.Y);
	
	const FVector2D circleCenter = agentPos + forward * m_OffsetDistance;
	const FVector2D displacement(
		FMath::Cos(m_WanderAngle) * m_Radius,
		FMath::Sin(m_WanderAngle) * m_Radius
	);

	const FVector2D wanderTarget = circleCenter + displacement;
	Target.Position = wanderTarget;

	SteeringOutput Steering = Seek::CalculateSteering(DeltaT, Agent);
	const FVector Center{circleCenter.X, circleCenter.Y, 0.f};

	DrawDebugCircle(Agent.GetWorld(), Center, m_Radius, 32, FColor::Blue, false, 0.f, 0, 2.f,
					FVector(1,0,0), FVector(0,1,0), false);
	
	return Steering;
}