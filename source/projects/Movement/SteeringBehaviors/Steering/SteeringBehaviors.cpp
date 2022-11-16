//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework/EliteMath/EMatrix2x3.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
	}

	return steering;
}

//FLEE
//****
SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= -1;
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
	}



	return steering;
}

//ARRIVE
//****
SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};
	const float slowArea{ 15.f };
	const float stopDistance{ 5.f };

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();


	float distance = steering.LinearVelocity.Magnitude();
	steering.LinearVelocity.Normalize();



	if (abs(distance - stopDistance) < 0.01f)
	{
		steering.LinearVelocity *= 0.f;
	}
	if (distance < slowArea)
	{
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * ((distance - stopDistance) / slowArea);
	}
	else
	{
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();
	}


	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0,1,0 });
	}

	return steering;
}

//FACE
//****
SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	pAgent->SetAutoOrient(false);
	SteeringOutput steering = {};

	Elite::Vector2 directionVector = m_Target.Position - pAgent->GetPosition();
	directionVector.Normalize();

	steering.AngularVelocity = Elite::VectorToOrientation(directionVector) - ((pAgent->GetRotation()));

	if (steering.AngularVelocity > pAgent->GetMaxAngularSpeed())
	{
		steering.AngularVelocity = pAgent->GetMaxAngularSpeed();
	}
	else if (steering.AngularVelocity < -(pAgent->GetMaxAngularSpeed()))
	{
		steering.AngularVelocity = -(pAgent->GetMaxAngularSpeed());
	}


	return steering;
}

//WANDER
//****
SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	
	m_Angle +=  (Elite::randomFloat(0,1) * (m_MaxAngle)) - (m_MaxAngle / 2.f) /** deltaT*/ ;
	Elite::ClampRef(m_Angle, Elite::ToRadians(-90), Elite::ToRadians(90));

	const Elite::Vector2 wanderCircleCenter{ pAgent->GetPosition() + pAgent->GetLinearVelocity().GetNormalized() * m_Offset };
	const Elite::Vector2 vecFromCenter{ cosf(m_Angle) * m_Radius,sinf(m_Angle) * m_Radius };

	m_Target.Position = (wanderCircleCenter + vecFromCenter);



	if (pAgent->CanRenderBehavior())
	{
		
		DEBUGRENDERER2D->DrawCircle(wanderCircleCenter, m_Radius, { 0.f, 0.f, 1.f, 0.5f }, 0.40f);
		DEBUGRENDERER2D->DrawSolidCircle(m_Target.Position, 0.1f, {}, { 0.f, 1.f, 0.f }, 0.40f);
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetLinearVelocity(), m_Offset, {0.f, 0.f, 1.f, 0.5f}, 0.40f);

		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgent->GetLinearVelocity(), pAgent->GetLinearVelocity().Magnitude(), { 1.f, 0.f, 1.f, 0.5f }, 0.40f);
	}

	return Seek::CalculateSteering(deltaT, pAgent);
}

//PURSUIT
//****
SteeringOutput Pursuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	if (m_Target.LinearVelocity.Magnitude() > 0.f)
	{
		m_Target.Position += m_Target.GetDirection() * (m_Target.Position - pAgent->GetPosition()).Magnitude() / pAgent->GetMaxLinearSpeed();
	}

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawSolidCircle(m_Target.Position, 0.25f, {}, { 1.f, 0.f, 1.f }, 0.40f);
	}

	return Seek::CalculateSteering(deltaT, pAgent);
}

//EVADE
//****
SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{ Pursuit::CalculateSteering(deltaT, pAgent) };

	steering.LinearVelocity = -steering.LinearVelocity;


	Elite::Vector2 distanceVector{
		pAgent->GetPosition().x - m_Target.Position.x,
		pAgent->GetPosition().y - m_Target.Position.y
	};

	float distanceToTargetSquared = 
		powf(pAgent->GetPosition().x - m_Target.Position.x,2)
		+ powf(pAgent->GetPosition().y - m_Target.Position.y,2);
	if (m_EvadeRange != 0)
	{
		if (m_EvadeRange * m_EvadeRange < distanceToTargetSquared)
		{
			steering.IsValid = false;
		}
	}
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawCircle(pAgent->GetPosition(), m_EvadeRange, {0.f, 0.f, 1.f, 0.5f}, 0.40f);
	}

	return steering;
}