#include "b2FrictionJoint.h"
#include "../b2Body.h"
#include "../b2World.h"

void b2FrictionJointDef::Initialize(b2Body* b1, const b2Vec2 &_frictionForce, const float32 _frictionTorque)
{
	body1 = b1;
	frictionForce = _frictionForce;
	frictionTorque = _frictionTorque;
}
void b2FrictionJointDef::Initialize(b2Body* b1, const float32 _frictionTorque)
{
	Initialize(b1, b2Vec2_zero, _frictionTorque);
}

b2FrictionJoint::b2FrictionJoint(const b2FrictionJointDef* def)
	: b2Joint(def)
{
	m_frictionForce = def->frictionForce;
	m_frictionTorque = def->frictionTorque;

	m_body1 = def->body1;
	b2Assert(def->body2 == NULL);
	m_body2 = NULL;
}

void b2FrictionJoint::InitVelocityConstraints(const b2TimeStep& step)
{
	// Get bodies
	b2Body* b1 = m_body1;
	m_lambda_p.Set(0.0f, 0.0f);
}

void b2FrictionJoint::SolveVelocityConstraints(const b2TimeStep& step)
{
	B2_NOT_USED(step);

	if (m_frictionForce.x != 0.0 || m_frictionForce.y != 0.0)
	{
		// Get bodies
		b2Body* b1 = m_body1;

		b2Vec2 initial_p(b1->m_mass * b1->m_linearVelocity);

		b2Vec2 initial_f = step.inv_dt * initial_p;

		//Clamp down the force applied
		b2Vec2 local_initial_f = b1->GetLocalVector(initial_f);

		//This is the counterforce we're going to want to apply
		b2Vec2 local_f = b2Clamp(local_initial_f, -m_frictionForce, m_frictionForce);

		b2Vec2 final_f = b1->GetWorldVector(local_f);

		b2Vec2 final_p = step.dt * final_f;

		m_lambda_p += final_p;

		// Counteract the velocity
		b1->m_linearVelocity -= b1->m_invMass * final_p;
	}


	// Now, let's do it for angular velocity
	if (m_frictionTorque != 0.0)
	{
		// Get bodies
		b2Body* b1 = m_body1;

		float32 initial_a_p(b1->m_mass * b1->m_angularVelocity);

		float32 initial_t = step.inv_dt * initial_a_p;

		//Clamp down the torque applied
		//This is the countertorque we're going to want to apply
		float32 final_t = b2Clamp(initial_t, -m_frictionTorque, m_frictionTorque);

		float32 final_a_p = step.dt * final_t;

		m_lambda_a_p += final_a_p;

		// Counteract the velocity
		b1->m_angularVelocity -= b1->m_invMass * final_a_p;
	}
}

bool b2FrictionJoint::SolvePositionConstraints(float32 baumgarte)
{
	B2_NOT_USED(baumgarte);

	// There's no position constraints to solve right now.  We might want to check that our body hasn't moved if
	// it didn't use all it's friction
	return true;
};


b2Vec2 b2FrictionJoint::GetAnchor1() const
{
	// Return arbitrary position (we have to implement this abstract virtual function)
	return m_body1->GetWorldCenter();
}

b2Vec2 b2FrictionJoint::GetAnchor2() const
{
	// Return arbitrary position (we have to implement this abstract virtual function)
	return b2Vec2_zero;
}

b2Vec2 b2FrictionJoint::GetReactionForce(float32 inv_dt) const
{
	return inv_dt * m_lambda_p;
}

float32 b2FrictionJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * m_lambda_a_p;
}
