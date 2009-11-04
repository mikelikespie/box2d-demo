#include "b2FrictionJoint.h"
#include "../b2Body.h"
#include "../b2World.h"

void b2FrictionJointDef::Initialize(b2Body* b1)
{
	body1 = b1;
	body2 = NULL;
}

b2FrictionJoint::b2FrictionJoint(const b2FrictionJointDef* def)
	: b2Joint(def)
{
	m_frictionForce = def->frictionForce;

	// Get bodies
	b2Body* b1 = m_body1;
	b2Body* b2 = m_body2;

	b2Assert(b2 == NULL);

	// Get initial delta position and angle
	m_dp = b1->GetXForm().position;
//	m_a = b2->GetAngle() - b1->GetAngle();
//	m_R0 = b2MulT(b1->GetXForm().R, b2->GetXForm().R);

	// Reset accumulators
	m_lambda_a = 0.0f;
	m_lambda_p.Set(0.0f, 0.0f);
	m_lambda_p_a = 0.0f;
}

void b2FrictionJoint::InitVelocityConstraints(const b2TimeStep& step)
{
	// Get bodies
	b2Body* b1 = m_body1;
	//b2Body* b2 = m_body2;

	// Get d for this step
	m_d = m_dp - b1->m_sweep.localCenter;// + b2Mul(m_R0, b2->m_sweep.localCenter);

	// Calculate effective mass for angle constraint
	float32 invMass = b1->m_invMass ;//+ b2->m_invMass;
	b2Assert(invMass > B2_FLT_EPSILON);
	m_mass = 1.0f / invMass;

	// Calculate effective inertia for angle constraint
	float32 invInertia = b1->m_invI;
	b2Assert(invInertia > B2_FLT_EPSILON);
	m_inertia = 1.0f / invInertia;

	if (step.warmStarting)
	{
		// Take results of previous frame for angular constraint
		b1->m_angularVelocity -= b1->m_invI * m_lambda_a;

		// Take results of previous frame for position constraint
		float32 s = sinf(b1->m_sweep.a), c = cosf(b1->m_sweep.a);
		b2Vec2 A(-s * m_d.x - c * m_d.y, c * m_d.x - s * m_d.y);
	//	b1->m_linearVelocity -= b1->m_invMass * m_lambda_p;
	//	b1->m_angularVelocity -= b1->m_invI * b2Dot(m_lambda_p, A);
	}
	else
	{
		// Reset accumulators
		m_lambda_a = 0.0f;
		m_lambda_p.Set(0.0f, 0.0f);
		m_lambda_p_a = 0.0f;
	}
}

void b2FrictionJoint::SolveVelocityConstraints(const b2TimeStep& step)
{
	B2_NOT_USED(step);

	// Get bodies
	b2Body* b1 = m_body1;


	b2Vec2 lambda_p(b1->m_mass * b1->m_linearVelocity  );


	//Clamp down the force applied

	b2Vec2 lambda_f = step.inv_dt * lambda_p;
	lambda_f = b1->GetLocalVector(lambda_f);
	lambda_f = b2Clamp(lambda_f, -m_frictionForce, m_frictionForce);
	lambda_f = b1->GetWorldVector(lambda_f);
	lambda_p = step.dt * lambda_f;

	m_lambda_p += lambda_p;

	b1->m_linearVelocity -= b1->m_invMass * lambda_p;
//	b1->m_angularVelocity -= b1->m_invI * lambda_p_a;
}

bool b2FrictionJoint::SolvePositionConstraints(float32 baumgarte)
{
	B2_NOT_USED(baumgarte);


	return false;

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
	return inv_dt * (m_lambda_a + m_lambda_p_a);
}
