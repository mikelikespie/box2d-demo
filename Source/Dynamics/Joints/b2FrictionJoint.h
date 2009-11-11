#ifndef B2_LINEAR_FRICTION_JOINT_H
#define B2_LINEAR_FRICTION_JOINT_H

#include "b2Joint.h"

/// FrictionJoint: simulated dry friction in a direction
/// the frictionForce is the max counterforce that can be applied
/// to the body  (so what you'd get if you multiplied a mass by a
/// friction coefficient).
///
/// To simulate tires, you'd want to set the one of the directions
/// to a very high value, and the other to a very low value.
///
/// Real friction probably is more like using the same accumulator for x and y and limiting
/// the magnitude of (x,y), but this is faster (no square roots :))
/// and works for tires and whatnot.

struct b2FrictionJointDef : public b2JointDef
{
	b2FrictionJointDef()
	{
		type = e_frictionJoint;
		frictionForce = b2Vec2_zero;
		frictionTorque = 0.0;
		body2 = NULL;
	}

	b2Vec2 frictionForce;
	float32 frictionTorque;

	/// Initialize the bodies.
	void Initialize(b2Body* body1, const b2Vec2 &_frictionForce, const float32 _frictionTorque = 0.0f);
	///And one if you don't need directional friction
	void Initialize(b2Body* body1, const float32 _frictionTorque = 0.0f);
};

/// A friction joint is not a joint, but just a constraint that applies friction
class b2FrictionJoint : public b2Joint
{
public:

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;

	b2Vec2 GetAnchor1() const;
	b2Vec2 GetAnchor2() const;


	//--------------- Internals Below -------------------

	b2FrictionJoint(const b2FrictionJointDef* data);

	void InitVelocityConstraints(const b2TimeStep& step);
	void SolveVelocityConstraints(const b2TimeStep& step);
	bool SolvePositionConstraints(float32 baumgarte);

	b2Vec2 m_frictionForce;
	float32 m_frictionTorque;

	// Accumulated impulse for warm starting and returning the constraint force/torque
	b2Vec2 m_lambda_p;

	float32 m_lambda_a_p;
};

#endif
