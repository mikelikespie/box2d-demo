/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "b2TensorDryFrictionController.h"

b2TensorDryFrictionController::b2TensorDryFrictionController(const b2TensorDryFrictionControllerDef* def) : b2Controller(def)
{
	T = def->T;
	maxCounterForce = def->maxCounterForce;
}

void b2TensorDryFrictionController::Step(const b2TimeStep& step)
{
	float32 timestep = step.dt;
	static int f = 0;


	for(b2ControllerEdge *i=m_bodyList;i;i=i->nextBody){
		b2Body* body = i->body;
		if(body->IsSleeping())
			continue;

		b2Vec2 localVelocity = body->GetLocalVector(body->GetLinearVelocity());
		//The impulse that it would have taken to in N * s
		b2Vec2 currentMomentum = body->GetMass() * localVelocity;

		b2Vec2 localCounterImpulse = b2Mul(T, currentMomentum);

		b2Vec2 localCounterForce = (1.0/step.dt)  * localCounterImpulse;

		b2Vec2 realLocalCounterForce = localCounterForce;//b2Clamp(localCounterForce, -maxCounterForce, maxCounterForce);

		b2Vec2 realLocalCounterImpulse = step.dt * realLocalCounterForce;

		b2Vec2 realLocalCounterVelocity = (1.0/body->GetMass()) * realLocalCounterImpulse;

		//	b2Vec2 realCounterVelocity = body->GetWorldVector(realLocalCounterVelocity);
		// b2Vec2 realCounterVelocity = b2Mul(T, body->GetLinearVelocity());

		//body->SetLinearVelocity(b2Vec2(0,0));

		//body->ApplyForce();
	//	printf("x %f y %f\n", realCounterImpulse.x, realCounterImpulse.y);

		if (!(f % 200)) {

			printf("- v  %f,%f\n", localVelocity.x, localVelocity.y);
			printf("- m     %f,%f\n", currentMomentum.x, currentMomentum.y);
			printf("- i     %f,%f\n", localCounterImpulse.x, localCounterImpulse.y);
			printf("- f        %f,%f\n", localCounterForce.x, localCounterForce.y);
			printf("- f        %f,%f\n", realLocalCounterForce.x, realLocalCounterForce.y);
			printf("- i     %f,%f\n", realLocalCounterImpulse.x, realLocalCounterImpulse.y);
			printf("- v  %f,%f\n\n", realLocalCounterVelocity.x, realLocalCounterVelocity.y);
		}
		f++;



	}
}

void b2TensorDryFrictionControllerDef::SetAxisFrictionForce(float32 xFrictionForce)
{
	T.col1.x = -1;
	T.col1.y = 0;
	T.col2.x = 0;
	T.col2.y = -1.0;

	maxCounterForce.x = xFrictionForce;
	maxCounterForce.y = 0.0f;
}

void b2TensorDryFrictionController::Destroy(b2BlockAllocator* allocator)
{
	allocator->Free(this, sizeof(b2TensorDryFrictionController));
}


b2TensorDryFrictionController* b2TensorDryFrictionControllerDef::Create(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2TensorDryFrictionController));
	return new (mem) b2TensorDryFrictionController(this);
}
