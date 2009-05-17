/*
* Copyright (c) 2007 Erin Catto http://www.gphysics.com
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

#include "b2Collision.h"
#include "Shapes/b2CircleShape.h"
#include "Shapes/b2PolygonShape.h"
#include "Shapes/b2EdgeShape.h"

int32 b2_maxToiIters = 0;
int32 b2_maxToiRootIters = 0;

#if 1
// This algorithm uses conservative advancement to compute the time of
// impact (TOI) of two shapes.
// Refs: Bullet, Young Kim
float32 b2TimeOfImpact(const b2Shape* shapeA, const b2Sweep& sweepA,
					   const b2Shape* shapeB, const b2Sweep& sweepB)
{
	float32 r1 = shapeA->GetSweepRadius();
	float32 r2 = shapeB->GetSweepRadius();

	b2Assert(sweepA.t0 == sweepB.t0);
	b2Assert(1.0f - sweepA.t0 > B2_FLT_EPSILON);

	float32 t0 = sweepA.t0;
	b2Vec2 v1 = sweepA.c - sweepA.c0;
	b2Vec2 v2 = sweepB.c - sweepB.c0;
	float32 omega1 = sweepA.a - sweepA.a0;
	float32 omega2 = sweepB.a - sweepB.a0;

	float32 alpha = 0.0f;

	b2Vec2 p1, p2;
	const int32 k_maxIterations = 20;	// TODO_ERIN b2Settings
	int32 iter = 0;
	b2Vec2 normal = b2Vec2_zero;
	float32 distance = 0.0f;
	float32 targetDistance = 0.0f;
	for(;;)
	{
		float32 t = (1.0f - alpha) * t0 + alpha;
		b2XForm xf1, xf2;
		sweepA.GetXForm(&xf1, t);
		sweepB.GetXForm(&xf2, t);

		// Get the distance between shapes.
		distance = b2Distance(&p1, &p2, shapeA, xf1, shapeB, xf2);

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (distance > 2.0f * b2_toiSlop)
			{
				targetDistance = 1.5f * b2_toiSlop;
			}
			else
			{
				targetDistance = b2Max(0.05f * b2_toiSlop, distance - 0.5f * b2_toiSlop);
			}
		}

		if (distance - targetDistance < 0.05f * b2_toiSlop || iter == k_maxIterations)
		{
			break;
		}

		normal = p2 - p1;
		normal.Normalize();

		// Compute upper bound on remaining movement.
		float32 approachVelocityBound = b2Dot(normal, v1 - v2) + b2Abs(omega1) * r1 + b2Abs(omega2) * r2;
		if (b2Abs(approachVelocityBound) < B2_FLT_EPSILON)
		{
			alpha = 1.0f;
			break;
		}

		// Get the conservative time increment. Don't advance all the way.
		float32 dAlpha = (distance - targetDistance) / approachVelocityBound;
		//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
		float32 newAlpha = alpha + dAlpha;

		// The shapes may be moving apart or a safe distance apart.
		if (newAlpha < 0.0f || 1.0f < newAlpha)
		{
			alpha = 1.0f;
			break;
		}

		// Ensure significant advancement.
		if (newAlpha < (1.0f + 100.0f * B2_FLT_EPSILON) * alpha)
		{
			break;
		}

		alpha = newAlpha;

		++iter;
	}

	b2_maxToiIters = b2Max(iter, b2_maxToiIters);

	return alpha;
}

#else

// u must be a unit vector
b2Vec2 b2GetSupport(const b2Shape* shape, const b2XForm& xf, const b2Vec2& u)
{
	b2ShapeType	type = shape->GetType();

	switch (type)
	{
	case e_circleShape:
		{
			b2CircleShape* circle = (b2CircleShape*)shape;
			b2Vec2 localP = circle->GetLocalPosition();
			b2Vec2 p = b2Mul(xf, localP);
			float32 radius = circle->GetRadius();
			return p + radius * u;
		}
		
	case e_polygonShape:
		{
			b2PolygonShape* polygon = (b2PolygonShape*)shape;
			return polygon->Support(xf, u);
		}

	case e_edgeShape:
		{
			b2EdgeShape* edge = (b2EdgeShape*)shape;
			return edge->Support(xf, u);
		}
	}

	return b2Vec2_zero;
}

// CCD via the secant method.
float32 b2TimeOfImpact(const b2Shape* shapeA, const b2Sweep& sweepA,
					   const b2Shape* shapeB, const b2Sweep& sweepB)
{
	b2Assert(sweepA.t0 == sweepB.t0);
	b2Assert(1.0f - sweepA.t0 > B2_FLT_EPSILON);

	float32 alpha = 0.0f;

	const int32 k_maxIterations = 1000;	// TODO_ERIN b2Settings
	int32 iter = 0;
	float32 target = 0.0f;
	for(;;)
	{
		b2XForm xfA, xfB;
		sweepA.GetXForm(&xfA, alpha);
		sweepB.GetXForm(&xfB, alpha);

		// Get the distance between shapes.
		b2Vec2 pA, pB;
		float32 distance = b2Distance(&pA, &pB, shapeA, xfA, shapeB, xfB);

		if (distance <= 0.0f)
		{
			alpha = 1.0f;
			break;
		}

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (distance > 2.0f * b2_toiSlop)
			{
				target = 1.5f * b2_toiSlop;
			}
			else
			{
				target = b2Max(0.05f * b2_toiSlop, distance - 0.5f * b2_toiSlop);
			}
		}

		if (b2Abs(distance - target) < 0.03f * b2_toiSlop)
		{
			if (iter == 0)
			{
				alpha = 1.0f;
				break;
			}

			break;
		}

		b2Vec2 n = pB - pA;
		n.Normalize();

		b2Vec2 vA = b2GetSupport(shapeA, xfA, n);
		b2Vec2 vB = b2GetSupport(shapeB, xfB, -n);
		float32 distanceCheck;
		distanceCheck = b2Dot(n, vB - vA);
		
		float32 newAlpha = alpha;
		{
			float32 a1 = alpha, a2 = 1.0f;
			//float32 t1 = t, t2 = 1.0f;

			float32 c1 = distance;

			sweepA.GetXForm(&xfA, a2);
			sweepB.GetXForm(&xfB, a2);
			b2Vec2 vA = b2GetSupport(shapeA, xfA, n);
			b2Vec2 vB = b2GetSupport(shapeB, xfB, -n);
			float32 c2 = b2Dot(n, vB - vA);

			// If intervals don't overlap at t2, then we are done.
			if (c2 >= target)
			{
				newAlpha = 1.0f;
				break;
			}

			// Determine when intervals intersect.
			int32 rootIterCount = 0;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				float32 a;
				if (rootIterCount & 3)
				{
					// Secant rule to improve convergence.
					a = a1 + (target - c1) * (a2 - a1) / (c2 - c1);
				}
				else
				{
					// Bisection to guarantee progress.
					a = 0.5f * (a1 + a2);
				}

				sweepA.GetXForm(&xfA, a);
				sweepB.GetXForm(&xfB, a);
				vA = b2GetSupport(shapeA, xfA, n);
				vB = b2GetSupport(shapeB, xfB, -n);
				float32 c = b2Dot(n, vB - vA);

				if (b2Abs(c - target) < 0.025f * b2_toiSlop)
				{
					newAlpha = a;
					break;
				}

				// Ensure we continue to bracket the root.
				if (c > target)
				{
					a1 = a;
					c1 = c;
				}
				else
				{
					a2 = a;
					c2 = c;
				}

				++rootIterCount;

				b2Assert(rootIterCount < 50);
			}

			b2_maxToiRootIters = b2Max(b2_maxToiRootIters, rootIterCount);
		}

		// Ensure significant advancement.
		if (newAlpha < (1.0f + 100.0f * B2_FLT_EPSILON) * alpha)
		{
			break;
		}

		alpha = newAlpha;

		++iter;

		if (iter == k_maxIterations)
		{
			break;
		}
	}

	if (iter == 6)
	{
		iter += 0;
	}

	b2_maxToiIters = b2Max(b2_maxToiIters, iter);

	return alpha;
}

#endif
