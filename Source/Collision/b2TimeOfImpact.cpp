/*
* Copyright (c) 2007-2009 Erin Catto http://www.gphysics.com
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
#include "b2Distance.h"
#include "b2TimeOfImpact.h"
#include "Shapes/b2CircleShape.h"
#include "Shapes/b2PolygonShape.h"
#include "Shapes/b2EdgeShape.h"

int32 b2_maxToiIters = 0;
int32 b2_maxToiRootIters = 0;

#if 1
// This algorithm uses conservative advancement to compute the time of
// impact (TOI) of two shapes.
// Refs: Bullet, Young Kim
template <typename TA, typename TB>
float32 b2TimeOfImpact(const b2TOIInput* input, const TA* shapeA, const TB* shapeB)
{
	b2Sweep sweepA = input->sweepA;
	b2Sweep sweepB = input->sweepB;

	float32 r1 = input->sweepRadiusA;
	float32 r2 = input->sweepRadiusB;

	float32 tolerance = input->tolerance;

	float32 radius = shapeA->m_radius + shapeB->m_radius;

	b2Assert(sweepA.t0 == sweepB.t0);
	b2Assert(1.0f - sweepA.t0 > B2_FLT_EPSILON);

	b2Vec2 v1 = sweepA.c - sweepA.c0;
	b2Vec2 v2 = sweepB.c - sweepB.c0;
	float32 omega1 = sweepA.a - sweepA.a0;
	float32 omega2 = sweepB.a - sweepB.a0;

	float32 alpha = 0.0f;

	b2DistanceInput distanceInput;
	distanceInput.useRadii = false;
	b2SimplexCache cache;
	cache.count = 0;

	b2Vec2 p1, p2;
	const int32 k_maxIterations = 1000;	// TODO_ERIN b2Settings
	int32 iter = 0;
	b2Vec2 normal = b2Vec2_zero;
	float32 distance = 0.0f;
	float32 targetDistance = 0.0f;
	for(;;)
	{
		b2XForm xf1, xf2;
		sweepA.GetTransform(&xf1, alpha);
		sweepB.GetTransform(&xf2, alpha);

		// Get the distance between shapes.
		distanceInput.transformA = xf1;
		distanceInput.transformB = xf2;
		b2DistanceOutput distanceOutput;
		b2Distance(&distanceOutput, &cache, &distanceInput, shapeA, shapeB);
		distance = distanceOutput.distance;
		p1 = distanceOutput.pointA;
		p2 = distanceOutput.pointB;

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (distance > radius)
			{
				targetDistance = b2Max(radius - tolerance, 0.75f * radius);
			}
			else
			{
				targetDistance = b2Max(distance - tolerance, 0.02f * radius);
			}
		}

		if (distance - targetDistance < 0.5f * tolerance || iter == k_maxIterations)
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
#elif 0

// CCD via the secant method.
template <typename TA, typename TB>
float32 b2TimeOfImpact(const b2TOIInput* input, const TA* shapeA, const TB* shapeB)
{
	b2Assert(sweepA.t0 == sweepB.t0);
	b2Assert(1.0f - sweepA.t0 > B2_FLT_EPSILON);

	float32 alpha = 0.0f;

	const int32 k_maxIterations = 1000;	// TODO_ERIN b2Settings
	int32 iter = 0;
	float32 target = 0.0f;

	// Prepare input for distance query.
	b2SimplexCache cache;
	cache.count = 0;
	b2DistanceInput distanceInput;
	distanceInput.useRadii = false;

	for(;;)
	{
		b2XForm xfA, xfB;
		sweepA.GetTransform(&xfA, alpha);
		sweepB.GetTransform(&xfB, alpha);

		// Get the distance between shapes.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b2DistanceOutput distanceOutput;
		b2Distance(&distanceOutput, &cache, &distanceInput, shapeA, shapeB);
		float32 distance = distanceOutput.distance;
		b2Vec2 pA = distanceOutput.pointA;
		b2Vec2 pB = distanceOutput.pointB;

		if (distance <= 0.0f)
		{
			alpha = 1.0f;
			break;
		}

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (distance > 2.0f * tolerance)
			{
				target = 1.5f * tolerance;
			}
			else
			{
				target = b2Max(0.05f * tolerance, distance - 0.5f * tolerance);
			}
		}

		if (b2Abs(distance - target) < 0.03f * tolerance)
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

		b2Vec2 nA = b2MulT(xfA.R,  n);
		b2Vec2 nB = b2MulT(xfB.R, -n);
		b2Vec2 vA = b2Mul(xfA, shapeA->GetSupportVertex(nA));
		b2Vec2 vB = b2Mul(xfB, shapeB->GetSupportVertex(nB));
		float32 distanceCheck;
		distanceCheck = b2Dot(n, vB - vA);
		
		float32 newAlpha = alpha;
		{
			float32 a1 = alpha, a2 = 1.0f;

			float32 c1 = distance;

			sweepA.GetTransform(&xfA, a2);
			sweepB.GetTransform(&xfB, a2);
			b2Vec2 vA = b2Mul(xfA, shapeA->GetSupportVertex(nA));
			b2Vec2 vB = b2Mul(xfB, shapeB->GetSupportVertex(nB));
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

				sweepA.GetTransform(&xfA, a);
				sweepB.GetTransform(&xfB, a);
				b2Vec2 vA = b2Mul(xfA, shapeA->GetSupportVertex(nA));
				b2Vec2 vB = b2Mul(xfB, shapeB->GetSupportVertex(nB));
				float32 c = b2Dot(n, vB - vA);

				if (b2Abs(c - target) < 0.025f * tolerance)
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

#else

template <typename TA, typename TB>
float32 b2FeatureSeparation(const b2SimplexCache* cache,
							const TA* shapeA, const b2XForm& transformA,
							const TB* shapeB, const b2XForm& transformB,
							const b2Vec2& axis)
{
	int32 count = cache->count;
	b2Assert(0 < count && count < 3);

	if (count == 1)
	{
		b2Vec2 pointA = b2Mul(transformA, shapeA->GetVertex(cache->indexA[0]));
		b2Vec2 pointB = b2Mul(transformB, shapeB->GetVertex(cache->indexB[0]));
		float32 separation = b2Dot(pointB - pointA, axis);
		return separation;
	}

	if (count == 2)
	{
		if (cache->indexA[0] == cache->indexA[1])
		{
			b2Assert(cache->indexB[0] != cache->indexB[1]);
			int32 indexB1 = cache->indexB[0];
			int32 indexB2 = cache->indexB[1];
			if (indexB2 < indexB1)
			{
				b2Swap(indexB1, indexB2);
			}

			b2Vec2 localPointB1 = shapeB->GetVertex(indexB1);
			b2Vec2 localPointB2 = shapeB->GetVertex(indexB2);

			b2Vec2 localEdge = localPointB2 - localPointB1;
			localEdge.Normalize();

			b2Vec2 edge = b2Mul(transformB.R, localEdge);
			b2Vec2 pointB1 = b2Mul(transformB, localPointB1);

			float32 separation = B2_FLT_MAX;
			for (int32 i = 0; i < shapeA->GetVertexCount(); ++i)
			{
				b2Vec2 pointA = b2Mul(transformA, shapeA->GetVertex(i));
				float32 s = b2Cross(pointA - pointB1, edge);
				separation = b2Min(separation, s);
			}
			return separation;
		}

		b2Assert(cache->indexA[0] != cache->indexA[1]);
		int32 indexA1 = cache->indexA[0];
		int32 indexA2 = cache->indexA[1];
		if (indexA2 < indexA1)
		{
			b2Swap(indexA1, indexA2);
		}

		b2Vec2 localPointA1 = shapeA->GetVertex(indexA1);
		b2Vec2 localPointA2 = shapeA->GetVertex(indexA2);

		b2Vec2 localEdge = localPointA2 - localPointA1;
		localEdge.Normalize();

		b2Vec2 edge = b2Mul(transformA.R, localEdge);
		b2Vec2 pointA1 = b2Mul(transformA, localPointA1);

		float32 separation = B2_FLT_MAX;
		for (int32 i = 0; i < shapeB->GetVertexCount(); ++i)
		{
			b2Vec2 pointB = b2Mul(transformB, shapeB->GetVertex(i));
			float32 s = b2Cross(pointB - pointA1, edge);
			separation = b2Min(separation, s);
		}
		return separation;
	}

	return 0.0f;
}

// CCD via the secant method.
template <typename TA, typename TB>
float32 b2TimeOfImpact(const b2TOIInput* input, const TA* shapeA, const TB* shapeB)
{
	b2Sweep sweepA = input->sweepA;
	b2Sweep sweepB = input->sweepB;

	b2Assert(sweepA.t0 == sweepB.t0);
	b2Assert(1.0f - sweepA.t0 > B2_FLT_EPSILON);

	float32 tolerance = input->tolerance;

	float32 alpha = 0.0f;

	const int32 k_maxIterations = 1000;	// TODO_ERIN b2Settings
	int32 iter = 0;
	float32 target = 0.0f;

	// Prepare input for distance query.
	b2SimplexCache cache;
	cache.count = 0;
	b2DistanceInput distanceInput;
	distanceInput.useRadii = false;

	for(;;)
	{
		b2XForm xfA, xfB;
		sweepA.GetTransform(&xfA, alpha);
		sweepB.GetTransform(&xfB, alpha);

		// Get the distance between shapes.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b2DistanceOutput distanceOutput;
		b2Distance(&distanceOutput, &cache, &distanceInput, shapeA, shapeB);

		if (distanceOutput.distance <= 0.0f)
		{
			alpha = 1.0f;
			break;
		}

		b2Vec2 n = distanceOutput.pointB - distanceOutput.pointA;
		n.Normalize();

		float32 separation = b2FeatureSeparation(&cache, shapeA, xfA, shapeB, xfB, n);
		if (separation <= 0.0f)
		{
			alpha = 1.0f;
			break;
		}

		if (iter == 0)
		{
			// Compute a reasonable target distance to give some breathing room
			// for conservative advancement.
			if (separation > 2.0f * tolerance)
			{
				target = 1.5f * tolerance;
			}
			else
			{
				target = b2Max(0.05f * tolerance, separation - 0.5f * tolerance);
			}
		}

		if (separation - target < 0.03f * tolerance)
		{
			if (iter == 0)
			{
				alpha = 1.0f;
				break;
			}

			break;
		}

		float32 newAlpha = alpha;
		{
			float32 a1 = alpha, a2 = 1.0f;

			float32 c1 = separation;

			sweepA.GetTransform(&xfA, a2);
			sweepB.GetTransform(&xfB, a2);
			float32 c2 = b2FeatureSeparation(&cache, shapeA, xfA, shapeB, xfB, n);

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

				sweepA.GetTransform(&xfA, a);
				sweepB.GetTransform(&xfB, a);

				float32 c = b2FeatureSeparation(&cache, shapeA, xfA, shapeB, xfB, n);

				if (b2Abs(c - target) < 0.025f * tolerance)
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

template float32
b2TimeOfImpact(const b2TOIInput* input, const b2CircleShape* shapeA, const b2CircleShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input, const b2CircleShape* shapeA, const b2EdgeShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input, const b2CircleShape* shapeA, const b2PolygonShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input,	const b2EdgeShape* shapeA, const b2CircleShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input,	const b2EdgeShape* shapeA, const b2EdgeShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input,	const b2EdgeShape* shapeA, const b2PolygonShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input,	const b2PolygonShape* shapeA, const b2CircleShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input,	const b2PolygonShape* shapeA, const b2EdgeShape* shapeB);

template float32
b2TimeOfImpact(const b2TOIInput* input,	const b2PolygonShape* shapeA, const b2PolygonShape* shapeB);

