/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

#ifndef TIME_OF_IMPACT_H
#define TIME_OF_IMPACT_H

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
		{
			m_shapeA.SetAsBox(0.1f, 10.0f);
		}

		{
			m_shapeB.SetAsBox(0.25f, 0.5f);
		}
	}

	static Test* Create()
	{
		return new TimeOfImpact;
	}

	void Step(Settings* settings)
	{
		settings->pause = 1;
		Test::Step(settings);
		settings->pause = 0;

		b2Sweep sweepA;
		sweepA.c0.Set(0.0f, 0.0f);
		sweepA.a0 = 0.0f;
		sweepA.c = sweepA.c0;
		sweepA.a = sweepA.a0;
		sweepA.t0 = 0.0f;
		sweepA.localCenter.SetZero();

		b2Sweep sweepB;
		sweepB.c0.Set(-1.0f, 10.0f);
		sweepB.a0 = 0.0f;
		sweepB.c.Set(1.0f, 10.0f);
		sweepB.a = 1.0f;
		sweepB.t0 = 0.0f;
		sweepB.localCenter.SetZero();

		b2TOIInput input;
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.sweepRadiusA = m_shapeA.ComputeSweepRadius(sweepA.localCenter);
		input.sweepRadiusB = m_shapeB.ComputeSweepRadius(sweepB.localCenter);
		input.tolerance = b2_linearSlop;

		float32 toi = b2TimeOfImpact(&input, &m_shapeA, &m_shapeB);

		m_debugDraw.DrawString(5, m_textLine, "toi = %g", (float) toi);
		m_textLine += 15;

		{
			m_debugDraw.DrawPolygon(m_shapeA.m_vertices, m_shapeA.m_vertexCount, b2Color(0.9f, 0.9f, 0.9f));
		}

		b2Vec2 vertices[b2_maxPolygonVertices];
		b2XForm transformB;
		sweepB.GetTransform(&transformB, 0.0f);
		for (int32 i = 0; i < m_shapeB.m_vertexCount; ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
		}
		m_debugDraw.DrawPolygon(vertices, m_shapeB.m_vertexCount, b2Color(0.9f, 0.9f, 0.9f));

		sweepB.GetTransform(&transformB, toi);
		for (int32 i = 0; i < m_shapeB.m_vertexCount; ++i)
		{
			vertices[i] = b2Mul(transformB, m_shapeB.m_vertices[i]);
		}
		m_debugDraw.DrawPolygon(vertices, m_shapeB.m_vertexCount, b2Color(0.5f, 0.7f, 0.9f));
	}

	b2PolygonShape m_shapeA;
	b2PolygonShape m_shapeB;
};

#endif
