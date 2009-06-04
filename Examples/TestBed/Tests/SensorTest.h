/*
* Copyright (c) 2008-2009 Erin Catto http://www.gphysics.com
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

#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

// This is used to test sensor shapes.
class SensorTest : public Test
{
public:

	SensorTest()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);

			b2Body* ground = m_world->CreateBody(&bd);

			{
				b2PolygonDef sd;
				sd.SetAsBox(50.0f, 10.0f);
				ground->CreateFixture(&sd);
			}

#if 0
			{
				b2PolygonDef sd;
				sd.SetAsBox(10.0f, 2.0f, b2Vec2(0.0f, 20.0f), 0.0f);
				sd.isSensor = true;
				m_sensor = ground->CreateFixture(&sd);
			}
#else
			{
				b2CircleDef cd;
				cd.isSensor = true;
				cd.radius = 5.0f;
				cd.localPosition.Set(0.0f, 20.0f);
				m_sensor = ground->CreateFixture(&cd);
			}
#endif
		}

		{
			b2CircleDef sd;
			sd.radius = 1.0f;
			sd.density = 1.0f;

			for (int32 i = 0; i < 7; ++i)
			{
				b2BodyDef bd;
				bd.position.Set(-10.0f + 3.0f * i, 20.0f);

				b2Body* body = m_world->CreateBody(&bd);

				body->CreateFixture(&sd);
				body->SetMassFromShapes();
			}
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			b2Fixture* fixture1 = point->fixtureA;
			b2Fixture* fixture2 = point->fixtureB;
			b2Body* other;

			if (fixture1 == m_sensor)
			{
				other = fixture2->GetBody();
			}
			else if (fixture2 == m_sensor)
			{
				other = fixture1->GetBody();
			}
			else
			{
				continue;
			}

			b2Body* ground = m_sensor->GetBody();

			b2CircleShape* circle = (b2CircleShape*)m_sensor->GetShape();
			b2Vec2 center = ground->GetWorldPoint(circle->m_p);

			b2Vec2 d = center - point->position;
			if (d.LengthSquared() < FLT_EPSILON * FLT_EPSILON)
			{
				continue;
			}

			d.Normalize();
			b2Vec2 F = 100.0f * d;
			other->ApplyForce(F, point->position);
		}
	}

	static Test* Create()
	{
		return new SensorTest;
	}

	b2Fixture* m_sensor;
};

#endif
