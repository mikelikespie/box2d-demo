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

#ifndef DISTANCE_TEST_H
#define DISTANCE_TEST_H

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, 20.0f);
			m_body1 = m_world->CreateBody(&bd);

			b2PolygonDef sd;
			sd.SetAsBox(0.1f, 10.0f, b2Vec2(-10.0f, 0.0f), 0.0f);
			m_shape1 = m_body1->CreateShape(&sd);
		}

		{
			b2BodyDef bd;
			bd.position.Set(-9.1892055008530633f, 17.037377814153160f);
			bd.angle = -34.723153436328857f;
			m_body2 = m_world->CreateBody(&bd);

			b2PolygonDef sd;
			sd.SetAsBox(0.1f, 4.0f);
			m_shape2 = m_body2->CreateShape(&sd);
		}

		m_world->SetGravity(b2Vec2(0.0f, 0.0f));
	}

	~DistanceTest()
	{
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		b2Vec2 x1, x2;
		float32 distance = b2Distance(&x1, &x2, m_shape1, m_body1->GetXForm(), m_shape2, m_body2->GetXForm());

		m_debugDraw.DrawString(5, m_textLine, "distance = %g", (float) distance);
		m_textLine += 15;

		extern int32 g_GJK_Iterations;

		m_debugDraw.DrawString(5, m_textLine, "iterations = %d", g_GJK_Iterations);
		m_textLine += 15;

		glPointSize(4.0f);
		glColor4f(1.0f, 0.0f, 0.0f,1);
		glBegin(GL_POINTS);
		glVertex2f(x1.x, x1.y);
		glVertex2f(x2.x, x2.y);
		glEnd();
		glPointSize(1.0f);

		glColor4f(1.0f, 1.0f, 0.0f,1);
		glBegin(GL_LINES);
		glVertex2f(x1.x, x1.y);
		glVertex2f(x2.x, x2.y);
		glEnd();
	}

	void Keyboard(unsigned char key)
	{
		b2Vec2 p = m_body2->GetPosition();
		float32 a = m_body2->GetAngle();

		switch (key)
		{
		case 'a':
			p.x -= 0.1f;
			break;

		case 'd':
			p.x += 0.1f;
			break;

		case 's':
			p.y -= 0.1f;
			break;

		case 'w':
			p.y += 0.1f;
			break;

		case 'q':
			a += 0.1f * b2_pi;
			break;

		case 'e':
			a -= 0.1f * b2_pi;
			break;
		}

		m_body2->SetXForm(p, a);
	}

	b2Body* m_body1;
	b2Body* m_body2;
	b2Shape* m_shape1;
	b2Shape* m_shape2;
};

#endif
