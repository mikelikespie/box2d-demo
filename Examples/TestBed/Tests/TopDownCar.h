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

#ifndef TOP_DOWN_CAR_H
#define TOP_DOWN_CAR_H


class MyCar
{
public:
	float32 wheelDims[2];

	float32 halfWidth;
	float32 halfHeight;
	float32 defaultDensity;
	float32 defaultFriction;
	float32 steerForce;
	float32 accelerateForce;

	b2Body* wheels[4];
	b2Body* chassis;
	b2World* world;

	b2RevoluteJoint *frontLeftJoint;

	bool frontWheelDrive;

	MyCar(b2World *_world, float32 width, float32 height) :
		halfWidth(width/2.0f),
		halfHeight(height/2.0f),
		defaultDensity(20.0f),
		defaultFriction(0.68f),
		steerForce(200.0f),
		accelerateForce(1500.0f),
		world(_world),
		frontWheelDrive(true)
	{
		wheelDims[0] = .125;
		wheelDims[1] = .5;

		{
			b2BodyDef chassisBd;

			chassisBd.position = b2Vec2(0.0f, 0.0f);

			chassis = world->CreateBody(&chassisBd);
			b2PolygonDef chassisFixture;

			chassisFixture.SetAsBox(halfWidth, halfHeight);

			chassisFixture.density = defaultDensity;
			chassisFixture.friction = defaultFriction;

			chassis->CreateFixture(&chassisFixture);
			chassis->SetMassFromShapes();
		}

		{
			b2PolygonDef wheelDef;
			wheelDef.SetAsBox(wheelDims[0], wheelDims[1]);
			wheelDef.density = defaultDensity/4.0;
			wheelDef.friction = defaultFriction;

			wheels[0] = makeWheel(-1, 1, wheelDef);
			wheels[1] = makeWheel(1, 1, wheelDef);
			wheels[3] = makeWheel(-1, -1, wheelDef);
			wheels[4] = makeWheel(1, -1, wheelDef);
		}

		{
			//Make the thing that ties the wheels together.
			b2DistanceJointDef rackDef;

			b2Body *w0 = wheels[0];
			b2Body *w1 = wheels[1];

			float32 offset = wheelDims[1] / 2.0f * 2.0f;

			b2Vec2 pos0 = w0->GetWorldCenter();
			b2Vec2 pos1 = w1->GetWorldCenter();

			pos0.y += offset;
			pos1.y += offset;

			rackDef.Initialize(w0, w1, pos0, pos1);

			world->CreateJoint(&rackDef);
		}
	}

	void Steer(float32 dir = 1.0)
	{
		frontLeftJoint->SetMotorSpeed(dir * 2.0);
	}

	void Accelerate(float32 dir = 1.0)
	{
		// Which two wheels do we power?
		int offset = frontWheelDrive ? 0 : 2;

		for (int i = 0; i < 2; i++) {
			b2Body *wheel = wheels[i + offset];
			b2Vec2 force_vector = wheel->GetXForm().R.col2;
			force_vector *= accelerateForce * dir;
			wheel->ApplyForce(force_vector, wheel->GetWorldCenter());
		}
	}

	virtual ~MyCar()
	{
	}

private:

	b2Body* makeWheel(float32 xmul, float32 ymul, b2PolygonDef &wheelDef)
	{
		b2BodyDef bd;
		b2Body *wheel;
		bd.position = b2Vec2(xmul * (halfWidth - wheelDims[0]), ymul * halfHeight);

		wheel = world->CreateBody(&bd);
		wheel->CreateFixture(&wheelDef);
		wheel->SetLinearDamping(0.01f);
		wheel->SetAngularDamping(5.0);
		wheel->SetMassFromShapes();

		b2RevoluteJointDef jointDef;
		jointDef.Initialize(chassis, wheel, wheel->GetWorldCenter());

		// If we're doing the front wheels
		if (ymul > 0) {
			jointDef.lowerAngle = -0.25;
			jointDef.upperAngle = 0.25;
		}

		jointDef.enableLimit = true;

		b2RevoluteJoint *joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);

		if (xmul < 0 && ymul > 0)
		{
			frontLeftJoint = joint;
			frontLeftJoint->EnableMotor(true);
			frontLeftJoint->SetMaxMotorTorque(5000.0f);
		}

		return wheel;
	}

};
class TopDownCar : public Test
{
	MyCar *car;
public:
	TopDownCar()
	{
		m_world->SetGravity(b2Vec2(0.0f, 0.0f));
		car = new MyCar(m_world, 2.0, 4.0);
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		if (keysDown['a'])
			car->Steer(1.0f);
		else if (keysDown['d'])
			car->Steer(-1.0f);
		else
			car->Steer(0.0f);

	}
	void KeyDown(unsigned char key)
	{
		switch (key)
		{
		case 'w':
			{
				car->Accelerate(1.0);
			}
			break;

		case 's':
			{
				car->Accelerate(-1.0);
			}
			break;

		}
	}

	virtual ~TopDownCar()
	{
		delete car;
	}

	static Test* Create()
	{
		return new TopDownCar;
	}

};

#endif
