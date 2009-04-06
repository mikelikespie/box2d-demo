/*
* Copyright (c) 2009 Erin Catto http://www.gphysics.com
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

#ifndef B2_DYNAMIC_TREE_H
#define B2_DYNAMIC_TREE_H

/// A proxy represents a client object in the dynamic tree.
/// The client should not modify the proxy directly, but rather
/// through the interfaces in b2DynamicTree.
struct b2DynamicTreeProxy
{
	// b2Filter?
	b2AABB aabb;
	void* userData;
};

/// A callback for AABB queries.
class b2QueryCallback
{
public:
	~b2QueryCallback() {}

	/// This function is called for each overlapping AABB.
	/// @return true if the query should continue.
	virtual bool Process(void* userData) = 0;
};

/// Ray-cast input data.
struct b2RayCastInput
{
	b2Vec2 p1, p2;
	float32 maxFraction;
};

/// A callback for ray casts.
class b2RayCastCallback
{
public:
	~b2RayCastCallback() {}

	/// Process a ray-cast. This allows the client to perform an exact ray-cast
	/// against their object (found from the proxyUserData pointer).
	/// @param input the original ray-cast segment with an adjusted maxFraction.
	/// @param maxFraction the clipping parameter, the ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param userData user data associated with the current proxy.
	/// @return the new max fraction. Return 0 to end the ray-cast. Return the input maxFraction to
	/// continue the ray cast. Return a value less than maxFraction to clip the ray-cast.
	virtual float32 Process(const b2RayCastInput& input, void* userData) = 0;
};

/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
class b2DynamicTree
{
	b2DynamicTree();
	~b2DynamicTree();

	void Initialize(int32 count);

	/// Create a proxy. Provide a tight fitting AABB and a userData pointer.
	int32 CreateProxy(const b2AABB& aabb, void* userData);

	/// Destroy a proxy. This asserts if the id is invalid.
	void DestroyProxy(int32 proxyId);

	/// Move a proxy. If the proxy has moved outside of its fattened AABB,
	/// then the proxy is removed from the tree and re-inserted. Otherwise
	/// the function returns immediately.
	void MoveProxy(int32 proxyId, const b2AABB& aabb);

	/// Get a proxy.
	/// @return the proxy or NULL if the id is invalid.
	b2DynamicTreeProxy* GetProxy(int32 proxyId);

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	void Query(const b2AABB& aabb, b2QueryCallback* callback);

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	void RayCast(const b2RayCastInput& input, b2RayCastCallback* callback);
};

#endif
