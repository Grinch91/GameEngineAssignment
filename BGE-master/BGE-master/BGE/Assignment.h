#pragma once
#include "Game.h"
#include "GameComponent.h"
#include "FountainEffect.h"
#include "Game.h"
#include "GameComponent.h"
#include "PhysicsFactory.h"
#include "PhysicsController.h"
#include <btBulletDynamicsCommon.h>

using namespace std;

namespace BGE
{
	class Assignment: public Game
	{

	private:
		btBroadphaseInterface* broadphase;
 
		// Set up the collision configuration and dispatcher
		btDefaultCollisionConfiguration * collisionConfiguration;
		btCollisionDispatcher * dispatcher;
 
		// The actual physics solver
		btSequentialImpulseConstraintSolver * solver;
	public:
		Assignment(void);
		~Assignment(void);
		
		btDiscreteDynamicsWorld * dynamicsWorld;
		std::shared_ptr<PhysicsFactory> physicsFactory;

		shared_ptr<GameComponent> ship1;
		shared_ptr<GameComponent> wizard;
        float elapsed;
        bool Initialise();
        void Update(float timeDelta);

        glm::vec3 force;
        float mass;

	};
}
