#include "Assignment.h"
#include "Content.h"
#include "VectorDrawer.h"
#include "LazerBeam.h"
#include "FountainEffect.h"
#include "snowEffect.h"
#include "RainEffect.h"
#include "Game.h"
#include "GameComponent.h"
#include "PhysicsFactory.h"
#include "PhysicsController.h"
#include <btBulletDynamicsCommon.h>

using namespace BGE;

Assignment::Assignment(void)
{
	elapsed = 10000;
}


Assignment::~Assignment(void)
{
}

bool Assignment::Initialise()
{
          
		// Set up the collision configuration and dispatcher
		collisionConfiguration = new btDefaultCollisionConfiguration();
		dispatcher = new btCollisionDispatcher(collisionConfiguration);
 
	    // The world.
		btVector3 worldMin(-1000,-1000,-1000);
		btVector3 worldMax(1000,1000,1000);
		broadphase = new btAxisSweep3(worldMin,worldMax);
		solver = new btSequentialImpulseConstraintSolver();
		dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
		dynamicsWorld->setGravity(btVector3(0,-9,0));

		string model = "wizard2";

		wizard = make_shared<GameComponent>();
		wizard->Attach(Content::LoadModel("wizard2", glm::rotate(glm::mat4(1), 90.0f, glm::vec3(0,1,0))));
		wizard->position = glm::vec3(0,8,0);
		wizard->scale = glm::vec3(10,10,10);
		wizard->diffuse = glm::vec3(1.2f, 1.2f, 1.2f);
		Attach(wizard);
		
		shared_ptr<RainEffect> rain = make_shared<RainEffect>();
		Attach(rain);

		shared_ptr<SnowEffect> snow = make_shared<SnowEffect>();
		Attach(snow);
   
        
        riftEnabled = false;
        fullscreen = false;
        width = 800;
        height = 600;

        mass = 1.0f;
        wizard->velocity = glm::vec3(0,0,0);

		int a=20;
		int b=10;
		float x=-20;
		float y=0;
		float z=20;

		physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);
		
		physicsFactory->CreateFromModel(model, glm::vec3(0,8,0),glm::quat(), glm::vec3(10,10,10));
		physicsFactory->CreateCylinder(5,5,glm::vec3(20,0,-20),glm::quat());		
		physicsFactory->CreateStructure("tower",glm::vec3(80,0,80),5,10);
		physicsFactory->CreateStructure("pyramid",glm::vec3(20,0,20),10,10);
		physicsFactory->CreateRagdoll(glm::vec3(-20,20,-20));
		physicsFactory->CreateGroundPhysics();
		physicsFactory->CreateCameraPhysics();
	
        Game::Initialise();

        camera->GetController()->position = glm::vec3(0, 20, 20);
        return true;
}

void Assignment::Update(float timeDelta)
{        
	    // Forces on ship1
        float newtons = 10.0f;
        float epsilon = glm::epsilon<float>();
        if (keyState[SDL_SCANCODE_UP])
        {
				force += wizard->look * newtons;
        }
        if (keyState[SDL_SCANCODE_DOWN])
        {
                force -= wizard->look * newtons;
        }
        if (keyState[SDL_SCANCODE_LEFT])
        {
                force -= wizard->right * newtons;
        }
        if (keyState[SDL_SCANCODE_RIGHT])
        {
                force += wizard->right * newtons;
        }

        // Now calculate the acceleration, new velocity and new position
        glm::vec3 accel = force / mass;
        wizard->velocity += accel * timeDelta;
        wizard->position += wizard->velocity * timeDelta;
        // Check if the velocity length is > epsilon and if so create the look vector from the velocity
        if (glm::length(wizard->velocity) > epsilon)
        {
                wizard->look = glm::normalize(wizard->velocity);                
        }
        // Now check to see if the |look - basis| > epsilon
        // And if so calculate the quaternion
        if (glm::length(wizard->look - GameComponent::basisLook) > epsilon)
        {
                glm::vec3 axis = glm::cross(GameComponent::basisLook, wizard->look);
                axis = glm::normalize(axis);
                float theta = glm::acos(glm::dot(wizard->look, GameComponent::basisLook));
                wizard->orientation = glm::angleAxis(glm::degrees(theta), axis);
        }
        // Apply damping
        wizard->velocity *= 0.99f;
        // Reset the force accumulator
        force = glm::vec3(0,0,0);
		dynamicsWorld->stepSimulation(timeDelta,100);
        Game::Update(timeDelta);

}