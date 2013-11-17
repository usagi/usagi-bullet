/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"
#include <memory>
#include <tuple>
#include <iostream>
///-----includes_end-----

/// This is a Hello World program for running a basic Bullet physics simulation

int main()
{
  ///-----initialization_start-----

  ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
  std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration(new btDefaultCollisionConfiguration());

  ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  std::unique_ptr<btCollisionDispatcher> dispatcher(new btCollisionDispatcher(collisionConfiguration.get()));

  ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
  std::unique_ptr<btBroadphaseInterface> overlappingPairCache(new btDbvtBroadphase());

  ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
  std::unique_ptr<btSequentialImpulseConstraintSolver> solver(new btSequentialImpulseConstraintSolver);

  std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld
  ( new btDiscreteDynamicsWorld
    ( dispatcher.get()
    , overlappingPairCache.get()
    , solver.get()
    , collisionConfiguration.get()
    )
  );

  dynamicsWorld->setGravity(btVector3(0,-10,0));

  ///-----initialization_end-----

  ///create a few basic rigid bodies
  std::unique_ptr<btCollisionShape> groundShape
  ( new btBoxShape
    ( btVector3( btScalar(50.), btScalar(50.), btScalar(50.) )
    )
  );

  //keep track of the shapes, we release memory at exit.
  //make sure to re-use collision shapes among rigid bodies whenever possible!
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  collisionShapes.push_back(groundShape.get());

  btTransform groundTransform;
  groundTransform.setIdentity();
  groundTransform.setOrigin(btVector3(0,-56,0));

  auto body_1_dtor = [&](){
    btScalar mass(0.);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
      groundShape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    std::unique_ptr<btDefaultMotionState> myMotionState(new btDefaultMotionState(groundTransform));
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState.get(), groundShape.get(), localInertia);
    std::unique_ptr<btRigidBody> body(new btRigidBody(rbInfo));

    //add the body to the dynamics world
    dynamicsWorld->addRigidBody(body.get());
    
    return std::make_tuple(std::move(myMotionState), std::move(body));
  }();


  //create a dynamic rigidbody

  //btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
  std::unique_ptr<btCollisionShape> colShape(new btSphereShape(btScalar(1.)));
  collisionShapes.push_back(colShape.get());
  
  auto body_2_dtor = [&](){
    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar  mass(1.f);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
      colShape->calculateLocalInertia(mass,localInertia);

      startTransform.setOrigin(btVector3(2,10,0));
    
      //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
      std::unique_ptr<btDefaultMotionState> myMotionState(new btDefaultMotionState(startTransform));
      btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState.get(), colShape.get(), localInertia);
      std::unique_ptr<btRigidBody> body(new btRigidBody(rbInfo));

      dynamicsWorld->addRigidBody(body.get());
    
    return std::make_tuple(std::move(myMotionState), std::move(body));
  }();



/// Do some simulation


  ///-----stepsimulation_start-----
  for(auto i = 0; i < 100; ++i)
  {
    dynamicsWorld->stepSimulation(1.f/60.f,10);
    
    //print positions of all objects
    for (int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
    {
      btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
      {
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        std::cout
          << "world pos = "
          << float(trans.getOrigin().getX()) << ","
          << float(trans.getOrigin().getY()) << ","
          << float(trans.getOrigin().getZ()) << "\n"
          ;
      }
    }
  }

  ///-----stepsimulation_end-----
}

