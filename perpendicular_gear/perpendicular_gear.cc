
/**
 * @file perpendicular_gear.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-02-28
 *
 * @copyright Copyright (c) 2025
 *
 */
#include <iostream>

#include "PxMaterial.h"              // For materials
#include "PxPhysicsAPI.h"            // Core PhysX functionalities
#include "PxRigidActor.h"            // rigid actor
#include "PxRigidDynamic.h"          // For dynamic rigid bodies
#include "PxScene.h"                 // For scene management
#include "PxShape.h"                 // For shapes
#include "extensions/PxD6Joint.h"    // For D6 joints
#include "foundation/PxTransform.h"  // For transformations
#include "pvd/PxPvd.h"

#define PVD_HOST \
  "127.0.0.1"  // Set this to the IP address of the system running the PhysX
               // Visual Debugger that you want to connect to.

// Function to create a gear
physx::PxRigidActor* createGear(physx::PxPhysics* physics,
                                const physx::PxTransform& transform,
                                float radius, float height) {
  physx::PxRigidDynamic* gear = physics->createRigidDynamic(transform);
  physx::PxShape* shape =
      physics->createShape(physx::PxCapsuleGeometry(radius, height),
                           *physics->createMaterial(0.5f, 0.5f, 0.1f));
  gear->attachShape(*shape);
  shape->release();
  gear->setMass(1.0f);
  return gear;
}

int main() {
  physx::PxDefaultAllocator allocator;
  physx::PxDefaultErrorCallback errorCallback;
  physx::PxFoundation* foundation =
      PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);

  auto pvd = physx::PxCreatePvd(*foundation);
  physx::PxPvdTransport* transport =
      physx::PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
  pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);

  physx::PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation,
                                              physx::PxTolerancesScale());

  physx::PxSceneDesc sceneDesc(physics->getTolerancesScale());
  sceneDesc.gravity = physx::PxVec3(0.0f, 0.0f, -9.81f);
  auto dispatcher = physx::PxDefaultCpuDispatcherCreate(2);
  sceneDesc.cpuDispatcher = dispatcher;
  sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
  physx::PxScene* scene = physics->createScene(sceneDesc);

  scene->setVisualizationParameter(physx::PxVisualizationParameter::eSCALE,
                                   1.0f);
  scene->setVisualizationParameter(physx::PxVisualizationParameter::eACTOR_AXES,
                                   2.0f);

  physx::PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
  if (pvdClient) {
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS,
                               true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
    pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES,
                               true);
  }

  float deltaTime = 0.01;

  // Create two gears
  physx::PxRigidActor* gear1 =
      createGear(physics,
                 physx::PxTransform(
                     physx::PxVec3(0, 0, 0),
                     physx::PxQuat(physx::PxHalfPi, physx::PxVec3(0, 1, 0))),
                 0.5f, 0.1f);
  physx::PxRigidActor* gear2 =
      createGear(physics,
                 physx::PxTransform(
                     physx::PxVec3(0, 0, 1),
                     physx::PxQuat(physx::PxHalfPi, physx::PxVec3(1, 0, 0))),
                 0.5f, 0.1f);

  scene->addActor(*gear1);
  scene->addActor(*gear2);

  // Create a D6 joint to connect the gears
  physx::PxD6Joint* gearJoint = PxD6JointCreate(
      *physics, gear1, physx::PxTransform(), gear2, physx::PxTransform());

  // Lock all motion except for the twist (rotation around Z-axis)
  gearJoint->setMotion(physx::PxD6Axis::eX, physx::PxD6Motion::eLOCKED);
  gearJoint->setMotion(physx::PxD6Axis::eY, physx::PxD6Motion::eLOCKED);
  gearJoint->setMotion(physx::PxD6Axis::eZ, physx::PxD6Motion::eLOCKED);
  gearJoint->setMotion(physx::PxD6Axis::eTWIST, physx::PxD6Motion::eFREE);

  // Set up the driver to enforce gear ratio (example 4:1)
  float gearRatio = 4.0f;
  gearJoint->setDrive(physx::PxD6Drive::eTWIST,
                      physx::PxD6JointDrive(gearRatio, 0, PX_MAX_F32));

  while (true) {
    scene->simulate(deltaTime);
    scene->fetchResults(true);
    // Render your scene here
  }

  scene->release();
  physics->release();
  foundation->release();

  return 0;
}
