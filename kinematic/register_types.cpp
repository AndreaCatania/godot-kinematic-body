
#include "./register_types.h"

#include "kinematic_actor.h"
#include "kinematic_object_3d.h"

void register_kinematic_types() {
	ClassDB::register_class<KinematicObject3D>();                                                                           
	ClassDB::register_class<KinematicActor>();
}

void unregister_kinematic_types() {
}
