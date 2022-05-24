
#include "kinematic_object_3d.h"

#include "core/engine.h"
#include "core/local_vector.h"
#include "modules/bullet/bullet_physics_server.h"
#include "modules/bullet/bullet_types_converter.h"
#include "modules/bullet/godot_result_callbacks.h"
#include "modules/bullet/shape_bullet.h"
#include "scene/3d/collision_shape.h"
#include "scene/3d/mesh_instance.h"
#include "scene/resources/box_shape.h"
#include "scene/resources/capsule_shape.h"
#include "scene/resources/cylinder_shape.h"
#include "scene/resources/ray_shape.h"
#include "scene/resources/sphere_shape.h"
#include "servers/physics_server.h"

#ifdef TOOLS_ENABLED
#include "plugins.h"

static bool gizmo_registered = false;
#endif

KinematicObject3D::KinematicObject3D() :
		Spatial() {
	// Make sure BulletPhysics engine is used.
	BulletPhysicsServer *singleton = dynamic_cast<BulletPhysicsServer *>(PhysicsServer::get_singleton());
	CRASH_COND_MSG(singleton == nullptr, "The KinematicObjct3D can work only using BulletPhysicsEngine.");

#ifdef TOOLS_ENABLED
	// Register the gizmo plugin.
	if (gizmo_registered == false) {
		if (SpatialEditor::get_singleton() != nullptr) {
			SpatialEditor::get_singleton()->add_gizmo_plugin(Ref<KinematicObject3DGizmoPlugin>(memnew(KinematicObject3DGizmoPlugin)));
			gizmo_registered = true;
		}
	}
#endif
}

KinematicObject3D::~KinematicObject3D() {
	set_shape(Ref<Shape>()); // Make sure to destroy any internal shape.
	set_in_world(false);
}

RID KinematicObject3D::get_physics_rid() const {
	return physics_rid;
}

void KinematicObject3D::set_entity_id(uint32_t p_entity_id) {
	entity_id = p_entity_id;
	if (collision_object) {
		EntityIDConverter c;
		c.entity_id_uint = entity_id;
		collision_object->setUserIndex3(c.entity_id_int);
	}
}

void KinematicObject3D::set_shape(const Ref<Shape> &p_shape) {
	BulletPhysicsServer *singleton = static_cast<BulletPhysicsServer *>(PhysicsServer::get_singleton());

	if (!shape.is_null()) {
		ShapeBullet *internal_shape = singleton->get_shape_owner()->getornull(shape->get_rid());
		ERR_FAIL_COND_MSG(internal_shape == nullptr, "At this point the shape RID is not supposed to point to a freed object. It's also causing memory leak. Check it!!!!");

		ERR_PRINT_ONCE("TODO please delete the shape using the new function provided by ShapeBullet.");
		delete bt_shape;
		bt_shape = nullptr;

		on_shape_remove(internal_shape);

		shape->unregister_owner(this);
		shape->disconnect("changed", this, "_shape_changed");
	}

	shape = p_shape;

	if (!shape.is_null()) {
		shape->register_owner(this);
		shape->connect("changed", this, "_shape_changed");

		ShapeBullet *internal_shape = singleton->get_shape_owner()->getornull(shape->get_rid());
		ERR_FAIL_COND(internal_shape == nullptr);

		// At this point the p_shape can't never be nullptr.
		CRASH_COND(p_shape == nullptr);

		btCollisionShape *_bt_shape = internal_shape->create_bt_shape(btVector3(1, 1, 1));
		if (_bt_shape->isConvex() == false) {
			ERR_PRINT_ONCE("TODO delete via ShapeBullet");
			delete _bt_shape;
			ERR_FAIL_MSG("The shape must be convex.");
		}

		bt_shape = static_cast<btConvexShape *>(_bt_shape);

		on_shape_set(internal_shape);
	}

	update_physics_body_shape();
	update_physics_body_transform();

	update_gizmo();
	if (is_inside_tree()) {
		_shape_changed();
	}
	update_configuration_warning();
}

Ref<Shape> KinematicObject3D::get_shape() const {
	return shape;
}

void KinematicObject3D::set_in_world(bool p_in_world) {
	in_world = p_in_world;
	if (in_world) {
		if (!physics_rid.is_valid()) {
			physics_rid = PhysicsServer::get_singleton()->body_create(PhysicsServer::BODY_MODE_KINEMATIC, true);
			PhysicsServer::get_singleton()->body_attach_object_instance_id(physics_rid, get_instance_id());
			if (get_world().is_valid()) {
				RID space = get_world()->get_space();
				PhysicsServer::get_singleton()->body_set_space(physics_rid, space);
			}
		}
		BulletPhysicsServer *singleton = static_cast<BulletPhysicsServer *>(PhysicsServer::get_singleton());
		collision_object = singleton->get_collision_object(physics_rid)->get_bt_collision_object();
		update_physics_body_shape();
		set_entity_id(entity_id);
	} else {
		if (physics_rid.is_valid()) {
			PhysicsServer::get_singleton()->free(physics_rid);
		}
		physics_rid = RID();
		collision_object = nullptr;
	}

	//set_notify_transform(in_world);
}

bool KinematicObject3D::get_in_world() const {
	return in_world;
}

void KinematicObject3D::set_collision_layer(uint32_t p_layer) {
	collision_layer = p_layer;
}

uint32_t KinematicObject3D::get_collision_layer() const {
	return collision_layer;
}

void KinematicObject3D::set_collision_mask(uint32_t p_mask) {
	collision_mask = p_mask;
}

uint32_t KinematicObject3D::get_collision_mask() const {
	return collision_mask;
}

void KinematicObject3D::set_collision_mask_bit(int p_bit, bool p_value) {
	uint32_t mask = get_collision_mask();
	if (p_value) {
		mask |= 1 << p_bit;
	} else {
		mask &= ~(1 << p_bit);
	}
	set_collision_mask(mask);
}

bool KinematicObject3D::get_collision_mask_bit(int p_bit) const {
	return get_collision_mask() & (1 << p_bit);
}

BtKinematicConvexQResult KinematicObject3D::test_motion(const btConvexShape *p_shape, const btVector3 &p_position, const btVector3 &p_motion, real_t p_margin, bool p_skip_if_moving_away) const {
	BtKinematicConvexQResult result(
			collision_object,
			p_motion.isZero() ? btVector3(0.0, 0.0, 0.0) : p_motion.normalized(),
			p_skip_if_moving_away);

	ERR_FAIL_COND_V(p_shape == nullptr, result);

	result.m_collisionFilterGroup = 0;
	result.m_collisionFilterMask = collision_mask;

	space->dynamicsWorld->convexSweepTest(
			p_shape,
			btTransform(btMatrix3x3::getIdentity(), p_position),
			btTransform(btMatrix3x3::getIdentity(), p_position + p_motion),
			result,
			p_margin);

	return result;
}

BtKinematicConvexQResult KinematicObject3D::test_motion_target(const btConvexShape *p_shape, const btVector3 &p_position, const btVector3 &p_target, real_t p_margin, bool p_skip_if_moving_away) const {
	return test_motion(p_shape, p_position, p_target - p_position, p_margin, p_skip_if_moving_away);
}

BtKinematicContactQResult KinematicObject3D::test_contact(btConvexShape *p_shape, const btVector3 &p_position, real_t p_margin, bool p_smooth_results) const {
	// Note: I'm not using the collision_object because I don't want to change
	// the main object transform. If turns out that this query is slow, we must
	// reconsider this.
	btCollisionObject query_collision_object;
	BtKinematicContactQResult result(collision_object, &query_collision_object);
	result.smooth_results = p_smooth_results;

	ERR_FAIL_COND_V(p_shape == nullptr, result);

	query_collision_object.setCollisionShape(p_shape);
	query_collision_object.setWorldTransform(btTransform(btMatrix3x3::getIdentity(), p_position));

	result.m_collisionFilterGroup = 0;
	result.m_collisionFilterMask = collision_mask;
	result.m_closestDistanceThreshold = p_margin;

	space->dynamicsWorld->contactTest(
			&query_collision_object,
			result);

	return result;
}

BtKinematicRayQResult KinematicObject3D::test_ray(const btVector3 &p_from, const btVector3 &p_to) const {
	BtKinematicRayQResult result(collision_object, p_from, p_to);

	result.m_collisionFilterGroup = 0;
	result.m_collisionFilterMask = collision_mask;

	space->dynamicsWorld->rayTest(p_from, p_to, result);
	return result;
}

void KinematicObject3D::on_shape_remove(ShapeBullet *p_shape) {
}

void KinematicObject3D::on_shape_set(ShapeBullet *p_shape) {
}

void KinematicObject3D::set_collision_layer_bit(int p_bit, bool p_value) {
	uint32_t mask = get_collision_layer();
	if (p_value) {
		mask |= 1 << p_bit;
	} else {
		mask &= ~(1 << p_bit);
	}
	set_collision_layer(mask);
}

bool KinematicObject3D::get_collision_layer_bit(int p_bit) const {
	return get_collision_layer() & (1 << p_bit);
}

void KinematicObject3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_physics_rid"), &KinematicObject3D::get_physics_rid);
	ClassDB::bind_method(D_METHOD("set_entity_id"), &KinematicObject3D::set_entity_id);

	ClassDB::bind_method(D_METHOD("set_shape", "shape"), &KinematicObject3D::set_shape);
	ClassDB::bind_method(D_METHOD("get_shape"), &KinematicObject3D::get_shape);

	ClassDB::bind_method(D_METHOD("set_in_world", "in_world"), &KinematicObject3D::set_in_world);
	ClassDB::bind_method(D_METHOD("get_in_world"), &KinematicObject3D::get_in_world);

	ClassDB::bind_method(D_METHOD("set_collision_layer", "layer"), &KinematicObject3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &KinematicObject3D::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "mask"), &KinematicObject3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &KinematicObject3D::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_collision_mask_bit", "bit", "value"), &KinematicObject3D::set_collision_mask_bit);
	ClassDB::bind_method(D_METHOD("get_collision_mask_bit", "bit"), &KinematicObject3D::get_collision_mask_bit);

	ClassDB::bind_method(D_METHOD("set_collision_layer_bit", "bit", "value"), &KinematicObject3D::set_collision_layer_bit);
	ClassDB::bind_method(D_METHOD("get_collision_layer_bit", "bit"), &KinematicObject3D::get_collision_layer_bit);

	ClassDB::bind_method(D_METHOD("update_physics_body_transform"), &KinematicObject3D::update_physics_body_transform);

	ClassDB::bind_method(D_METHOD("_update_debug_shape"), &KinematicObject3D::_update_debug_shape);
	ClassDB::bind_method(D_METHOD("resource_changed", "resource"), &KinematicObject3D::resource_changed);

	ClassDB::bind_method(D_METHOD("_shape_changed"), &KinematicObject3D::_shape_changed);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "shape", PROPERTY_HINT_RESOURCE_TYPE, "Shape"), "set_shape", "get_shape");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "in_world"), "set_in_world", "get_in_world");

	ADD_GROUP("Collision", "collision_");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
}

void KinematicObject3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			// Search collision shape.
			if (Engine::get_singleton()->is_editor_hint()) {
				// Editor only
			} else {
				ERR_FAIL_COND(get_world().is_null());
				ERR_FAIL_COND(get_world()->get_space().is_valid() == false);

				BulletPhysicsServer *singleton = static_cast<BulletPhysicsServer *>(PhysicsServer::get_singleton());
				ERR_FAIL_COND(singleton == nullptr);

				space = singleton->get_space_owner()->getornull(get_world()->get_space());
				ERR_FAIL_COND(space == nullptr);

				if (physics_rid.is_valid()) {
					singleton->body_set_space(physics_rid, get_world()->get_space());
				}
			}
#ifdef TOOLS_ENABLED
			_update_debug_shape();
#endif
		} break;
			//case NOTIFICATION_TRANSFORM_CHANGED: {
			//	if (in_world) {
			//		update_physics_body_transform();
			//	}
			//} break;
	}
}

String KinematicObject3D::get_configuration_warning() const {
	if (shape.is_null()) {
		return TTR("Please add a shape.");
	} else if (Object::cast_to<SphereShape>(*shape) == nullptr &&
			Object::cast_to<BoxShape>(*shape) == nullptr &&
			Object::cast_to<CapsuleShape>(*shape) == nullptr &&
			Object::cast_to<CylinderShape>(*shape) == nullptr &&
			Object::cast_to<RayShape>(*shape) == nullptr) {
		return TTR("The set shape is not supported.");
	}

	return String();
}

void KinematicObject3D::update_physics_body_shape() {
	if (!physics_rid.is_valid()) {
		return;
	}

	if (shape.is_null()) {
		PhysicsServer::get_singleton()->body_set_shape(physics_rid, 0, RID());
	} else {
		if (PhysicsServer::get_singleton()->body_get_shape_count(physics_rid) == 0) {
			PhysicsServer::get_singleton()->body_add_shape(physics_rid, shape->get_rid(), Transform(), false);
		} else {
			PhysicsServer::get_singleton()->body_set_shape(physics_rid, 0, shape->get_rid());
		}
	}
}

void KinematicObject3D::update_physics_body_transform() {
	if (!physics_rid.is_valid()) {
		return;
	}
	PhysicsServer::get_singleton()->body_set_state(
			physics_rid,
			PhysicsServer::BODY_STATE_TRANSFORM,
			get_global_transform());
}

void KinematicObject3D::_shape_changed() {
#ifdef TOOLS_ENABLED
	// If this is a heightfield shape our center may have changed
	if (debug_shape_dirty == false) {
		debug_shape_dirty = true;
		call_deferred("_update_debug_shape");
	}
#endif
}

void KinematicObject3D::resource_changed(RES res) {
	update_gizmo();
}

void KinematicObject3D::_update_debug_shape() {
#ifdef TOOLS_ENABLED
	if (is_inside_tree() == false || get_tree()->is_debugging_collisions_hint() == false || debug_shape_dirty == false) {
		return;
	}

	debug_shape_dirty = false;

	if (debug_shape) {
		debug_shape->queue_delete();
		debug_shape = nullptr;
	}

	Ref<Shape> s = get_shape();
	if (s.is_null()) {
		return;
	}

	Ref<Mesh> mesh = s->get_debug_mesh();
	MeshInstance *mi = memnew(MeshInstance);
	mi->set_mesh(mesh);
	add_child(mi);
	debug_shape = mi;
#endif
}
