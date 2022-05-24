/* Author: AndreaCatania */

#ifndef KINEMATIC_OBJECT_3D_H
#define KINEMATIC_OBJECT_3D_H

#include "core/local_vector.h"
#include "scene/3d/spatial.h"
#include "thirdparty/bullet/BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "utilities.h"

class CollisionShape3D;
class MeshInstance;
class btCollisionObject;
class ShapeBullet;

/// The KinematicObject is the base class to integrate any kinematic body efficiently.
class KinematicObject3D : public Spatial {
	GDCLASS(KinematicObject3D, Spatial);

protected:
	RID physics_rid;

	// Entity ID.
	uint32_t entity_id = UINT32_MAX;

	Ref<Shape> shape;

	/// When true, a KinematicBody is craeted. This is useful to make the object
	/// interact with the environment.
	bool in_world = false;
	uint32_t collision_layer = 1;
	uint32_t collision_mask = 1;

	/// Collision object, used to represent this object in the Dynamics World.
	/// This may be nullptr when the object doesn't want that the rigid bodies
	/// interact with it.
	// This is constructed via `PhysicsServer3D`, similarly is done with the shape.
	btCollisionObject *collision_object = nullptr;

	/// The dynamic world where this kinematic objec is into.
	// Taken into the Ready notification.
	class SpaceBullet *space = nullptr;

	/// The shape of this kienamtic object.
	// Taken from the `PhysicsServer3D` for fast usage.
	class btConvexShape *bt_shape = nullptr;

#ifdef TOOLS_ENABLED
protected:
	bool debug_shape_dirty = true;
	MeshInstance *debug_shape = nullptr;
#endif

public:
	KinematicObject3D();
	virtual ~KinematicObject3D();

	RID get_physics_rid() const;

	void set_entity_id(uint32_t p_entity_id);

	void set_shape(const Ref<Shape> &p_shape);
	Ref<Shape> get_shape() const;

	void set_in_world(bool p_in_world);
	bool get_in_world() const;

	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const;

	void set_collision_mask(uint32_t p_mask);
	uint32_t get_collision_mask() const;

	void set_collision_layer_bit(int p_bit, bool p_value);
	bool get_collision_layer_bit(int p_bit) const;

	void set_collision_mask_bit(int p_bit, bool p_value);
	bool get_collision_mask_bit(int p_bit) const;

	BtKinematicConvexQResult test_motion(const btConvexShape *p_shape, const btVector3 &p_position, const btVector3 &p_motion, real_t p_margin, bool p_skip_if_moving_away) const;
	BtKinematicConvexQResult test_motion_target(const btConvexShape *p_shape, const btVector3 &p_position, const btVector3 &p_target, real_t p_margin, bool p_skip_if_moving_away) const;

	/// Perform a contact test and returns a result that can contain
	/// up to 3 contacts. The contacts will be the deepest found and also
	/// will be ordered, from deepest to less.
	BtKinematicContactQResult test_contact(btConvexShape *p_shape, const btVector3 &p_position, real_t p_margin, bool p_smooth_results) const;

	/// Perform a test ray from world space position to world space position.
	/// Retuns the closest result.
	BtKinematicRayQResult test_ray(const btVector3 &p_from, const btVector3 &p_to) const;

protected:
	/// Callback solely used to notify shape changes.
	virtual void on_shape_remove(ShapeBullet *p_shape);
	virtual void on_shape_set(ShapeBullet *p_shape);

protected:
	static void _bind_methods();
	void _notification(int p_what);

	virtual String get_configuration_warning() const override;

	void update_physics_body_shape();
	void update_physics_body_transform();

private:
	void _shape_changed();

	void resource_changed(RES res);

protected:
	virtual void _update_debug_shape();
};

#endif
