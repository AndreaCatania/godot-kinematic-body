
#ifndef KINEMATIC_ACTOR_H
#define KINEMATIC_ACTOR_H

// TODO is this too much margin?
// TODO I need to customize this?
// Margin used to unstuck the body.
#define MARGIN 0.01 // TODO Too big, was 0.005. consider use 0.05

#include "kinematic_object_3d.h"

/// The KinematicActor is the node that provide access to some low level queries
/// to perform kinematic motion.
class KinematicActor : public KinematicObject3D {
	GDCLASS(KinematicActor, KinematicObject3D);

	static void _bind_methods();

public:
	struct ShapeHolder {
		class btConvexShape *shape = nullptr;
		class btConvexShape *margin_shape = nullptr;
		Ref<Shape> godot_shape;
	};

	struct StrafingResult {
		const real_t fulfillment;
		const btVector3 motion;
		const btVector3 direction;
		const real_t motion_length;

		static StrafingResult zero() {
			return StrafingResult{ 0.0, btVector3(0, 0, 0), btVector3(0, 0, 0), 0.0 };
		}
	};

public:
	KinematicActor();

	// TODO add a method to create the 'StrafingResult`.

	/// This function performs a series of checks to make the Actor move and
	/// slide on any encountered surface.
	/// The first argument p_reference is used to change the orientation of the
	/// world, so the stepping mechanism can also work on different planes.
	StrafingResult move(
			const ShapeHolder &p_motion_shape,
			const btMatrix3x3 &p_reference,
			btVector3 &r_position,
			const btVector3 &p_motion,
			real_t p_step_height,
			bool p_search_floor) const;

	/// Unstuck the Actor from any collision;
	btVector3 unstuck(
			const ShapeHolder &p_shape,
			btVector3 &r_position,
			const btVector3 &p_up_dir,
			real_t p_margin,
			real_t p_unstuck_factor = 1.0) const;
};

#endif
