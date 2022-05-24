
#include "kinematic_actor.h"

#include "modules/bullet/bullet_physics_server.h"
#include "modules/bullet/bullet_types_converter.h"
#include "thirdparty/bullet/LinearMath/btTransform.h"
#include "utilities.h"

#define UNSTUCK_TESTS 4
#define MAX_STRAFING_TESTS 10
#define UNSTUCK_FACTOR 1.0 // TODO use lower value?

void KinematicActor::_bind_methods() {
}

KinematicActor::KinematicActor() :
		KinematicObject3D() {
}

KinematicActor::StrafingResult KinematicActor::move(
		const ShapeHolder &p_motion_shape,
		const btMatrix3x3 &p_reference,
		btVector3 &r_position,
		const btVector3 &p_motion,
		real_t p_step_height,
		bool p_search_floor) const {
#ifdef DEBUG_ENABLED
	// This function is not supposed to be called with null shpae.
	CRASH_COND(p_motion_shape.shape == nullptr);
	CRASH_COND(p_motion_shape.margin_shape == nullptr);
	CRASH_COND(!p_motion_shape.godot_shape.is_valid());
#endif

	const btVector3 up_dir = p_reference * btVector3(0.0, 1.0, 0.0);
	const btVector3 initial_position = r_position;

	const real_t motion_length = p_motion.length();
	real_t remaining_strafing = motion_length;
	btVector3 strafing_dir(0, 0, 0);
	if (motion_length >= CMP_EPSILON) {
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		ERR_PRINT_ONCE("TODO Please add enclosing radius");
		//const real_t radius = p_motion_shape.godot_shape->get_enclosing_radius() + MARGIN;
		const real_t radius = 2.0 + MARGIN;
		strafing_dir = p_motion / motion_length;

		// Unstuck to start from a valid position.
		const btVector3 unstuck_dir = unstuck(p_motion_shape, r_position, up_dir, MARGIN);
		const btVector3 unstuck_motion = (unstuck_dir.length2() <= CMP_EPSILON ? up_dir : unstuck_dir) * p_step_height;
		const btVector3 safe_initial_position = r_position;

		// Phase 1: Unstuck motion
		{
			// Then move along the unstuck_dir, and if there is a collision put the body in between.
			const BtKinematicConvexQResult result = test_motion(p_motion_shape.shape, r_position, unstuck_motion, 0.0, true);
			if (result.hasHit() == false) {
				// Only if no hit.
				r_position += unstuck_motion;
			}
		}

		// Phase 2: Step forward and move.
		{
			// At this point the Actor is into the air, unstucked and free to move.
			// We need to move a certain amount, no more than that.
			// From the initial_position we can establish the ideal target location,
			// just by summing the motion.
			// From the currunt Actor position, we can try to reach that location:
			// - If the Actor doesn't collide, the movement is complete.
			// - If the Actor collides, we need to adjust the target location so to
			//   to make it move.
			// The adjustment changes the motion, to make it parallel to the ground
			// on the next test.
			// The test is repeated but with the new adjusted motion till the full
			// movement is done or the MAX_STRAFING_TESTS is hit.

			// The first strafing just uses the initial motion.
			btVector3 cast_position = r_position;
			//r_position = safe_initial_position; // TODO with this the Actor vibrates, but the actor is much more free to move.
			r_position = initial_position;
			btVector3 target_position = r_position + p_motion;
			bool special_manouvre = false;
			bool special_manouvre2 = false;

			for (int test_n = 0; test_n < MAX_STRAFING_TESTS && remaining_strafing >= CMP_EPSILON; test_n += 1) {
				// Target location from the initial position.
				// Test the motion from the actual up moved position to the target position.
				const BtKinematicConvexQResult result = test_motion_target(
						p_motion_shape.shape,
						cast_position,
						target_position,
						0.0,
						true);

				if (result.hasHit()) {
					const btVector3 hit = cast_position.lerp(
							target_position,
							result.m_closestHitFraction);

					// Comparing the distance between the hit and the target
					// is possible to know if we can consider this hit as moving
					// forward or not. Sometimes happens that the motion goes
					// backward (depending the surface), this prevents such
					// behaviour.
					const btVector3 new_remaining_strafing = target_position - hit;
					const real_t nrs = new_remaining_strafing.length();

					if (nrs < remaining_strafing) {
						const btVector3 strafing_done = hit - r_position;
						const real_t strafing_done_length = strafing_done.length();
						if (strafing_done_length <= 0.0) {
							// No strafing done. nothing more to do.
							break;
						}
						remaining_strafing -= strafing_done_length;

						// Adjust the strafing_motion and try again:
						// Spread the strafing along the perpendicular of the normal.
						const btVector3 nxt_strafing_motion = (strafing_done / strafing_done_length) * remaining_strafing;
						const btVector3 perp = vec_project_perpendicular(nxt_strafing_motion, result.hit_normal);
						const real_t perp_mag = perp.length();

						r_position = hit;

						if (unlikely(perp_mag <= 0.0)) {
							// Can't move anymore, this is the last test.
							break;
						} else {
							strafing_dir = perp / perp_mag;
							target_position = r_position + strafing_dir * remaining_strafing;
							// In this way, if needed we can recompute the downhill adjustment
							special_manouvre2 = false;
						}
					} else {
						// The body went far away instead to get closer.
						if (special_manouvre == false) {
							special_manouvre = true;
							// Make sure to perform the check again.
							test_n -= 1;
							// We are on a wall, Just try to move from the
							// safe_initial_position so to be sure to stay on ground.
							cast_position = safe_initial_position;
						} else {
							// After the safe cast we hit the wall again, this time
							// wre are on the wall without the unstuck_motion.
							// Stop the strafing here.
							break;
						}
					}

				} else {
					if (special_manouvre2 == false && p_search_floor) {
						// This can happen when the Actor is on a flat surface or
						// downhill.
						// In case of downhill, we need to move down in order to
						// avoid speeding up.
						// I'm using two ray casts to establish the slope dir.
						// On flat slopes this is reliable, not differently to
						// others approaches, and works really well with stairs
						// (where the returned normals are always parpendicular
						// to the ground or parallel).
						// The steps case is really difficult to handle well:
						// - A precise motion may slow down the Actor.
						// - A not precise motion may speed up the Actor.
						// This is a balanced way that some times speedup the actor,
						// but not too much.
						// Note:
						// To make this more precise, it's enough perform this check
						// multiple times.

						special_manouvre2 = true;
						test_n -= 1;

						const BtKinematicRayQResult ray_1_result = test_ray(
								r_position,
								r_position + (up_dir * -(p_step_height * 4.0 + radius)));

						const BtKinematicRayQResult ray_2_result = test_ray(
								target_position,
								target_position + (up_dir * -(p_step_height * 4.0 + radius)));

						if (ray_1_result.hasHit() && ray_2_result.hasHit()) {
							btVector3 new_dir = ray_2_result.m_hitPointWorld -
									ray_1_result.m_hitPointWorld;
							if (likely(new_dir.length2() > CMP_EPSILON)) {
								new_dir.normalize();
								if (new_dir.dot(up_dir) < -HALF_DEG_TOLERANCE) {
									// We are on a slope.
									target_position = r_position + new_dir * remaining_strafing;
									continue;
								}
							}
						}
					}

					r_position = target_position;
					remaining_strafing = 0.0;
					break;
				}
			}
		}
	}

	// Phase 3: Step down to make sure we keep touching the ground.
	{
		if (p_search_floor) {
			// At this point the Actor has moved and it should be on ground.
			// Sometimes the strafing is not precise and it may lost the greound
			// contact.
			// This step down make sure that this doesn't happen.
			const btVector3 vertical_motion = up_dir * -p_step_height * 2.0;
			const BtKinematicConvexQResult result = test_motion(p_motion_shape.shape, r_position, vertical_motion, 0.0, true);
			if (result.hasHit()) {
				r_position += vertical_motion * result.m_closestHitFraction;
			}
		}
	}

	// Unstuck to avoid bumps!
	unstuck(p_motion_shape, r_position, up_dir, 0.0, 0.9);

	const btVector3 motion = r_position - initial_position;
	const real_t len = motion.length();

	return StrafingResult{
		real_t(motion_length >= CMP_EPSILON ? len / motion_length : 1.0),
		motion,
		len >= CMP_EPSILON ? motion / len : btVector3(0.0, 0.0, 0.0),
		len
	};
}

btVector3 KinematicActor::unstuck(
		const ShapeHolder &p_shape,
		btVector3 &r_position,
		const btVector3 &p_up_dir,
		real_t p_margin,
		real_t p_unstuck_factor // TODO do I need this, now?
) const {
#ifdef TOOLS_ENABLED
	// This function is not supposed to be called with null shpae.
	CRASH_COND(p_shape.margin_shape == nullptr);
#endif

	const btVector3 initial = r_position;

	for (int x = 0; x < UNSTUCK_TESTS; x += 1) {
		const BtKinematicContactQResult result = test_contact(p_shape.margin_shape, r_position, p_margin, true);

		btVector3 norm(0.0, 0.0, 0.0);
		real_t depth = 0.0;

		if (result.result_count <= 0) {
			// Shortcut when no collision is found.
			break;
		}

		// Combine the normal.
		for (uint32_t i = 0; i < result.result_count; i += 1) {
			if (unlikely(result.results[i].distance > 0.0)) {
				continue;
			}

			const real_t dot = result.results[i].hit_normal.dot(p_up_dir);
			if (dot >= HALF_DEG_TOLERANCE * 4.0) {
				// Any bottom collision produces a depenetration toward up.
				// This allows to no slide downhil.
				norm += p_up_dir;
			} else {
				// All others, produces depenetration toward the normal.
				norm += result.results[i].hit_normal;
			}
		}

		if (norm.length2() <= 0.0) {
			break;
		}

		// Along this normal takes the bigger penetration.
		norm.normalize();

		for (uint32_t i = 0; i < result.result_count; i += 1) {
			if (unlikely(result.results[i].distance > 0.0)) {
				continue;
			}

			const real_t equality = norm.dot(result.results[i].hit_normal);
			const real_t new_depth = ((-result.results[i].distance) * equality) * UNSTUCK_FACTOR;
			if (new_depth > depth) {
				depth = new_depth;
			}
		}

		// Apply the penetration.
		if (depth == 0.0) {
			// Shortcut when no depenetration is performed.
			break;
		} else {
			r_position += norm * depth * p_unstuck_factor;
		}
	}

	// Computes the unstuck movement normal.
	const btVector3 n = r_position - initial;
	if (n.length2() > 0.0) {
		return n.normalized();
	} else {
		return btVector3(0.0, 0.0, 0.0);
	}
}
