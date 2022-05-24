#ifndef PLUGIN_DISABLE

#include "plugins.h"

#include "kinematic_object_3d.h"
#include "scene/resources/box_shape.h"
#include "scene/resources/capsule_shape.h"
#include "scene/resources/cylinder_shape.h"
#include "scene/resources/ray_shape.h"
#include "scene/resources/sphere_shape.h"

KinematicObject3DGizmoPlugin::KinematicObject3DGizmoPlugin() {
	const Color gizmo_color = EDITOR_DEF("editors/3d_gizmos/gizmo_colors/shape", Color(0.5, 0.7, 1));
	create_material("shape_material", gizmo_color);
	const float gizmo_value = gizmo_color.get_v();
	const Color gizmo_color_disabled = Color(gizmo_value, gizmo_value, gizmo_value, 0.65);
	create_material("shape_material_disabled", gizmo_color_disabled);
	create_handle_material("handles");
}

bool KinematicObject3DGizmoPlugin::has_gizmo(Spatial *p_node) {
	return Object::cast_to<KinematicObject3D>(p_node) != nullptr;
}

String KinematicObject3DGizmoPlugin::get_name() const {
	return "KinematicObject3D";
}

int KinematicObject3DGizmoPlugin::get_priority() const {
	return -1;
}

void KinematicObject3DGizmoPlugin::redraw(EditorSpatialGizmo *p_gizmo) {
	p_gizmo->clear();

	const KinematicObject3D *k = Object::cast_to<KinematicObject3D>(p_gizmo->get_spatial_node());

	Ref<Shape> s = k->get_shape();
	if (s.is_null()) {
		return;
	}

	const bool disabled = false; // TODO Why this? probably was left on master for compatibility. Remove it if not needed.
	draw_shape(p_gizmo, s, Vector3(), !disabled ? "shape_material" : "shape_material_disabled", "handles");
}

String KinematicObject3DGizmoPlugin::get_handle_name(const EditorSpatialGizmo *p_gizmo, int p_idx) const {
	const KinematicObject3D *k = Object::cast_to<KinematicObject3D>(p_gizmo->get_spatial_node());

	Ref<Shape> s = k->get_shape();
	return get_shape_handle_name(s, p_idx);
}

Variant KinematicObject3DGizmoPlugin::get_handle_value(EditorSpatialGizmo *p_gizmo, int p_idx) const {
	const KinematicObject3D *k = Object::cast_to<KinematicObject3D>(p_gizmo->get_spatial_node());

	Ref<Shape> s = k->get_shape();
	return get_shape_handle_value(s, p_idx);
}

void KinematicObject3DGizmoPlugin::set_handle(EditorSpatialGizmo *p_gizmo, int p_idx, Camera *p_camera, const Point2 &p_point) {
	const KinematicObject3D *k = Object::cast_to<KinematicObject3D>(p_gizmo->get_spatial_node());

	Ref<Shape> s = k->get_shape();
	const Transform gt = k->get_global_transform();

	set_shape_handle(s, gt, p_idx, p_camera, p_point);
}

void KinematicObject3DGizmoPlugin::commit_handle(EditorSpatialGizmo *p_gizmo, int p_idx, const Variant &p_restore, bool p_cancel) {
	const KinematicObject3D *k = Object::cast_to<KinematicObject3D>(p_gizmo->get_spatial_node());

	Ref<Shape> s = k->get_shape();
	commit_shape_handle(s, p_idx, p_restore, p_cancel);
}

int KinematicObject3DGizmoPlugin::draw_shape(EditorSpatialGizmo *p_gizmo, Ref<Shape> s, const Vector3 &p_offset, const String &p_shape_mat_name, const String &p_handle_mat_name) {
	const int h = p_gizmo->handles.size();
	const Ref<Material> material = get_material(p_shape_mat_name, p_gizmo);
	Ref<Material> handles_material = get_material(p_handle_mat_name);

	if (Object::cast_to<SphereShape>(*s)) {
		Ref<SphereShape> sp = s;
		float r = sp->get_radius();

		Vector<Vector3> points;

		for (int i = 0; i <= 360; i++) {
			float ra = Math::deg2rad((float)i);
			float rb = Math::deg2rad((float)i + 1);
			Point2 a = Vector2(Math::sin(ra), Math::cos(ra)) * r;
			Point2 b = Vector2(Math::sin(rb), Math::cos(rb)) * r;

			points.push_back(p_offset + Vector3(a.x, 0, a.y));
			points.push_back(p_offset + Vector3(b.x, 0, b.y));
			points.push_back(p_offset + Vector3(0, a.x, a.y));
			points.push_back(p_offset + Vector3(0, b.x, b.y));
			points.push_back(p_offset + Vector3(a.x, a.y, 0));
			points.push_back(p_offset + Vector3(b.x, b.y, 0));
		}

		Vector<Vector3> collision_segments;

		for (int i = 0; i < 64; i++) {
			float ra = i * Math_PI * 2.0 / 64.0;
			float rb = (i + 1) * Math_PI * 2.0 / 64.0;
			Point2 a = Vector2(Math::sin(ra), Math::cos(ra)) * r;
			Point2 b = Vector2(Math::sin(rb), Math::cos(rb)) * r;

			collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y));
			collision_segments.push_back(p_offset + Vector3(b.x, 0, b.y));
			collision_segments.push_back(p_offset + Vector3(0, a.x, a.y));
			collision_segments.push_back(p_offset + Vector3(0, b.x, b.y));
			collision_segments.push_back(p_offset + Vector3(a.x, a.y, 0));
			collision_segments.push_back(p_offset + Vector3(b.x, b.y, 0));
		}

		p_gizmo->add_lines(points, material);
		p_gizmo->add_collision_segments(collision_segments);
		Vector<Vector3> handles;
		handles.push_back(p_offset + Vector3(r, 0, 0));
		p_gizmo->add_handles(handles, handles_material);
	}

	if (Object::cast_to<BoxShape>(*s)) {
		Ref<BoxShape> bs = s;
		Vector<Vector3> lines;
		AABB aabb;
		aabb.position = -bs->get_extents();
		aabb.size = aabb.position * -2;

		for (int i = 0; i < 12; i++) {
			Vector3 a, b;
			aabb.get_edge(i, a, b);
			lines.push_back(p_offset + a);
			lines.push_back(p_offset + b);
		}

		Vector<Vector3> handles;

		for (int i = 0; i < 3; i++) {
			Vector3 ax;
			ax[i] = bs->get_extents()[i];
			handles.push_back(p_offset + ax);
		}

		p_gizmo->add_lines(lines, material);
		p_gizmo->add_collision_segments(lines);
		p_gizmo->add_handles(handles, handles_material);
	}

	if (Object::cast_to<CapsuleShape>(*s)) {
		Ref<CapsuleShape> cs2 = s;
		float radius = cs2->get_radius();
		float height = cs2->get_height();

		Vector<Vector3> points;

		Vector3 d(0, height * 0.5, 0);
		for (int i = 0; i < 360; i++) {
			float ra = Math::deg2rad((float)i);
			float rb = Math::deg2rad((float)i + 1);
			Point2 a = Vector2(Math::sin(ra), Math::cos(ra)) * radius;
			Point2 b = Vector2(Math::sin(rb), Math::cos(rb)) * radius;

			points.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
			points.push_back(p_offset + Vector3(b.x, 0, b.y) + d);

			points.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			points.push_back(p_offset + Vector3(b.x, 0, b.y) - d);

			if (i % 90 == 0) {
				points.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
				points.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			}

			Vector3 dud = i < 180 ? d : -d;

			points.push_back(p_offset + Vector3(0, a.x, a.y) + dud);
			points.push_back(p_offset + Vector3(0, b.x, b.y) + dud);
			points.push_back(p_offset + Vector3(a.y, a.x, 0) + dud);
			points.push_back(p_offset + Vector3(b.y, b.x, 0) + dud);
		}

		p_gizmo->add_lines(points, material);

		Vector<Vector3> collision_segments;

		for (int i = 0; i < 64; i++) {
			float ra = i * Math_PI * 2.0 / 64.0;
			float rb = (i + 1) * Math_PI * 2.0 / 64.0;
			Point2 a = Vector2(Math::sin(ra), Math::cos(ra)) * radius;
			Point2 b = Vector2(Math::sin(rb), Math::cos(rb)) * radius;

			collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
			collision_segments.push_back(p_offset + Vector3(b.x, 0, b.y) + d);

			collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			collision_segments.push_back(p_offset + Vector3(b.x, 0, b.y) - d);

			if (i % 16 == 0) {
				collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
				collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			}

			Vector3 dud = i < 32 ? d : -d;

			collision_segments.push_back(p_offset + Vector3(0, a.x, a.y) + dud);
			collision_segments.push_back(p_offset + Vector3(0, b.x, b.y) + dud);
			collision_segments.push_back(p_offset + Vector3(a.y, a.x, 0) + dud);
			collision_segments.push_back(p_offset + Vector3(b.y, b.x, 0) + dud);
		}

		p_gizmo->add_collision_segments(collision_segments);

		Vector<Vector3> handles;
		handles.push_back(p_offset + Vector3(cs2->get_radius(), 0, 0));
		handles.push_back(p_offset + Vector3(0, cs2->get_height() * 0.5 + cs2->get_radius(), 0));
		p_gizmo->add_handles(handles, handles_material);
	}

	if (Object::cast_to<CylinderShape>(*s)) {
		Ref<CylinderShape> cs2 = s;
		float radius = cs2->get_radius();
		float height = cs2->get_height();

		Vector<Vector3> points;

		Vector3 d(0, height * 0.5, 0);
		for (int i = 0; i < 360; i++) {
			float ra = Math::deg2rad((float)i);
			float rb = Math::deg2rad((float)i + 1);
			Point2 a = Vector2(Math::sin(ra), Math::cos(ra)) * radius;
			Point2 b = Vector2(Math::sin(rb), Math::cos(rb)) * radius;

			points.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
			points.push_back(p_offset + Vector3(b.x, 0, b.y) + d);

			points.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			points.push_back(p_offset + Vector3(b.x, 0, b.y) - d);

			if (i % 90 == 0) {
				points.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
				points.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			}
		}

		p_gizmo->add_lines(points, material);

		Vector<Vector3> collision_segments;

		for (int i = 0; i < 64; i++) {
			float ra = i * Math_PI * 2.0 / 64.0;
			float rb = (i + 1) * Math_PI * 2.0 / 64.0;
			Point2 a = Vector2(Math::sin(ra), Math::cos(ra)) * radius;
			Point2 b = Vector2(Math::sin(rb), Math::cos(rb)) * radius;

			collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
			collision_segments.push_back(p_offset + Vector3(b.x, 0, b.y) + d);

			collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			collision_segments.push_back(p_offset + Vector3(b.x, 0, b.y) - d);

			if (i % 16 == 0) {
				collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) + d);
				collision_segments.push_back(p_offset + Vector3(a.x, 0, a.y) - d);
			}
		}

		p_gizmo->add_collision_segments(collision_segments);

		Vector<Vector3> handles;
		handles.push_back(p_offset + Vector3(cs2->get_radius(), 0, 0));
		handles.push_back(p_offset + Vector3(0, cs2->get_height() * 0.5, 0));
		p_gizmo->add_handles(handles, handles_material);
	}

	if (Object::cast_to<RayShape>(*s)) {
		Ref<RayShape> rs = s;

		Vector<Vector3> points;
		points.push_back(p_offset + Vector3());
		points.push_back(p_offset + Vector3(0, 0, rs->get_length()));
		p_gizmo->add_lines(points, material);
		p_gizmo->add_collision_segments(points);
		Vector<Vector3> handles;
		handles.push_back(p_offset + Vector3(0, 0, rs->get_length()));
		p_gizmo->add_handles(handles, handles_material);
	}

	if (h < p_gizmo->handles.size()) {
		return h;
	} else {
		return -1;
	}
}

String KinematicObject3DGizmoPlugin::get_shape_handle_name(Ref<Shape> p_shape, int p_idx) const {
	if (p_shape.is_null()) {
		return "";
	}

	if (Object::cast_to<SphereShape>(*p_shape)) {
		return "Radius";
	}

	if (Object::cast_to<BoxShape>(*p_shape)) {
		return "Extents";
	}

	if (Object::cast_to<CapsuleShape>(*p_shape)) {
		return p_idx == 0 ? "Radius" : "Height";
	}

	if (Object::cast_to<CylinderShape>(*p_shape)) {
		return p_idx == 0 ? "Radius" : "Height";
	}

	if (Object::cast_to<RayShape>(*p_shape)) {
		return "Length";
	}

	return "";
}

Variant KinematicObject3DGizmoPlugin::get_shape_handle_value(Ref<Shape> p_shape, int p_idx) const {
	if (p_shape.is_null()) {
		return Variant();
	}

	if (Object::cast_to<SphereShape>(*p_shape)) {
		Ref<SphereShape> ss = p_shape;
		return ss->get_radius();
	}

	if (Object::cast_to<BoxShape>(*p_shape)) {
		Ref<BoxShape> bs = p_shape;
		return bs->get_extents();
	}

	if (Object::cast_to<CapsuleShape>(*p_shape)) {
		Ref<CapsuleShape> cs2 = p_shape;
		return p_idx == 0 ? cs2->get_radius() : cs2->get_height();
	}

	if (Object::cast_to<CylinderShape>(*p_shape)) {
		Ref<CylinderShape> cs2 = p_shape;
		return p_idx == 0 ? cs2->get_radius() : cs2->get_height();
	}

	if (Object::cast_to<RayShape>(*p_shape)) {
		Ref<RayShape> cs2 = p_shape;
		return cs2->get_length();
	}

	return Variant();
}

void KinematicObject3DGizmoPlugin::set_shape_handle(Ref<Shape> p_shape, const Transform &gt, int p_idx, Camera *p_camera, const Point2 &p_point) {
	if (p_shape.is_null()) {
		return;
	}

	Transform gi = gt.affine_inverse();

	Vector3 ray_from = p_camera->project_ray_origin(p_point);
	Vector3 ray_dir = p_camera->project_ray_normal(p_point);

	Vector3 sg[2] = { gi.xform(ray_from), gi.xform(ray_from + ray_dir * 4096) };

	if (Object::cast_to<SphereShape>(*p_shape)) {
		Ref<SphereShape> ss = p_shape;
		Vector3 ra, rb;
		Geometry::get_closest_points_between_segments(Vector3(), Vector3(4096, 0, 0), sg[0], sg[1], ra, rb);
		float d = ra.x;
		if (SpatialEditor::get_singleton()->is_snap_enabled()) {
			d = Math::stepify(d, SpatialEditor::get_singleton()->get_translate_snap());
		}

		if (d < 0.001) {
			d = 0.001;
		}

		ss->set_radius(d);
	}

	if (Object::cast_to<RayShape>(*p_shape)) {
		Ref<RayShape> rs = p_shape;
		Vector3 ra, rb;
		Geometry::get_closest_points_between_segments(Vector3(), Vector3(0, 0, 4096), sg[0], sg[1], ra, rb);
		float d = ra.z;
		if (SpatialEditor::get_singleton()->is_snap_enabled()) {
			d = Math::stepify(d, SpatialEditor::get_singleton()->get_translate_snap());
		}

		if (d < 0.001) {
			d = 0.001;
		}

		rs->set_length(d);
	}

	if (Object::cast_to<BoxShape>(*p_shape)) {
		Vector3 axis;
		axis[p_idx] = 1.0;
		Ref<BoxShape> bs = p_shape;
		Vector3 ra, rb;
		Geometry::get_closest_points_between_segments(Vector3(), axis * 4096, sg[0], sg[1], ra, rb);
		float d = ra[p_idx];
		if (SpatialEditor::get_singleton()->is_snap_enabled()) {
			d = Math::stepify(d, SpatialEditor::get_singleton()->get_translate_snap());
		}

		if (d < 0.001) {
			d = 0.001;
		}

		Vector3 he = bs->get_extents();
		he[p_idx] = d;
		bs->set_extents(he);
	}

	if (Object::cast_to<CapsuleShape>(*p_shape)) {
		Vector3 axis;
		axis[p_idx == 0 ? 0 : 2] = 1.0;
		Ref<CapsuleShape> cs2 = p_shape;
		Vector3 ra, rb;
		Geometry::get_closest_points_between_segments(Vector3(), axis * 4096, sg[0], sg[1], ra, rb);
		float d = axis.dot(ra);
		if (p_idx == 1) {
			d -= cs2->get_radius();
		}

		if (SpatialEditor::get_singleton()->is_snap_enabled()) {
			d = Math::stepify(d, SpatialEditor::get_singleton()->get_translate_snap());
		}

		if (d < 0.001) {
			d = 0.001;
		}

		if (p_idx == 0) {
			cs2->set_radius(d);
		} else if (p_idx == 1) {
			cs2->set_height(d * 2.0);
		}
	}

	if (Object::cast_to<CylinderShape>(*p_shape)) {
		Vector3 axis;
		axis[p_idx == 0 ? 0 : 1] = 1.0;
		Ref<CylinderShape> cs2 = p_shape;
		Vector3 ra, rb;
		Geometry::get_closest_points_between_segments(Vector3(), axis * 4096, sg[0], sg[1], ra, rb);
		float d = axis.dot(ra);
		if (SpatialEditor::get_singleton()->is_snap_enabled()) {
			d = Math::stepify(d, SpatialEditor::get_singleton()->get_translate_snap());
		}

		if (d < 0.001) {
			d = 0.001;
		}

		if (p_idx == 0) {
			cs2->set_radius(d);
		} else if (p_idx == 1) {
			cs2->set_height(d * 2.0);
		}
	}
}

void KinematicObject3DGizmoPlugin::commit_shape_handle(Ref<Shape> p_shape, int p_idx, const Variant &p_restore, bool p_cancel) {
	if (p_shape.is_null()) {
		return;
	}

	if (Object::cast_to<SphereShape>(*p_shape)) {
		Ref<SphereShape> ss = p_shape;
		if (p_cancel) {
			ss->set_radius(p_restore);
			return;
		}

		UndoRedo *ur = SpatialEditor::get_singleton()->get_undo_redo();
		ur->create_action(TTR("Change Sphere Shape Radius"));
		ur->add_do_method(ss.ptr(), "set_radius", ss->get_radius());
		ur->add_undo_method(ss.ptr(), "set_radius", p_restore);
		ur->commit_action();
	}

	if (Object::cast_to<BoxShape>(*p_shape)) {
		Ref<BoxShape> ss = p_shape;
		if (p_cancel) {
			ss->set_extents(p_restore);
			return;
		}

		UndoRedo *ur = SpatialEditor::get_singleton()->get_undo_redo();
		ur->create_action(TTR("Change Box Shape Extents"));
		ur->add_do_method(ss.ptr(), "set_extents", ss->get_extents());
		ur->add_undo_method(ss.ptr(), "set_extents", p_restore);
		ur->commit_action();
	}

	if (Object::cast_to<CapsuleShape>(*p_shape)) {
		Ref<CapsuleShape> ss = p_shape;
		if (p_cancel) {
			if (p_idx == 0) {
				ss->set_radius(p_restore);
			} else {
				ss->set_height(p_restore);
			}
			return;
		}

		UndoRedo *ur = SpatialEditor::get_singleton()->get_undo_redo();
		if (p_idx == 0) {
			ur->create_action(TTR("Change Capsule Shape Radius"));
			ur->add_do_method(ss.ptr(), "set_radius", ss->get_radius());
			ur->add_undo_method(ss.ptr(), "set_radius", p_restore);
		} else {
			ur->create_action(TTR("Change Capsule Shape Height"));
			ur->add_do_method(ss.ptr(), "set_height", ss->get_height());
			ur->add_undo_method(ss.ptr(), "set_height", p_restore);
		}

		ur->commit_action();
	}

	if (Object::cast_to<CylinderShape>(*p_shape)) {
		Ref<CylinderShape> ss = p_shape;
		if (p_cancel) {
			if (p_idx == 0) {
				ss->set_radius(p_restore);
			} else {
				ss->set_height(p_restore);
			}
			return;
		}

		UndoRedo *ur = SpatialEditor::get_singleton()->get_undo_redo();
		if (p_idx == 0) {
			ur->create_action(TTR("Change Cylinder Shape Radius"));
			ur->add_do_method(ss.ptr(), "set_radius", ss->get_radius());
			ur->add_undo_method(ss.ptr(), "set_radius", p_restore);
		} else {
			ur->create_action(
					///

					////////
					TTR("Change Cylinder Shape Height"));
			ur->add_do_method(ss.ptr(), "set_height", ss->get_height());
			ur->add_undo_method(ss.ptr(), "set_height", p_restore);
		}

		ur->commit_action();
	}

	if (Object::cast_to<RayShape>(*p_shape)) {
		Ref<RayShape> ss = p_shape;
		if (p_cancel) {
			ss->set_length(p_restore);
			return;
		}

		UndoRedo *ur = SpatialEditor::get_singleton()->get_undo_redo();
		ur->create_action(TTR("Change Ray Shape Length"));
		ur->add_do_method(ss.ptr(), "set_length", ss->get_length());
		ur->add_undo_method(ss.ptr(), "set_length", p_restore);
		ur->commit_action();
	}
}

#endif
