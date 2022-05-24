/* Author: AndreaCatania */

#ifndef KINEMATIC_PLUGINS_H
#define KINEMATIC_PLUGINS_H

#ifdef TOOLS_ENABLED
#define PLUGIN_DISABLED
#endif

#ifndef PLUGIN_DISABLE

#include "editor/spatial_editor_gizmos.h"

class KinematicObject3DGizmoPlugin : public EditorSpatialGizmoPlugin {
	GDCLASS(KinematicObject3DGizmoPlugin, EditorSpatialGizmoPlugin);

public:
	KinematicObject3DGizmoPlugin();

	virtual bool has_gizmo(Spatial *p_node) override;
	virtual String get_name() const override;
	virtual int get_priority() const override;

	virtual void redraw(EditorSpatialGizmo *p_gizmo) override;
	virtual String get_handle_name(const EditorSpatialGizmo *p_gizmo, int p_idx) const override;
	virtual Variant get_handle_value(EditorSpatialGizmo *p_gizmo, int p_idx) const override;
	virtual void set_handle(EditorSpatialGizmo *p_gizmo, int p_idx, Camera *p_camera, const Point2 &p_point) override;
	virtual void commit_handle(EditorSpatialGizmo *p_gizmo, int p_idx, const Variant &p_restore, bool p_cancel = false) override;

	int draw_shape(EditorSpatialGizmo *p_gizmo, Ref<Shape> p_shape, const Vector3 &p_offset, const String &p_shape_mat_name, const String &p_handle_mat_name);
	String get_shape_handle_name(Ref<Shape> p_shape, int p_idx) const;
	Variant get_shape_handle_value(Ref<Shape> p_shape, int p_idx) const;
	void set_shape_handle(Ref<Shape> p_shape, const Transform &p_gt, int p_idx, Camera *p_camera, const Point2 &p_point);
	void commit_shape_handle(Ref<Shape> p_shape, int p_idx, const Variant &p_restore, bool p_cancel);
};

#endif
#endif
