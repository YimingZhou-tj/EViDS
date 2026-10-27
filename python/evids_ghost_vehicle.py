"""Independent ghost spawning helper, separate from cached sensor managers."""
import numpy as np
import carla

def _vehicle_box(actor, transform):
    """World OBB, including actor and bounding-box rotations."""
    box = actor.bounding_box
    matrix = np.asarray(transform.get_matrix(), dtype=float)
    local = np.array([box.location.x, box.location.y, box.location.z, 1.0])
    center = np.dot(matrix, local)[:3]
    rotation = getattr(box, 'rotation', carla.Rotation())
    box_matrix = np.asarray(carla.Transform(carla.Location(), rotation).get_matrix(), dtype=float)
    axes = np.dot(matrix[:3, :3], box_matrix[:3, :3])
    extent = np.array([box.extent.x, box.extent.y, box.extent.z], dtype=float)
    return center, axes, extent


def _vehicle_boxes_overlap(first, second):
    """Separating-axis test; touching boundaries alone are not overlap."""
    center_a, axes_a, extent_a = first
    center_b, axes_b, extent_b = second
    candidates = [axes_a[:, i] for i in range(3)] + [axes_b[:, i] for i in range(3)]
    candidates += [np.cross(axes_a[:, i], axes_b[:, j]) for i in range(3) for j in range(3)]
    delta = center_b - center_a
    for axis in candidates:
        if np.linalg.norm(axis) < 1e-9:
            continue
        radius_a = np.dot(extent_a, np.abs(np.dot(axes_a.T, axis)))
        radius_b = np.dot(extent_b, np.abs(np.dot(axes_b.T, axis)))
        if abs(np.dot(delta, axis)) >= radius_a + radius_b - 1e-6:
            return False
    return True


def spawn_ghost_vehicle(world, blueprint, target, diagnostics_path=''):
    """Spawn a kinematic visualization vehicle without changing the target pose.

    A CarSim reference origin at road height can fail CARLA's initial collision
    check. Try a deterministic elevated staging pose, disable physics, check
    existing vehicles at the target, then teleport to the exact requested pose.
    No pre-existing actor is destroyed, and no world settings are changed here.
    """
    import json
    from pathlib import Path
    actor = None
    record = {'requested_pose': {
        'x': target.location.x, 'y': target.location.y, 'z': target.location.z,
        'pitch': target.rotation.pitch, 'roll': target.rotation.roll, 'yaw': target.rotation.yaw},
        'attempts': [], 'status': 'starting'}

    def save_record():
        if diagnostics_path:
            path = Path(str(diagnostics_path))
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(record, indent=2), encoding='utf-8')

    try:
        actor = world.try_spawn_actor(blueprint, target)
        record['attempts'].append({'z': target.location.z, 'success': actor is not None})
        if actor is not None:
            actor.set_simulate_physics(False)
            record.update(status='success', method='direct', actor_id=actor.id)
            save_record()
            return actor

        # Road height is used only to find staging clearance, never to rewrite
        # CarSim pose, tire positions or the four-wheel road-feedback outputs.
        top = max(float(target.location.z) + 50.0, 50.0)
        bottom = float(target.location.z) - 50.0
        hits = world.cast_ray(carla.Location(target.location.x, target.location.y, top),
                              carla.Location(target.location.x, target.location.y, bottom))
        road = [float(h.location.z) for h in hits if int(h.label) in (6, 7, 8)]
        road_z = max(road) if road else float(target.location.z)
        record['staging_road_z'] = road_z
        base_z = max(float(target.location.z), road_z)
        for clearance in (0.5, 1.0, 2.0, 4.0, 8.0):
            staging = carla.Transform(
                carla.Location(target.location.x, target.location.y, base_z + clearance),
                target.rotation)
            actor = world.try_spawn_actor(blueprint, staging)
            record['attempts'].append({'z': staging.location.z, 'success': actor is not None})
            if actor is not None:
                break
        if actor is None:
            raise RuntimeError('No free staging position above the requested XY. '
                               'See spawn_diagnostics.json for the requested pose and attempted heights.')
        actor.set_simulate_physics(False)
        target_box = _vehicle_box(actor, target)
        for other in world.get_actors().filter('vehicle.*'):
            if other.id != actor.id and other.is_alive:
                if _vehicle_boxes_overlap(target_box, _vehicle_box(other, other.get_transform())):
                    record['occupied_by_actor'] = other.id
                    raise RuntimeError('The target vehicle footprint is occupied by actor %d. '
                                       'No existing vehicles were removed.' % other.id)
        actor.set_transform(target)
        record.update(status='success', method='elevated_then_teleported', actor_id=actor.id)
        save_record()
        print('EVIDS ghost vehicle created via elevated staging; exact CarSim pose preserved: %s' % record['requested_pose'])
        return actor
    except Exception as exc:
        if actor is not None and actor.is_alive:
            actor.destroy()
        record.update(status='failed', error=str(exc))
        save_record()
        raise
