"""Full fixture-on-peg contact wrench from normal AND friction contact data."""

import torch


class ContactWrench:
    """PhysX impulses / physics dt, world axes, moment about a supplied point.

    The standard Lab 2.3 ContactSensor net force excludes friction. Read both
    detailed streams instead. Each stream has its own valid pair ranges; unused
    entries in the allocated buffers must never be summed.
    """

    def __init__(self, sim, body_path, filters, capacity=4096):
        sim.carb_settings.set_bool("/physics/disableContactProcessing", False)
        self.dt = sim.get_physics_dt()
        self.capacity = capacity
        self.view = sim.physics_sim_view.create_rigid_contact_view(
            body_path, filter_patterns=filters, max_contact_data_count=capacity,
        )
        if self.view.sensor_count != 1 or self.view.filter_count != len(filters):
            raise RuntimeError("Unexpected contact view dimensions")

    def _aggregate(self, forces, points, counts, starts, origin):
        force = torch.zeros(3, device=origin.device)
        torque = torch.zeros_like(force)
        total = 0
        for count, start in zip(counts.flatten().tolist(), starts.flatten().tolist()):
            count, start = int(count), int(start)
            if count == 0:
                continue
            if start < 0 or start + count >= self.capacity:
                raise RuntimeError("Contact buffer at capacity; refusing incomplete force data")
            f = forces[start:start+count]
            p = points[start:start+count]
            force += f.sum(0)
            torque += torch.cross(p - origin, f, dim=-1).sum(0)
            total += count
        if not torch.isfinite(force).all() or not torch.isfinite(torque).all():
            raise RuntimeError("Non-finite contact wrench")
        return force, torque, total

    def read(self, origin):
        magnitudes, points, normals, separations, counts, starts = self.view.get_contact_data(self.dt)
        normal, normal_torque, n = self._aggregate(magnitudes * normals, points, counts, starts, origin)
        min_separation = 0.0
        normal_load = 0.0
        for count, start in zip(counts.flatten().tolist(), starts.flatten().tolist()):
            if count:
                normal_load += magnitudes[int(start):int(start+count)].sum().item()
                min_separation = min(min_separation, separations[int(start):int(start+count)].min().item())
        # get_friction_data reuses counts/starts buffers: finish normal processing first.
        forces, points, counts, starts = self.view.get_friction_data(self.dt)
        friction, friction_torque, nf = self._aggregate(forces, points, counts, starts, origin)
        return {
            "force": normal + friction, "torque": normal_torque + friction_torque,
            "normal_force": normal, "friction_force": friction,
            "contact_count": n, "friction_count": nf,
            "min_separation_m": min_separation, "normal_load_n": normal_load,
        }
