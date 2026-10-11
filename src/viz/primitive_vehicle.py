"""Procedural traffic assets with bbox-adaptive bodies and circular wheels.

All templates use +X forward, +Y left and +Z up, centered on their metric bbox.
Topology and vertex colors never change when dimensions change.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import trimesh


@dataclass(frozen=True)
class _Wheel:
    start: int
    stop: int
    longitudinal_fraction: float
    side: int
    radius: float
    depth: float
    center: np.ndarray


class PrimitiveVehicleAsset:
    """A reusable template with independently repositioned wheel assemblies.

    Wheel radius and width remain fixed in meters for ordinary bbox changes.
    An impossibly small bbox uniformly shrinks each complete wheel assembly by
    the minimum factor needed to fit all axles along its length, its height,
    and allocated width. Adjacent tires retain a small separation gap.
    Body surfaces follow all three bbox dimensions. These are class silhouettes,
    not reconstructions of particular vehicle makes or models.
    """

    SPECS = {
        "car": ((4.65, 1.85, 1.48), .33, .21),
        "truck": ((8.0, 2.5, 3.5), .49, .30),
        "bus": ((12.0, 2.55, 3.25), .48, .30),
        "articulated_bus": ((18.0, 2.55, 3.25), .48, .30),
        "trailer": ((12.0, 2.55, 3.8), .49, .30),
        "construction_vehicle": ((6.5, 2.7, 3.2), .67, .40),
        "motorcycle": ((2.15, .80, 1.25), .31, .13),
        "bicycle": ((1.8, .60, 1.10), .33, .045),
        "ambulance": ((6.0, 2.15, 2.8), .37, .24),
        "police": ((4.8, 1.9, 1.58), .34, .22),
    }
    SUPPORTED_KINDS = tuple(SPECS)
    BODY = (232, 234, 236, 255)
    GLASS = (165, 169, 173, 255)
    TIRE = (65, 68, 72, 255)
    RIM = (88, 93, 99, 255)
    TRIM = (88, 97, 104, 255)
    WHITE = (244, 245, 246, 255)
    RED = (143, 66, 64, 255)
    BLUE = (67, 91, 131, 255)

    def __init__(self, kind: str = "car") -> None:
        import trimesh

        if kind not in self.SPECS:
            raise ValueError(f"Unsupported primitive vehicle kind: {kind!r}")
        self.kind = kind
        self.nominal_dimensions = np.array(self.SPECS[kind][0], dtype=float)
        self._trimesh = trimesh
        self._parts: list[trimesh.Trimesh] = []
        self._vertex_count = 0
        self._wheels: list[_Wheel] = []
        self._axle_links = []
        if kind in ("car", "police"):
            self._car()
        elif kind in ("truck", "ambulance"):
            self._truck()
        elif kind in ("bus", "articulated_bus"):
            self._bus()
        elif kind == "trailer":
            self._trailer()
        elif kind == "construction_vehicle":
            self._loader()
        else:
            self._cycle()
        self._mesh = trimesh.util.concatenate(self._parts)
        self._parts.clear()
        # Keep thin overlays just outside their host panels while bringing the
        # full body inside the requested bbox; wheels remain metric geometry.
        body_mask = np.ones(len(self._mesh.vertices), dtype=bool)
        for wheel in self._wheels:
            body_mask[wheel.start:wheel.stop] = False
        body = self._mesh.vertices[body_mask].copy()
        lower, upper = body.min(axis=0), body.max(axis=0)
        for axis in range(3):
            if max(abs(lower[axis]), abs(upper[axis])) > self.nominal_dimensions[axis] / 2:
                body[:, axis] = (body[:, axis] - (lower[axis] + upper[axis]) / 2) * (
                    self.nominal_dimensions[axis] / (upper[axis] - lower[axis])
                )
        self._mesh.vertices[body_mask] = body
        self._body_vertices = np.asarray(self._mesh.vertices).copy()
        self._body_vertices.setflags(write=False)

    @property
    def mesh(self) -> trimesh.Trimesh:
        """Nominal metric template. Copy before modifying geometry or colors."""
        return self._mesh

    def vertices_for_dimensions(self, dimensions_lwh) -> np.ndarray:
        """Return updated metric vertices without rebuilding faces or colors."""
        dimensions = np.asarray(dimensions_lwh, dtype=float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
            raise ValueError("Vehicle dimensions must be three finite positive LWH values")
        vertices = self._body_vertices * (dimensions / self.nominal_dimensions)
        if self.kind in ("car", "police"):
            # A low bbox needs a shallow cabin, rather than a hood below the
            # fixed-size tires. Raise the belt line only when wheel clearance
            # requires it; nominal and taller cars retain the authored profile.
            _, wheel_factor = self._wheel_placement(dimensions, self._wheels[0])
            height = dimensions[2]
            belt = min(.90 * height, max(.55 * height,
                       2 * self._wheels[0].radius * wheel_factor + .02))
            if belt > .55 * height:
                z_fraction = self._body_vertices[:, 2] / self.nominal_dimensions[2] + .5
                raised_z = np.where(z_fraction <= .55, z_fraction / .55 * belt,
                                    belt + (z_fraction - .55) / .45 * (height - belt))
                vertices[:, 2] = raised_z - height / 2
        centers = {}
        for wheel in self._wheels:
            center, factor = self._wheel_placement(dimensions, wheel)
            vertices[wheel.start:wheel.stop] = (
                self._body_vertices[wheel.start:wheel.stop] - wheel.center
            ) * factor + center
            centers[wheel.longitudinal_fraction] = center
        # Cycle forks and rear stays track their axles as wheel radius stays
        # fixed. Blend only their endpoint displacement; keep the upper frame
        # attached to the dimension-scaled saddle and steering head.
        for start, stop, axle_fraction, nominal_axle, weights in self._axle_links:
            scaled_axle = nominal_axle * dimensions / self.nominal_dimensions
            displacement = centers[axle_fraction] - scaled_axle
            vertices[start:stop] += weights[:, None] * displacement
            # Impossibly flat cycle boxes can put a thick fork above the roof
            # after hub attachment. Clip only the frame, never the round wheel.
            vertices[start:stop] = np.clip(vertices[start:stop], -dimensions / 2, dimensions / 2)
        return vertices

    def _wheel_placement(self, dimensions, wheel):
        length, width, height = dimensions
        # Side wheels each occupy at most half the width; centerline cycle
        # wheels may use the full width. One scalar preserves circles.
        allowed_width = width / (2 if wheel.side else 1)
        fractions = sorted(w.longitudinal_fraction for w in self._wheels if w.side == wheel.side)
        # Account for every axle before applying the safety cap: fitting one
        # diameter alone could leave neighboring tires overlapping in a short
        # truck or trailer. Ordinary compact boxes reposition full-size wheels.
        length_needed = wheel.radius * (2 + 2.05 * (len(fractions) - 1))
        factor = min(1., length / length_needed, height / (2 * wheel.radius),
                     allowed_width / wheel.depth)
        radius, depth = wheel.radius * factor, wheel.depth * factor
        if self.kind in ("bicycle", "motorcycle"):
            x = np.sign(wheel.longitudinal_fraction) * (length / 2 - radius)
        else:
            lower, upper = -length / 2 + radius, length / 2 - radius
            positions = np.clip(np.array(fractions) * length, lower, upper)
            spacing = 2.05 * radius
            for index in range(1, len(positions)):
                positions[index] = max(positions[index], positions[index - 1] + spacing)
            positions[-1] = min(positions[-1], upper)
            for index in range(len(positions) - 2, -1, -1):
                positions[index] = min(positions[index], positions[index + 1] - spacing)
            x = positions[fractions.index(wheel.longitudinal_fraction)]
        return np.array([x, wheel.side * (width / 2 - depth / 2), -height / 2 + radius]), factor

    def mesh_for_dimensions(self, dimensions_lwh) -> trimesh.Trimesh:
        mesh = self._mesh.copy()
        mesh.vertices = self.vertices_for_dimensions(dimensions_lwh)
        return mesh

    def export(self, path: Path, dimensions_lwh=None) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        mesh = self.mesh if dimensions_lwh is None else self.mesh_for_dimensions(dimensions_lwh)
        mesh.export(str(path), file_type="glb")
        return path

    def _append(self, mesh, color, *, smooth=False, face_colors=None):
        mesh.fix_normals()
        light = np.array([.25, -.35, .90])
        light /= np.linalg.norm(light)
        if smooth and face_colors is None:
            normals = mesh.vertex_normals
        else:
            normals = (mesh.vertex_normals[mesh.faces].reshape(-1, 3) if smooth
                       else np.repeat(mesh.face_normals, 3, axis=0))
            vertices = mesh.triangles.reshape(-1, 3).copy()
            mesh = self._trimesh.Trimesh(vertices=vertices,
                                        faces=np.arange(len(vertices)).reshape(-1, 3), process=False)
        brightness = .78 + .22 * np.maximum(normals @ light, 0.)
        colors = (np.tile(color, (len(mesh.vertices), 1)) if face_colors is None
                  else np.repeat(np.asarray(face_colors), 3, axis=0))
        colors[:, :3] = np.rint(colors[:, :3] * brightness[:, None])
        mesh.visual.vertex_colors = colors
        self._parts.append(mesh)
        self._vertex_count += len(mesh.vertices)

    def _coords(self, points):
        points = np.asarray(points, dtype=float).copy()
        points[..., 2] -= .5
        return points * self.nominal_dimensions

    def _poly(self, points, faces, color, *, smooth=False):
        self._append(self._trimesh.Trimesh(vertices=self._coords(points), faces=faces, process=False),
                     color, smooth=smooth)

    def _quad(self, points, color):
        self._poly(points, [[0, 1, 2], [0, 2, 3]], color)

    def _box(self, lower, upper, color):
        lower, upper = self._coords([lower, upper])
        mesh = self._trimesh.creation.box(extents=upper - lower)
        mesh.apply_translation((lower + upper) / 2)
        self._append(mesh, color)

    def _loft(self, sections, color, *, smooth=False):
        """Sections: X, half width, bottom, top; rounded rectangular shoulders."""
        points = []
        # Chamfered 12-point section keeps horizontal hood and clean shoulder.
        for x, width, bottom, top in sections:
            dz = (top - bottom) * .16
            points.extend([(x, -width * .78, bottom), (x, width * .78, bottom),
                           (x, width * .95, bottom + dz * .4), (x, width, bottom + dz),
                           (x, width, top - dz), (x, width * .95, top - dz * .4),
                           (x, width * .78, top), (x, -width * .78, top),
                           (x, -width * .95, top - dz * .4), (x, -width, top - dz),
                           (x, -width, bottom + dz), (x, -width * .95, bottom + dz * .4)])
        faces = []
        for section in range(len(sections) - 1):
            for corner in range(12):
                a, b = section * 12 + corner, section * 12 + (corner + 1) % 12
                faces.extend([[a, b, b + 12], [a, b + 12, a + 12]])
        last = (len(sections) - 1) * 12
        for corner in range(1, 11):
            faces.extend([[0, corner + 1, corner], [last, last + corner, last + corner + 1]])
        self._poly(points, faces, color, smooth=smooth)

    def _wheel(self, x_fraction, side, *, open_center=False):
        _, radius, depth = self.SPECS[self.kind]
        length, width, height = self.nominal_dimensions
        center = np.array([x_fraction * length, side * (width / 2 - depth / 2), -height / 2 + radius])
        start = self._vertex_count
        # Rounded tire shoulders and a single muted inset rim avoid concentric
        # contrasting hubs. Cycles use hollow rings, with thin spokes below.
        segments = 24
        profile = [(depth / 2, radius * .72), (depth / 2, radius * .90),
                   (depth * .32, radius), (-depth * .32, radius),
                   (-depth / 2, radius * .90), (-depth / 2, radius * .72)]
        if open_center:
            profile = [(depth / 2, radius * .90), (depth / 2, radius * .96),
                       (depth * .30, radius), (-depth * .30, radius),
                       (-depth / 2, radius * .96), (-depth / 2, radius * .90)]
        points = []
        for y, r in profile:
            for angle in np.arange(segments) * 2 * np.pi / segments:
                points.append([r * np.cos(angle), y, r * np.sin(angle)])
        faces = []
        for ring in range(len(profile)):
            nxt = (ring + 1) % len(profile)
            for index in range(segments):
                a, b = ring * segments + index, ring * segments + (index + 1) % segments
                c, d = nxt * segments + index, nxt * segments + (index + 1) % segments
                faces.extend([[a, b, d], [a, d, c]])
        tire = self._trimesh.Trimesh(vertices=np.asarray(points) + center, faces=faces, process=False)
        self._append(tire, self.TIRE, smooth=True)
        if not open_center:
            rim = self._trimesh.creation.cylinder(radius=radius * .73, height=depth * .92, sections=24)
            rim.apply_transform(self._trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
            rim.apply_translation(center)
            self._append(rim, self.RIM, smooth=True)
        else:
            for angle in np.arange(8) * np.pi / 4:
                delta = np.array([np.cos(angle), 0., np.sin(angle)]) * radius * .91
                self._rod_metric(center, center + delta, .005, self.TRIM)
        self._wheels.append(_Wheel(start, self._vertex_count, x_fraction, side, radius, depth, center))

    def _axles(self, positions):
        for x in positions:
            for side in (-1, 1):
                self._wheel(x, side)

    def _lights(self, front=.499, rear=-.499, z=.44, *, car=False):
        for side in (-1, 1):
            ys = sorted([side * (.12 if car else .26), side * (.32 if car else .43)])
            self._box((front - .002, ys[0], z), (front + .001, ys[1], z + .027), self.WHITE)
            rear_ys = sorted([side * .08, side * .32]) if car else ys
            self._box((rear - .001, rear_ys[0], z),
                      (rear + .003, rear_ys[1], z + (.043 if car else .027)),
                      (167, 63, 60, 255) if car else self.RED)

    def _car(self):
        police = self.kind == "police"
        roof = .935 if police else 1.
        self._loft([(-.498, .35, .23, .47), (-.475, .43, .17, .55),
                    (-.39, .475, .14, .60), (-.23, .495, .14, .62),
                    (.12, .495, .14, .60), (.31, .48, .16, .55),
                    (.44, .43, .21, .48), (.498, .35, .29, .42)], self.BODY, smooth=True)
        self._car_cabin(roof)
        self._lights(z=.36, car=True)
        if police:
            self._box((-.075, -.28, .94), (.035, .28, .975), self.TRIM)
            self._box((-.072, -.275, .975), (.032, -.01, 1.), self.BLUE)
            self._box((-.072, .01, .975), (.032, .275, 1.), self.RED)
            for side in (-1, 1):
                y = side * .497
                self._quad([(-.22, y, .35), (.17, y, .35), (.17, y, .44), (-.22, y, .44)], self.WHITE)
        self._axles((-.31, .31))

    def _car_cabin(self, roof):
        """One continuous roof/rail/pillar shell with flush glass face regions.

        All surfaces share the same ring vertices and smooth normals before
        material boundaries are split for vertex coloring. There is no separate
        roof slab, overlapping trim, or B-pillar interrupting the side glass.
        """
        sections = [
            (-.350, .360, .575, .600), (-.320, .373, .580, .700),
            (-.260, .380, .580, .860), (-.200, .369, .590, .960),
            (-.140, .355, .590, .990), (-.060, .353, .590, 1.000),
            (.025, .350, .580, .995), (.090, .353, .575, .965),
            (.160, .360, .565, .875), (.235, .365, .550, .715),
            (.300, .360, .540, .560),
        ]
        points = []
        for x, width, bottom, top in sections:
            top = bottom + (top - bottom) * ((roof - .59) / (1. - .59))
            bevel = (top - bottom) * .16
            points.extend([
                (x, -width * .78, bottom), (x, width * .78, bottom),
                (x, width * .95, bottom + bevel * .4), (x, width, bottom + bevel),
                (x, width, top - bevel), (x, width * .95, top - bevel * .4),
                (x, width * .78, top), (x, -width * .78, top),
                (x, -width * .95, top - bevel * .4), (x, -width, top - bevel),
                (x, -width, bottom + bevel), (x, -width * .95, bottom + bevel * .4),
            ])
        faces, colors = [], []
        for section in range(len(sections) - 1):
            for corner in range(12):
                a, b = section * 12 + corner, section * 12 + (corner + 1) % 12
                faces.extend([[a, b, b + 12], [a, b + 12, a + 12]])
                side_glass = corner in (3, 9) and 1 <= section <= 8
                end_glass = corner == 6 and section in (0, 1, 2, 7, 8, 9)
                colors.extend([self.GLASS if side_glass or end_glass else self.BODY] * 2)
        last = (len(sections) - 1) * 12
        for corner in range(1, 11):
            faces.extend([[0, corner + 1, corner], [last, last + corner, last + corner + 1]])
            colors.extend([self.BODY] * 2)
        mesh = self._trimesh.Trimesh(vertices=self._coords(points), faces=faces, process=False)
        self._append(mesh, self.BODY, smooth=True, face_colors=colors)

    def _truck(self):
        ambulance = self.kind == "ambulance"
        cargo_end = .13 if ambulance else .10
        self._loft([(-.50, .47, .25, .98), (-.48, .50, .23, 1.),
                    (cargo_end, .50, .23, 1.)], self.WHITE if ambulance else self.BODY)
        self._loft([(cargo_end + .01, .46, .22, .78), (.34, .46, .22, .78),
                    (.50, .40, .24, .55)], self.BODY)
        self._quad([(.497, -.32, .565), (.497, .32, .565),
                    (.355, .38, .768), (.355, -.38, .768)], self.GLASS)
        for side in (-1, 1):
            y = side * .462
            p = [(cargo_end + .04, y, .52), (.34, y, .52),
                 (.34, y, .735), (cargo_end + .04, y, .735)]
            self._quad(p if side < 0 else p[::-1], self.GLASS)
            # The cab tapers ahead of X=.34: a separate triangular pane follows
            # that surface instead of a twisted quad clipping into the body.
            front = [(.34, y, .52), (.455, side * .419, .52), (.34, y, .735)]
            self._poly(front, [[0, 1, 2]] if side < 0 else [[2, 1, 0]], self.GLASS)
            if ambulance:
                self._box((-.45, min(side * .501, side * .498), .47),
                          (cargo_end - .02, max(side * .501, side * .498), .51), self.RED)
                y = side * .503
                self._quad([(-.24, y, .60), (-.18, y, .60), (-.18, y, .86), (-.24, y, .86)], self.RED)
                self._quad([(-.29, y, .68), (-.13, y, .68), (-.13, y, .78), (-.29, y, .78)], self.RED)
        self._lights(z=.38)
        self._axles((-.31, .34) if ambulance else (-.34, -.17, .34))

    def _bus(self):
        articulated = self.kind == "articulated_bus"
        segments = [(-.5, -.075), (.005, .5)] if articulated else [(-.5, .5)]
        for rear, front in segments:
            self._loft([(rear, .45, .15, .97), (rear + .015, .5, .13, 1.),
                        (front - .015, .5, .13, 1.), (front, .45, .15, .97)], self.BODY)
            for side in (-1, 1):
                y = side * .501
                p = [(rear + .026, y, .57), (front - .026, y, .57),
                     (front - .026, y, .865), (rear + .026, y, .865)]
                self._quad(p if side < 0 else p[::-1], self.GLASS)
        if articulated:
            for x in np.linspace(-.073, .003, 10):
                self._box((x, -.475, .19), (x + .003, .475, .94), self.TRIM)
        self._quad([(.5001, -.37, .51), (.5001, .37, .51),
                    (.5001, .37, .90), (.5001, -.37, .90)], self.GLASS)
        self._lights(z=.32)
        self._axles((-.36, -.10, .37) if articulated else (-.34, .34))

    def _trailer(self):
        self._loft([(-.50, .47, .27, .98), (-.485, .5, .25, 1.),
                    (.485, .5, .25, 1.), (.50, .47, .27, .98)], self.BODY)
        self._box((-.44, -.34, .19), (.44, .34, .26), self.TRIM)
        for side in (-1, 1):
            self._box((.29, side * .30 - .012, .03), (.31, side * .30 + .012, .23), self.TRIM)
        self._lights(z=.32)
        self._axles((-.35, -.20, -.05))

    def _loader(self):
        paint = (157, 143, 104, 255)
        self._loft([(-.50, .38, .26, .52), (-.45, .43, .21, .57),
                    (.22, .43, .21, .49)], paint)
        self._loader_cabin(paint)
        for side in (-1, 1):
            self._rod((-.02, side * .33, .39), (.36, side * .33, .22), .08, paint)
        # Open loader bucket: floor, inclined back and side plates.
        self._box((.30, -.5, .025), (.50, .5, .065), paint)
        self._quad([(.30, -.5, .06), (.30, .5, .06), (.25, .5, .30), (.25, -.5, .30)], paint)
        for side in (-1, 1):
            y = side * .5
            self._quad([(.25, y, .30), (.30, y, .06), (.50, y, .06), (.45, y, .20)], paint)
        self._axles((-.31, .18))

    def _loader_cabin(self, paint):
        """Tapered glazing, corner pillars and roof share one closed shell."""
        sections = [
            (.49, -.20, .16, .30), (.55, -.195, .155, .297),
            (.92, -.165, .115, .275), (.97, -.16, .11, .272),
            (1., -.148, .098, .260),
        ]
        points = []
        for z, rear, front, width in sections:
            bevel = .025
            points.extend([
                (rear, -width + bevel, z), (rear + bevel, -width, z),
                (front - bevel, -width, z), (front, -width + bevel, z),
                (front, width - bevel, z), (front - bevel, width, z),
                (rear + bevel, width, z), (rear, width - bevel, z),
            ])
        faces, colors = [], []
        for section in range(len(sections) - 1):
            for corner in range(8):
                a, b = section * 8 + corner, section * 8 + (corner + 1) % 8
                faces.extend([[a, b, b + 8], [a, b + 8, a + 8]])
                glass = section == 1 and corner in (1, 3, 5, 7)
                colors.extend([self.GLASS if glass else paint] * 2)
        last = (len(sections) - 1) * 8
        for corner in range(1, 7):
            faces.extend([[0, corner + 1, corner], [last, last + corner, last + corner + 1]])
            colors.extend([paint] * 2)
        mesh = self._trimesh.Trimesh(vertices=self._coords(points), faces=faces, process=False)
        self._append(mesh, paint, face_colors=colors)

    def _rod_metric(self, start, end, radius, color):
        start, end = np.asarray(start), np.asarray(end)
        delta = end - start
        mesh = self._trimesh.creation.cylinder(radius=radius, height=np.linalg.norm(delta), sections=8)
        transform = self._trimesh.geometry.align_vectors([0, 0, 1], delta)
        transform[:3, 3] = (start + end) / 2
        mesh.apply_transform(transform)
        self._append(mesh, color, smooth=True)

    def _rod(self, start, end, radius, color):
        start, end = self._coords([start, end])
        self._rod_metric(start, end, radius, color)

    def _cycle(self):
        bicycle = self.kind == "bicycle"
        _, radius, _ = self.SPECS[self.kind]
        length, _, height = self.nominal_dimensions
        axle_fraction = .5 - radius / length
        wheel_z = radius / height
        rear, front = (-axle_fraction, 0, wheel_z), (axle_fraction, 0, wheel_z)
        bottom, seat, head = (-.03, 0, .29), (-.12, 0, .77), (.23, 0, .79)
        frame_color = self.BODY if bicycle else self.TRIM
        for a, b in [(rear, seat), (rear, bottom), (seat, bottom), (seat, head), (head, bottom), (head, front)]:
            start = self._vertex_count
            self._rod(a, b, .018 if bicycle else .028, frame_color)
            if a == rear or b == front:
                axle, fixed = (a, b) if a == rear else (b, a)
                nominal_axle, nominal_fixed = self._coords([axle, fixed])
                direction = nominal_axle - nominal_fixed
                rod_vertices = self._parts[-1].vertices
                weights = np.clip((rod_vertices - nominal_fixed) @ direction / (direction @ direction), 0, 1)
                self._axle_links.append((start, self._vertex_count, axle[0], nominal_axle, weights))
        self._box((-.20, -.16, .78), (-.05, .16, .825), self.TIRE)
        self._rod(head, (.24, 0, .975), .018, self.TRIM)
        self._box((.225, -.5, .973), (.255, .5, 1.), self.TRIM)
        if not bicycle:
            self._loft([(-.21, .21, .37, .61), (.02, .24, .35, .72),
                        (.20, .13, .44, .64)], self.BODY, smooth=True)
            self._box((.24, -.13, .75), (.27, .13, .84), self.WHITE)
        for fraction in (-axle_fraction, axle_fraction):
            self._wheel(fraction, 0, open_center=bicycle)


SUPPORTED_KINDS = PrimitiveVehicleAsset.SUPPORTED_KINDS
