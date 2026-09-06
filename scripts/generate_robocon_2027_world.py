#!/usr/bin/env python3
"""Generate the parameterized ABU Robocon 2027 Gazebo field."""

from math import atan2, cos, pi, sin
from pathlib import Path
import xml.etree.ElementTree as ET


MM = 0.001
FIELD_OUTER = 11.0
WALL = 0.05
FIELD_INNER = FIELD_OUTER / 2.0 - WALL
L1_SIZE = 6.0
L1_Z = 0.6
L2_SIZE = 3.0
L2_Z = 0.9
OVERLAY = 0.002


def rgb(red, green, blue):
    """Convert an RGB255 triplet to an SDF RGBA string."""
    return f'{red / 255:.6f} {green / 255:.6f} {blue / 255:.6f} 1'


COLORS = {
    'red_ground': rgb(240, 210, 210),
    'blue_ground': rgb(170, 210, 230),
    'red_zone': rgb(223, 34, 34),
    'blue_zone': rgb(50, 0, 255),
    'red_l1': rgb(235, 180, 160),
    'blue_l1': rgb(150, 215, 220),
    'red_ramp': rgb(200, 150, 140),
    'blue_ramp': rgb(130, 180, 200),
    'red_transfer': rgb(245, 170, 60),
    'blue_transfer': rgb(60, 170, 245),
    'shared': rgb(190, 190, 185),
    'building': rgb(40, 100, 50),
    'barrier': rgb(100, 62, 0),
    'pillar': rgb(100, 62, 0),
    'cup_bottom': rgb(45, 31, 16),
    'gold': rgb(218, 165, 32),
    'grid_light': rgb(235, 245, 220),
    'grid_green': rgb(205, 230, 190),
    'grid_center': rgb(145, 105, 35),
}


def sub(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def pose_text(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    return f'{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}'


def add_material(visual, rgba):
    material = sub(visual, 'material')
    sub(material, 'ambient', rgba)
    sub(material, 'diffuse', rgba)
    sub(material, 'specular', '0.08 0.08 0.08 1')
    sub(material, 'double_sided', 'true')


def add_box(
        model, name, x, y, z, size_x, size_y, size_z, color,
        *, collision=True, roll=0.0, pitch=0.0, yaw=0.0,
        cast_shadows=True):
    link = sub(model, 'link', name=name)
    sub(link, 'pose', pose_text(x, y, z, roll, pitch, yaw))
    if collision:
        collision_node = sub(link, 'collision', name='collision')
        geometry = sub(collision_node, 'geometry')
        box = sub(geometry, 'box')
        sub(box, 'size', f'{size_x:.6f} {size_y:.6f} {size_z:.6f}')
    visual = sub(link, 'visual', name='visual')
    geometry = sub(visual, 'geometry')
    box = sub(geometry, 'box')
    sub(box, 'size', f'{size_x:.6f} {size_y:.6f} {size_z:.6f}')
    sub(visual, 'cast_shadows', str(cast_shadows).lower())
    add_material(visual, color)
    return link


def add_solid_ramp(model, name, x, y, color):
    """Add a closed ramp visual with DART-compatible primitive collisions."""
    run = 3.5
    width = 1.0
    rise = L1_Z
    link = sub(model, 'link', name=name)
    sub(link, 'pose', pose_text(x, y, 0.0))

    visual = sub(link, 'visual', name='visual')
    geometry = sub(visual, 'geometry')
    mesh = sub(geometry, 'mesh')
    sub(mesh, 'uri', 'model://ares_2027sim/meshes/ramp_wedge.obj')
    add_material(visual, color)

    # DART 6.13 cannot construct an SDF mesh collision.  A thin rotated box
    # gives the exact driveable plane while 100 mm box slices fill the wedge
    # beneath it, making its vertical sides and high end collidable.
    slope_length = (run ** 2 + rise ** 2) ** 0.5
    slope = sub(link, 'collision', name='slope_collision')
    sub(slope, 'pose', pose_text(0.0, 0.0, rise / 2.0, -atan2(rise, run), 0.0, 0.0))
    geometry = sub(slope, 'geometry')
    box = sub(geometry, 'box')
    sub(box, 'size', f'{width:.6f} {slope_length:.6f} 0.020000')

    # A separate primitive visual keeps the team-colored driving surface
    # bright and stable regardless of mesh backface or normal handling.
    slope_visual = sub(link, 'visual', name='slope_surface_visual')
    sub(
        slope_visual, 'pose',
        pose_text(0.0, 0.001, rise / 2.0 + 0.005, -atan2(rise, run), 0.0, 0.0),
    )
    geometry = sub(slope_visual, 'geometry')
    box = sub(geometry, 'box')
    sub(box, 'size', f'{width:.6f} {slope_length:.6f} 0.010000')
    add_material(slope_visual, color)

    slices = 35
    slice_length = run / slices
    for index in range(slices):
        center_y = -run / 2.0 + (index + 0.5) * slice_length
        # Use the lower edge of each slice so no collision protrudes through
        # the exact sloped driving plane.
        low_edge_y = center_y + slice_length / 2.0
        height = rise * (run / 2.0 - low_edge_y) / run
        if height <= 0.0:
            continue
        collision = sub(link, 'collision', name=f'fill_collision_{index:02d}')
        sub(collision, 'pose', pose_text(0.0, center_y, height / 2.0))
        geometry = sub(collision, 'geometry')
        box = sub(geometry, 'box')
        sub(box, 'size', f'{width:.6f} {slice_length:.6f} {height:.6f}')

    high_end = sub(link, 'collision', name='high_end_collision')
    sub(high_end, 'pose', pose_text(0.0, -run / 2.0 + 0.005, rise / 2.0))
    geometry = sub(high_end, 'geometry')
    box = sub(geometry, 'box')
    sub(box, 'size', f'{width:.6f} 0.010000 {rise:.6f}')
    return link


def add_cylinder(
        model, name, x, y, z, radius, length, color, *, collision=True):
    link = sub(model, 'link', name=name)
    sub(link, 'pose', pose_text(x, y, z))
    if collision:
        collision_node = sub(link, 'collision', name='collision')
        geometry = sub(collision_node, 'geometry')
        cylinder = sub(geometry, 'cylinder')
        sub(cylinder, 'radius', f'{radius:.6f}')
        sub(cylinder, 'length', f'{length:.6f}')
    visual = sub(link, 'visual', name='visual')
    geometry = sub(visual, 'geometry')
    cylinder = sub(geometry, 'cylinder')
    sub(cylinder, 'radius', f'{radius:.6f}')
    sub(cylinder, 'length', f'{length:.6f}')
    add_material(visual, color)
    return link


def add_overlay(model, name, x, y, z, size_x, size_y, color):
    return add_box(
        model, name, x, y, z + OVERLAY / 2.0,
        size_x, size_y, OVERLAY, color,
        collision=False, cast_shadows=False,
    )


def add_pillar(model, name, x, y, base_z, height):
    """Build a pillar with a collision-capable 180 x 100 mm cylindrical cup."""
    cup_depth = 0.1
    inner_radius = 0.09
    outer_radius = 0.135
    lower_height = height - cup_depth
    add_cylinder(
        model, f'{name}_body', x, y,
        base_z + lower_height / 2.0,
        outer_radius, lower_height, COLORS['pillar'],
    )
    add_cylinder(
        model, f'{name}_cup_bottom', x, y,
        base_z + lower_height - 0.005,
        inner_radius, 0.01, COLORS['cup_bottom'],
    )

    # SDF has no primitive annulus. Overlapping radial boxes form a sealed ring
    # collision while preserving the specified 180 mm cylindrical opening.
    segments = 24
    radial_size = outer_radius - inner_radius
    tangent_size = 2.0 * outer_radius * atan2(sin(pi / segments), cos(pi / segments))
    ring_radius = (inner_radius + outer_radius) / 2.0
    for index in range(segments):
        angle = 2.0 * pi * index / segments
        add_box(
            model,
            f'{name}_cup_ring_{index:02d}',
            x + ring_radius * cos(angle),
            y + ring_radius * sin(angle),
            base_z + height - cup_depth / 2.0,
            radial_size,
            tangent_size,
            cup_depth,
            COLORS['pillar'],
            yaw=angle,
        )


def add_ground(field):
    add_box(
        field, 'ground_collision', 0.0, 0.0, -0.01,
        FIELD_OUTER, FIELD_OUTER, 0.02, COLORS['red_ground'],
    )
    add_overlay(field, 'ground_red', -2.75, 0.0, 0.0, 5.5, 11.0, COLORS['red_ground'])
    add_overlay(field, 'ground_blue', 2.75, 0.0, 0.0, 5.5, 11.0, COLORS['blue_ground'])

    # Storage areas touch both outer walls in the +Y corners.
    add_overlay(field, 'storage_red', -4.45, 4.95, 0.002, 2.0, 1.0, COLORS['red_zone'])
    add_overlay(field, 'storage_blue', 4.45, 4.95, 0.002, 2.0, 1.0, COLORS['blue_zone'])

    # Start zones touch the -Y wall and run inward from each X-side wall.
    for team, sign, color in (
            ('red', -1.0, COLORS['red_zone']),
            ('blue', 1.0, COLORS['blue_zone'])):
        add_overlay(field, f'start_{team}_outer', sign * 5.10, -5.10, 0.002, 0.7, 0.7, color)
        add_overlay(field, f'start_{team}_inner', sign * 4.35, -5.10, 0.002, 0.7, 0.7, color)

    # The two ground shared regions are centered in the free strips.
    add_overlay(field, 'mustika_shared_ground', 0.0, 4.225, 0.002, 1.0, 1.0, COLORS['shared'])
    add_overlay(field, 'sky_shared_ground', 0.0, -4.225, 0.002, 1.2, 1.2, COLORS['shared'])

    # Draw the empty 5 x 5 placement grid; no Sky Blocks are instantiated.
    grid_center_y = -4.225
    for row in range(5):
        for column in range(5):
            x = (column - 2) * 0.2
            y = grid_center_y + (row - 2) * 0.2
            if row == 2 and column == 2:
                color = COLORS['grid_center']
            else:
                color = COLORS['grid_green'] if (row + column) % 2 == 0 else COLORS['grid_light']
            add_overlay(
                field, f'sky_grid_{row}_{column}', x, y, 0.005,
                0.192, 0.192, color,
            )

    # Outer boundary, with the 11 m measurement taken at the outer faces.
    add_box(field, 'wall_ground_x_pos', 5.475, 0.0, 0.025, 0.05, 11.0, 0.05, COLORS['barrier'])
    add_box(field, 'wall_ground_x_neg', -5.475, 0.0, 0.025, 0.05, 11.0, 0.05, COLORS['barrier'])
    add_box(field, 'wall_ground_y_pos', 0.0, 5.475, 0.025, 10.9, 0.05, 0.05, COLORS['barrier'])
    add_box(field, 'wall_ground_y_neg', 0.0, -5.475, 0.025, 10.9, 0.05, 0.05, COLORS['barrier'])

    # Red/blue divider segments stop at each shared region and at L1.
    divider_segments = (
        ('positive_inner', 3.3625, 0.725),
        ('positive_outer', 5.0875, 0.725),
        ('negative_inner', -3.3125, 0.625),
        ('negative_outer', -5.1375, 0.625),
    )
    for suffix, center_y, length in divider_segments:
        add_box(
            field, f'wall_ground_divider_{suffix}',
            0.0, center_y, 0.025, 0.05, length, 0.05,
            COLORS['barrier'],
        )


def add_level_one(field):
    # Solid plinths cover every exposed vertical face between Ground and L1.
    add_box(field, 'l1_red', -1.5, 0.0, L1_Z / 2.0, 3.0, 6.0, L1_Z, COLORS['red_l1'])
    add_box(field, 'l1_blue', 1.5, 0.0, L1_Z / 2.0, 3.0, 6.0, L1_Z, COLORS['blue_l1'])

    for sign, suffix in ((1.0, 'positive'), (-1.0, 'negative')):
        center_y = sign * 2.625
        add_overlay(field, f'l1_shared_{suffix}', 0.0, center_y, L1_Z, 1.0, 0.75, COLORS['shared'])
        # The Building Spot touches the inside face of the outer L1 wall.
        spot_y = sign * (3.0 - WALL - 0.25)
        add_overlay(field, f'l1_shared_spot_{suffix}', 0.0, spot_y, L1_Z + 0.002, 0.5, 0.5, COLORS['building'])

    for team, sign, retry_color in (
            ('red', -1.0, COLORS['red_zone']),
            ('blue', 1.0, COLORS['blue_zone'])):
        for y_suffix, y in (('positive', 2.75), ('negative', -2.75)):
            add_overlay(
                field, f'l1_spot_{team}_{y_suffix}',
                sign * 2.75, y, L1_Z + 0.002,
                0.5, 0.5, COLORS['building'],
            )
        # BR retry zones sit between the exclusive and shared Building Spots,
        # touching the inside face of the +Y L1 wall as shown in Figure 2.
        add_overlay(
            field, f'l1_retry_{team}', sign * 1.5, 2.60,
            L1_Z + 0.002, 0.7, 0.7, retry_color,
        )

    wall_z = L1_Z + WALL / 2.0
    add_box(field, 'wall_l1_y_pos', 0.0, 2.975, wall_z, 6.0, 0.05, 0.05, COLORS['barrier'])
    add_box(field, 'wall_l1_y_neg', 0.0, -2.975, wall_z, 6.0, 0.05, 0.05, COLORS['barrier'])
    for team, sign in (('red', -1.0), ('blue', 1.0)):
        wall_x = sign * 2.975
        add_box(field, f'wall_l1_{team}_before_transfer', wall_x, -2.4, wall_z, 0.05, 1.2, 0.05, COLORS['barrier'])
        add_box(field, f'wall_l1_{team}_after_transfer', wall_x, 1.1, wall_z, 0.05, 3.8, 0.05, COLORS['barrier'])

    # Divider walls occupy the 750 mm gaps between L2 and the shared areas.
    add_box(field, 'wall_l1_divider_positive', 0.0, 1.875, wall_z, 0.05, 0.75, 0.05, COLORS['barrier'])
    add_box(field, 'wall_l1_divider_negative', 0.0, -1.875, wall_z, 0.05, 0.75, 0.05, COLORS['barrier'])


def add_transition_modules(field):
    for team, sign, ramp_color, transfer_color in (
            ('red', -1.0, COLORS['red_ramp'], COLORS['red_transfer']),
            ('blue', 1.0, COLORS['blue_ramp'], COLORS['blue_transfer'])):
        center_x = sign * 3.5
        add_solid_ramp(field, f'ramp_{team}', center_x, 0.95, ramp_color)
        # Transfer Areas are solid up to L1, including their exposed side faces.
        add_box(
            field, f'transfer_{team}', center_x, -1.30, L1_Z / 2.0,
            1.0, 1.0, L1_Z, transfer_color,
        )

        # Four 150 mm rises use three exposed 300 mm treads; the transfer
        # platform itself is the fourth and highest level.
        for index, top_height in enumerate((0.15, 0.30, 0.45), start=1):
            center_y = -2.85 + index * 0.30
            add_box(
                field, f'stair_{team}_{index}', center_x, center_y,
                top_height / 2.0, 1.0, 0.30, top_height, ramp_color,
            )


def add_level_two(field):
    # L2 is a solid 300 mm rise above L1, so all four vertical faces collide.
    l2_rise = L2_Z - L1_Z
    add_box(
        field, 'l2_platform', 0.0, 0.0, L1_Z + l2_rise / 2.0,
        3.0, 3.0, l2_rise, COLORS['shared'],
    )

    # The 300 mm rise is formed by one physical 150 mm tread followed by the
    # gray L2 surface, which acts as the second step.
    for team, sign, color in (
            ('red', -1.0, COLORS['red_l1']),
            ('blue', 1.0, COLORS['blue_l1'])):
        add_box(
            field, f'l2_stair_{team}',
            sign * (L2_SIZE / 2.0 + 0.15), 0.0,
            L1_Z + 0.075,
            0.30, 1.0, 0.15, color,
        )
    for x_suffix, x in (('red', -1.25), ('blue', 1.25)):
        for y_suffix, y in (('positive', 1.25), ('negative', -1.25)):
            add_overlay(
                field, f'l2_spot_{x_suffix}_{y_suffix}',
                x, y, L2_Z, 0.5, 0.5, COLORS['building'],
            )


def add_mustika(world, x, y, center_z):
    model = sub(world, 'model', name='mustika')
    sub(model, 'pose', pose_text(x, y, center_z))
    link = sub(model, 'link', name='mustika_link')
    inertial = sub(link, 'inertial')
    sub(inertial, 'mass', '0.42')
    inertia = sub(inertial, 'inertia')
    for tag, value in (
            ('ixx', 0.00168), ('iyy', 0.00168), ('izz', 0.00168),
            ('ixy', 0.0), ('ixz', 0.0), ('iyz', 0.0)):
        sub(inertia, tag, f'{value:.8f}')
    collision = sub(link, 'collision', name='collision')
    geometry = sub(collision, 'geometry')
    sphere = sub(geometry, 'sphere')
    sub(sphere, 'radius', '0.1')
    visual = sub(link, 'visual', name='visual')
    geometry = sub(visual, 'geometry')
    sphere = sub(geometry, 'sphere')
    sub(sphere, 'radius', '0.1')
    add_material(visual, COLORS['gold'])


def add_dynamic_box(
        world, name, x, y, z, size, mass, color, *, yaw=0.0,
        body_visual=True):
    """Add a free-moving cubic game object with box inertia and collision."""
    model = sub(world, 'model', name=name)
    sub(model, 'pose', pose_text(x, y, z, yaw=yaw))
    link = sub(model, 'link', name=f'{name}_link')

    inertial = sub(link, 'inertial')
    sub(inertial, 'mass', f'{mass:.6f}')
    inertia = sub(inertial, 'inertia')
    diagonal = mass * (size ** 2 + size ** 2) / 12.0
    for tag, value in (
            ('ixx', diagonal), ('iyy', diagonal), ('izz', diagonal),
            ('ixy', 0.0), ('ixz', 0.0), ('iyz', 0.0)):
        sub(inertia, tag, f'{value:.8f}')

    collision = sub(link, 'collision', name='collision')
    geometry = sub(collision, 'geometry')
    box = sub(geometry, 'box')
    sub(box, 'size', f'{size:.6f} {size:.6f} {size:.6f}')
    surface = sub(collision, 'surface')
    friction = sub(surface, 'friction')
    ode = sub(friction, 'ode')
    sub(ode, 'mu', '0.80')
    sub(ode, 'mu2', '0.80')

    if body_visual:
        visual = sub(link, 'visual', name='body')
        geometry = sub(visual, 'geometry')
        box = sub(geometry, 'box')
        sub(box, 'size', f'{size:.6f} {size:.6f} {size:.6f}')
        add_material(visual, color)
    return model, link


def add_earth_blocks(world):
    """Place each team's 20 Earth Blocks legally, at most two blocks high."""
    size = 0.35
    mass = 0.275
    x_offsets = (-0.70, -0.35, 0.0, 0.35, 0.70)
    y_offsets = (-0.175, 0.175)
    for team, center_x, color in (
            ('red', -4.45, COLORS['red_zone']),
            ('blue', 4.45, COLORS['blue_zone'])):
        index = 1
        for layer in range(2):
            for y_offset in y_offsets:
                for x_offset in x_offsets:
                    add_dynamic_box(
                        world, f'earth_block_{team}_{index:02d}',
                        center_x + x_offset, 4.95 + y_offset,
                        size / 2.0 + layer * size,
                        size, mass, color,
                    )
                    index += 1


def add_sky_blocks(world):
    """Place the 12 half-red, half-blue Sky Blocks as shown in Figure 3."""
    size = 0.20
    mass = 0.115
    center_y = -4.225
    # Direction of the red half on the upward split face.  The positions and
    # orientations reproduce the example arrangement in Chinese rule Figure 3.
    red_directions = {
        (0, 0): (0.0, 1.0), (0, 2): (-1.0, 0.0), (0, 4): (0.0, -1.0),
        (1, 1): (-1.0, 0.0), (1, 3): (1.0, 0.0),
        (2, 0): (0.0, -1.0), (2, 4): (0.0, 1.0),
        (3, 1): (1.0, 0.0), (3, 3): (-1.0, 0.0),
        (4, 0): (0.0, 1.0), (4, 2): (1.0, 0.0), (4, 4): (0.0, -1.0),
    }
    for index, ((row, column), (red_x, red_y)) in enumerate(
            red_directions.items(), start=1):
        x = (column - 2) * 0.2
        y = center_y + (2 - row) * 0.2
        yaw = atan2(red_y, red_x)
        _, link = add_dynamic_box(
            world, f'sky_block_{index:02d}', x, y, size / 2.0,
            size, mass, COLORS['red_zone'], yaw=yaw, body_visual=False,
        )

        # The real prop is half red and half blue.  One collision cube contains
        # two half-cube visuals: four faces are split, while the two end faces
        # are solid red and blue.  A 90-degree roll can therefore expose a
        # single-color scoring face on top.
        for team, offset_x, face_color in (
                ('red', size / 4.0, COLORS['red_zone']),
                ('blue', -size / 4.0, COLORS['blue_zone'])):
            visual = sub(link, 'visual', name=f'{team}_half')
            sub(visual, 'pose', pose_text(offset_x, 0.0, 0.0))
            geometry = sub(visual, 'geometry')
            box = sub(geometry, 'box')
            sub(box, 'size', f'{size / 2.0:.6f} {size:.6f} {size:.6f}')
            add_material(visual, face_color)


def build_world():
    sdf = ET.Element('sdf', {'version': '1.9'})
    world = sub(sdf, 'world', name='robocon_2027')
    sub(world, 'gravity', '0 0 -9.8')
    sub(world, 'magnetic_field', '0.000006 0.000023 -0.000042')
    scene = sub(world, 'scene')
    sub(scene, 'ambient', '0.62 0.62 0.62 1')
    sub(scene, 'background', '0.82 0.84 0.87 1')
    sub(scene, 'shadows', 'true')

    sub(world, 'plugin', name='gz::sim::systems::Physics', filename='gz-sim-physics-system')
    sub(world, 'plugin', name='gz::sim::systems::UserCommands', filename='gz-sim-user-commands-system')
    sub(world, 'plugin', name='gz::sim::systems::SceneBroadcaster', filename='gz-sim-scene-broadcaster-system')

    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 10 0 0 0')
    sub(light, 'diffuse', '0.92 0.92 0.92 1')
    sub(light, 'specular', '0.2 0.2 0.2 1')
    sub(light, 'direction', '-0.35 0.25 -0.9')
    sub(light, 'cast_shadows', 'true')

    field = sub(world, 'model', name='robocon_2027_field')
    sub(field, 'static', 'true')
    add_ground(field)
    add_level_one(field)
    add_transition_modules(field)
    add_level_two(field)
    add_pillar(field, 'mustika_pillar', 0.0, 4.225, 0.0, 0.5)
    add_pillar(field, 'central_pillar', 0.0, 0.0, L2_Z, 0.8)

    add_earth_blocks(world)
    add_sky_blocks(world)

    # A 200 mm ball rests on the 180 mm rim at approximately z=0.544 m.
    add_mustika(world, 0.0, 4.225, 0.545)
    ET.indent(sdf, space='  ')
    return ET.ElementTree(sdf)


def main():
    output = Path(__file__).resolve().parents[1] / 'worlds' / 'robocon_2027.sdf'
    tree = build_world()
    tree.write(output, encoding='utf-8', xml_declaration=True)
    print(output)


if __name__ == '__main__':
    main()
