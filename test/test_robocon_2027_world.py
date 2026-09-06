from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


WORLD_FILE = (
    Path(__file__).resolve().parents[1]
    / 'worlds'
    / 'robocon_2027.sdf'
)


@pytest.fixture(scope='module')
def world_root():
    return ET.parse(WORLD_FILE).getroot().find('world')


def _field_links(world_root):
    field = world_root.find("model[@name='robocon_2027_field']")
    assert field is not None
    return {link.attrib['name']: link for link in field.findall('link')}


def _pose(link):
    return tuple(float(value) for value in link.findtext('pose').split())


def _box_size(link):
    text = link.findtext('visual/geometry/box/size')
    return tuple(float(value) for value in text.split())


def _diffuse(link):
    return link.findtext('visual/material/diffuse')


def test_world_is_valid_sdf_with_required_models(world_root):
    assert world_root is not None
    assert world_root.attrib['name'] == 'robocon_2027'
    assert world_root.find("model[@name='robocon_2027_field']") is not None
    assert world_root.find("model[@name='mustika']") is not None


def _dynamic_models(world_root, prefix):
    return [
        model for model in world_root.findall('model')
        if model.attrib.get('name', '').startswith(prefix)
    ]


def _model_pose(model):
    return tuple(float(value) for value in model.findtext('pose').split())


def test_earth_blocks_have_rule_dimensions_mass_and_legal_layout(world_root):
    for team, center_x in (('red', -4.45), ('blue', 4.45)):
        models = _dynamic_models(world_root, f'earth_block_{team}_')
        assert len(models) == 20
        for model in models:
            link = model.find('link')
            assert float(link.findtext('inertial/mass')) == pytest.approx(0.275)
            size = tuple(float(value) for value in link.findtext('collision/geometry/box/size').split())
            assert size == pytest.approx((0.35, 0.35, 0.35))
            x, y, z = _model_pose(model)[:3]
            assert center_x - 1.0 <= x <= center_x + 1.0
            assert 4.45 <= y <= 5.45
            assert any(z == pytest.approx(expected) for expected in (0.175, 0.525))


def test_sky_blocks_match_figure_three_layout_and_two_color_geometry(world_root):
    models = _dynamic_models(world_root, 'sky_block_')
    assert len(models) == 12
    positions = set()
    orientations = set()
    for model in models:
        link = model.find('link')
        assert float(link.findtext('inertial/mass')) == pytest.approx(0.115)
        size = tuple(float(value) for value in link.findtext('collision/geometry/box/size').split())
        assert size == pytest.approx((0.2, 0.2, 0.2))
        x, y, z, _, _, yaw = _model_pose(model)
        positions.add((round(x, 6), round(y, 6)))
        orientations.add(round(yaw, 6))
        assert z == pytest.approx(0.1)
        assert link.find("visual[@name='body']") is None
        red_half = link.find("visual[@name='red_half']")
        blue_half = link.find("visual[@name='blue_half']")
        assert red_half is not None
        assert blue_half is not None
        for half in (red_half, blue_half):
            half_size = tuple(float(value) for value in half.findtext('geometry/box/size').split())
            assert half_size == pytest.approx((0.1, 0.2, 0.2))
        assert red_half.findtext('material/diffuse') != blue_half.findtext('material/diffuse')
    assert (0.0, -4.225) not in positions
    expected_positions = {
        (round((column - 2) * 0.2, 6), round(-4.225 + (2 - row) * 0.2, 6))
        for row in range(5) for column in range(5)
        if (row + column) % 2 == 0 and (row, column) != (2, 2)
    }
    assert positions == expected_positions
    assert orientations == {-1.570796, 0.0, 1.570796, 3.141593}


def test_platform_dimensions_and_heights(world_root):
    links = _field_links(world_root)
    assert _box_size(links['ground_collision'])[:2] == (11.0, 11.0)
    assert _box_size(links['l1_red'])[:2] == (3.0, 6.0)
    assert _pose(links['l1_red'])[0] == pytest.approx(-1.5)
    assert _pose(links['l1_blue'])[0] == pytest.approx(1.5)
    assert _pose(links['l1_red'])[2] == pytest.approx(0.30)
    assert _box_size(links['l1_red'])[2] == pytest.approx(0.60)
    assert _box_size(links['l2_platform'])[:2] == (3.0, 3.0)
    assert _pose(links['l2_platform'])[2] == pytest.approx(0.75)
    assert _box_size(links['l2_platform'])[2] == pytest.approx(0.30)


def test_l1_to_l2_uses_one_tread_then_l2_surface(world_root):
    links = _field_links(world_root)
    for team, sign in (('red', -1.0), ('blue', 1.0)):
        stair = links[f'l2_stair_{team}']
        assert _box_size(stair) == pytest.approx((0.30, 1.0, 0.15))
        assert _pose(stair)[:3] == pytest.approx((sign * 1.65, 0.0, 0.675))
        assert _pose(stair)[2] + _box_size(stair)[2] / 2.0 == pytest.approx(0.75)
        assert f'l2_stair_{team}_2' not in links


def test_l1_perimeter_walls_use_barrier_color(world_root):
    links = _field_links(world_root)
    barrier_color = _diffuse(links['wall_ground_x_pos'])
    perimeter_names = (
        'wall_l1_y_pos',
        'wall_l1_y_neg',
        'wall_l1_red_before_transfer',
        'wall_l1_red_after_transfer',
        'wall_l1_blue_before_transfer',
        'wall_l1_blue_after_transfer',
    )
    for name in perimeter_names:
        assert _diffuse(links[name]) == barrier_color


def test_elevated_structures_have_solid_collidable_sides(world_root):
    links = _field_links(world_root)
    for name in ('l1_red', 'l1_blue', 'l2_platform', 'transfer_red', 'transfer_blue'):
        assert links[name].find('collision/geometry') is not None
        assert links[name].find('visual/geometry') is not None
    assert _box_size(links['transfer_red'])[2] == pytest.approx(0.60)
    for name in ('ramp_red', 'ramp_blue'):
        assert links[name].findtext('visual/geometry/mesh/uri') == 'model://ares_2027sim/meshes/ramp_wedge.obj'
        collisions = links[name].findall('collision')
        assert len(collisions) == 36
        assert all(collision.find('geometry/box') is not None for collision in collisions)
        assert links[name].find('collision/geometry/mesh') is None
        slope_visual = links[name].find("visual[@name='slope_surface_visual']")
        assert slope_visual is not None
        slope_size = tuple(
            float(value)
            for value in slope_visual.findtext('geometry/box/size').split()
        )
        assert slope_size == pytest.approx((1.0, (3.5 ** 2 + 0.6 ** 2) ** 0.5, 0.01))


def test_ground_zone_coordinates(world_root):
    links = _field_links(world_root)
    assert _pose(links['mustika_shared_ground'])[:2] == (0.0, 4.225)
    assert _box_size(links['mustika_shared_ground'])[:2] == (1.0, 1.0)
    assert _pose(links['sky_shared_ground'])[:2] == (0.0, -4.225)
    assert _box_size(links['sky_shared_ground'])[:2] == (1.2, 1.2)
    assert _pose(links['storage_red'])[:2] == (-4.45, 4.95)
    assert _pose(links['storage_blue'])[:2] == (4.45, 4.95)
    assert _box_size(links['storage_red'])[:2] == (2.0, 1.0)
    assert _diffuse(links['storage_red']) == _diffuse(links['l1_retry_red'])
    assert _diffuse(links['storage_blue']) == _diffuse(links['l1_retry_blue'])


def test_start_zones_touch_the_negative_y_and_side_walls(world_root):
    links = _field_links(world_root)
    expected = {
        'start_red_outer': (-5.10, -5.10),
        'start_red_inner': (-4.35, -5.10),
        'start_blue_outer': (5.10, -5.10),
        'start_blue_inner': (4.35, -5.10),
    }
    for name, xy in expected.items():
        assert _pose(links[name])[:2] == xy
        assert _box_size(links[name])[:2] == (0.7, 0.7)


def test_building_spot_count_and_coordinates(world_root):
    links = _field_links(world_root)
    spot_links = {
        name: link for name, link in links.items()
        if name.startswith('l1_spot_')
        or name.startswith('l1_shared_spot_')
        or name.startswith('l2_spot_')
    }
    assert len(spot_links) == 10
    assert _pose(links['l1_shared_spot_positive'])[:2] == (0.0, 2.70)
    assert _pose(links['l1_shared_spot_negative'])[:2] == (0.0, -2.70)
    assert _pose(links['l1_spot_red_positive'])[:2] == (-2.75, 2.75)
    assert _pose(links['l1_spot_blue_negative'])[:2] == (2.75, -2.75)
    assert _pose(links['l2_spot_red_positive'])[:2] == (-1.25, 1.25)
    assert _pose(links['l2_spot_blue_negative'])[:2] == (1.25, -1.25)
    for link in spot_links.values():
        assert _box_size(link)[:2] == (0.5, 0.5)


def test_l1_br_retry_zones_are_mirrored_and_touch_wall(world_root):
    links = _field_links(world_root)
    assert _pose(links['l1_retry_red'])[:2] == (-1.5, 2.60)
    assert _pose(links['l1_retry_blue'])[:2] == (1.5, 2.60)
    for team in ('red', 'blue'):
        retry = links[f'l1_retry_{team}']
        assert _box_size(retry)[:2] == (0.7, 0.7)
        outer_edge = _pose(retry)[1] + _box_size(retry)[1] / 2.0
        assert outer_edge == pytest.approx(2.95)


def test_transition_modules_are_mirrored_and_centered(world_root):
    links = _field_links(world_root)
    assert _pose(links['ramp_red'])[:3] == (-3.5, 0.95, 0.0)
    assert _pose(links['ramp_blue'])[:3] == (3.5, 0.95, 0.0)
    assert _pose(links['transfer_red'])[:3] == (-3.5, -1.30, 0.30)
    assert _pose(links['transfer_blue'])[:3] == (3.5, -1.30, 0.30)
    assert _pose(links['stair_red_1'])[:3] == (-3.5, -2.55, 0.075)
    assert _pose(links['stair_red_3'])[:3] == (-3.5, -1.95, 0.225)
    assert links['ramp_red'].findtext('visual/geometry/mesh/uri') == 'model://ares_2027sim/meshes/ramp_wedge.obj'


def test_transfer_openings_are_one_metre_wide(world_root):
    links = _field_links(world_root)
    for team in ('red', 'blue'):
        before = links[f'wall_l1_{team}_before_transfer']
        after = links[f'wall_l1_{team}_after_transfer']
        before_max_y = _pose(before)[1] + _box_size(before)[1] / 2.0
        after_min_y = _pose(after)[1] - _box_size(after)[1] / 2.0
        assert after_min_y - before_max_y == pytest.approx(1.0)


def test_pillar_cups_have_collision_ring_and_mustika(world_root):
    links = _field_links(world_root)
    mustika_ring = [name for name in links if name.startswith('mustika_pillar_cup_ring_')]
    central_ring = [name for name in links if name.startswith('central_pillar_cup_ring_')]
    assert len(mustika_ring) == 24
    assert len(central_ring) == 24
    mustika_pose = tuple(
        float(value)
        for value in world_root.find("model[@name='mustika']/pose").text.split()
    )
    assert mustika_pose[:3] == (0.0, 4.225, 0.545)
