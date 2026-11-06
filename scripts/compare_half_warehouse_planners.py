#!/usr/bin/env python3
"""Audit two completed half-warehouse planner flights and write a comparison."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def read_result(path):
    path = Path(path).resolve()
    if not path.name.endswith('_result.json'):
        raise ValueError('Expected a *_result.json file: {}'.format(path))
    prefix = str(path)[:-len('_result.json')]
    result = json.loads(path.read_text(encoding='utf-8'))
    shelf = json.loads(Path(prefix + '_shelf_audit.json').read_text(encoding='utf-8'))
    scene = json.loads(Path(prefix + '_scene.json').read_text(encoding='utf-8'))
    history = json.loads(Path(prefix + '_status_history.json').read_text(encoding='utf-8'))
    saved_map = json.loads(Path(prefix + '_final_map.json').read_text(encoding='utf-8'))
    pcd = Path(prefix + '_final_map.pcd')
    if hashlib.sha256(pcd.read_bytes()).hexdigest() != saved_map['sha256']:
        raise ValueError('Saved PCD hash mismatch: {}'.format(pcd))
    walk = Path(prefix + '_walk_once.log').read_text(encoding='utf-8')
    return dict(path=path, prefix=prefix, result=result, shelf=shelf, scene=scene,
                history=history, saved_map=saved_map,
                pedestrian_walk_done='Automatic walk done' in walk)


def compare_conditions(a, b, deployed=None):
    problems = []
    for key in ('source_sha256', 'bounds_world', 'spawn_world', 'kept_shelves',
                'boxes', 'pedestrian_world'):
        if a['scene'].get(key) != b['scene'].get(key):
            problems.append('Scene mismatch: ' + key)
    for key in ('scene', 'headless', 'no_gazebo_gui', 'no_rviz', 'keep_open', 'manual_pedestrian',
                'freedom_baseline', 'return_after_goals', 'test_seconds', 'max_wall_seconds'):
        if a['result']['run_options'].get(key) != b['result']['run_options'].get(key):
            problems.append('Run option mismatch: ' + key)
    ac, bc = a['result'].get('configuration', {}), b['result'].get('configuration', {})
    for key in sorted(set(ac) | set(bc)):
        if key in ('/warehouse_explorer/planner_mode', '/warehouse_explorer/coverage_region_size'):
            continue
        if ac.get(key) != bc.get(key):
            problems.append('ROS parameter mismatch: ' + key)
    if ac.get('/warehouse_explorer/planner_mode') != 'frontier':
        problems.append('First flight is not frontier mode')
    if bc.get('/warehouse_explorer/planner_mode') != 'coverage_guided':
        problems.append('Second flight is not coverage_guided mode')
    if ac.get('/warehouse_explorer/resolution') != .15 or bc.get('/warehouse_explorer/resolution') != .15:
        problems.append('Both planning grids must be 0.15 m')
    if not a['pedestrian_walk_done'] or not b['pedestrian_walk_done']:
        problems.append('Automatic pedestrian walk completion missing in one or both flights')
    if deployed is not None:
        for key in ('source_sha256', 'bounds_world', 'spawn_world', 'kept_shelves',
                    'boxes', 'pedestrian_world'):
            if deployed['scene'].get(key) != a['scene'].get(key):
                problems.append('Deployed scene mismatch: ' + key)
        for key in ('scene', 'headless', 'no_gazebo_gui', 'no_rviz', 'keep_open', 'manual_pedestrian',
                    'freedom_baseline', 'return_after_goals', 'test_seconds', 'max_wall_seconds'):
            if deployed['result']['run_options'].get(key) != a['result']['run_options'].get(key):
                problems.append('Deployed run option mismatch: ' + key)
        dc = deployed['result'].get('configuration', {})
        for key in sorted(set(dc) | set(ac)):
            if key in ('/warehouse_explorer/planner_mode', '/warehouse_explorer/resolution'):
                continue
            if dc.get(key) != ac.get(key):
                problems.append('Deployed ROS parameter mismatch: ' + key)
        if deployed['result']['mission'].get('planner_mode') != 'frontier':
            problems.append('Deployed flight is not frontier mode')
        if dc.get('/warehouse_explorer/resolution') != .20:
            problems.append('Deployed planning grid must be 0.20 m')
        if not deployed['pedestrian_walk_done']:
            problems.append('Deployed automatic pedestrian walk completion missing')
    for label, record in [('frontier 15 cm', a), ('coverage guided 15 cm', b),
                          *([('original 20 cm', deployed)] if deployed is not None else [])]:
        if not record['result'].get('task_success') or not record['result'].get('map_saved'):
            problems.append(label + ' did not return and save successfully')
    return problems


def metric(record, key):
    result, mission, shelf = record['result'], record['result']['mission'], record['shelf']
    values = {
        'mission_sim_seconds': mission.get('mission_sim_seconds'),
        'return_start_sim_seconds': result.get('timing', {}).get('sim_seconds_at_return'),
        'mission_wall_seconds': mission.get('wall_seconds'),
        'total_wall_seconds': result.get('wall_elapsed_seconds'),
        'estimated_rtf': mission.get('estimated_rtf'),
        'distance_m': mission.get('travel_xy_m'),
        'goals_reached': mission.get('goals_reached'),
        'known_free_area_m2': mission.get('known_free_area_m2'),
        'surface_quality_pct': 100 * mission.get('surface_quality', {}).get('observed_surface_quality_ratio', 0),
        'shelf_side_pct': 100 * shelf['shelf_side_coverage'],
        'shelf_min_rack_pct': 100 * shelf['minimum_rack_coverage'],
        'map_points': record['saved_map'].get('points'),
        'compute_p95_ms': mission.get('compute_p95_ms'),
        'residual_useful_frontiers': mission.get('useful_frontier_candidates'),
    }
    return values[key]


def fmt(value, decimals=2):
    return '—' if value is None else ('{:.{}f}'.format(value, decimals))


def curve(record, key):
    x, y = [], []
    for row in record['history']:
        t = row.get('mission_sim_seconds')
        if t is None:
            continue
        value = row.get('surface_quality', {}).get('observed_surface_quality_ratio') if key == 'surface' else row.get(key)
        if value is None:
            continue
        if x and t < x[-1]:
            continue
        x.append(t)
        y.append(100 * value if key == 'surface' else value)
    return x, y


def plot_curves(a, b, output, deployed=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.7))
    specs = [('known_free_area_m2', 'Observed free area (m²)'),
             ('surface', 'Observed surface quality (%)'),
             ('travel_xy_m', 'Flight distance (m)')]
    for ax, (key, ylabel) in zip(axes, specs):
        series = [(a, 'Frontier 15 cm', '#1f78b4'),
                  (b, 'Coverage guided 15 cm', '#d95f02')]
        if deployed is not None:
            series.insert(0, (deployed, 'Original frontier 20 cm', '#666666'))
        for record, label, color in series:
            x, y = curve(record, key)
            ax.plot(x, y, label=label, color=color, linewidth=1.4)
        ax.set(xlabel='Mission simulation time (s)', ylabel=ylabel)
        ax.grid(alpha=.25)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frontier_result')
    parser.add_argument('guided_result')
    parser.add_argument('--deployed-result', help='Optional original 20 cm frontier run')
    parser.add_argument('--output', required=True, help='Directory for Markdown, JSON and progress plot')
    args = parser.parse_args()
    a, b = read_result(args.frontier_result), read_result(args.guided_result)
    deployed = read_result(args.deployed_result) if args.deployed_result else None
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    problems = compare_conditions(a, b, deployed)
    keys = [
        ('mission_sim_seconds', 'Task simulation time (s)'),
        ('return_start_sim_seconds', 'Return started at simulation time (s)'),
        ('mission_wall_seconds', 'Task real time (s)'),
        ('total_wall_seconds', 'Total real time (s)'),
        ('estimated_rtf', 'Estimated real-time factor'),
        ('distance_m', 'Flight distance (m)'),
        ('goals_reached', 'Goals reached'),
        ('known_free_area_m2', 'Known free area (m²)'),
        ('surface_quality_pct', 'Observed surface quality (%)'),
        ('shelf_side_pct', 'Offline shelf-side support (%)'),
        ('shelf_min_rack_pct', 'Minimum rack support (%)'),
        ('map_points', 'Saved XYZ points'),
        ('compute_p95_ms', 'Planner compute P95 (ms)'),
        ('residual_useful_frontiers', 'Residual useful frontiers'),
    ]
    lines = ['# Half-warehouse exploration target-selection comparison', '']
    if deployed is not None:
        lines.append('Original frontier 20 cm: `{}`'.format(deployed['path']))
    lines += ['Frontier 15 cm: `{}`'.format(a['path']),
             'Coverage guided: `{}`'.format(b['path']), '',
             'Target protocol: same half-warehouse source, automatic pedestrian script, '
             'speed and safety parameters. The matched control and guided arm both use '
             'a 15 cm fixed-altitude planning grid. '
             'The guided arm is this project’s FALCON-inspired engineering demo; '
             'the original FALCON implementation was not run. Both arms retain the '
             'same safe-grid path search and flight controller; the intervention '
             'is global frontier/region target ordering.', '',
             'Condition audit: **{}**.'.format('passed' if not problems else 'FAILED'), '']
    if problems:
        lines.extend('- ' + issue for issue in problems)
        lines += ['', '**The conditions differ; do not use the metric table as a controlled planner comparison.**', '']
    if deployed is not None:
        lines += ['The original 20 cm arm is the as-deployed configuration. Comparing it '
                  'directly with guided 15 cm changes both planning resolution and target '
                  'selection; the 15 cm frontier arm isolates target selection.', '']
    header = '| Metric | ' + ('Original 20 cm | ' if deployed is not None else '') + \
             'Frontier 15 cm | Coverage guided 15 cm | Guided − frontier 15 cm |'
    separator = '|---|' + ('---:|' if deployed is not None else '') + '---:|---:|---:|'
    lines += [header, separator]
    for key, label in keys:
        first, second = metric(a, key), metric(b, key)
        delta = None if first is None or second is None else second - first
        decimals = 0 if key in ('goals_reached', 'map_points', 'residual_useful_frontiers') else 2
        values = [label]
        if deployed is not None:
            values.append(fmt(metric(deployed, key), decimals))
        values += [fmt(first, decimals), fmt(second, decimals), fmt(delta, decimals)]
        lines.append('| ' + ' | '.join(values) + ' |')
    lines += ['', '| Outcome | ' + ('Original 20 cm | ' if deployed is not None else '') +
              'Frontier 15 cm | Coverage guided 15 cm |',
              '|---|' + ('---|' if deployed is not None else '') + '---|---|']
    for label, path in [('Return and save', ('task_success',)),
                        ('Termination', ('termination',)),
                        ('Finish reason', ('mission', 'finish_reason')),
                        ('Relaxed acceptance triggered', ('mission', 'observed_map_acceptance', 'accepted')),
                        ('Exploration complete', ('mission', 'exploration_complete')),
                        ('3D coverage complete', ('mission', 'coverage_complete'))]:
        def get(record):
            value = record['result']
            for part in path:
                value = value.get(part, {}) if isinstance(value, dict) else None
            return value
        values = [label]
        if deployed is not None:
            values.append(str(get(deployed)))
        values += [str(get(a)), str(get(b))]
        lines.append('| ' + ' | '.join(values) + ' |')
    lines += ['', 'The shelf metric uses exposed collision-box shelf sides at 0.25 m tolerance; '
              'it is not 3D map completeness. The observed-surface quality ratio uses only '
              'already observed surfaces. Known free area is estimated from each planner grid, '
              'so comparisons across 20 cm and 15 cm include discretization effects. '
              'A point count is not a quality score. '
              'This is one sequential flight per strategy, with independent sensor and pedestrian '
              'timing; differences are descriptive, not a statistical or causal superiority claim.', '',
              'Commands used, in order (each with automatic pedestrian and no target cap):', '',
              '```bash',
              'bash scripts/run_sh/half_warehouse_frontier_15cm_demo.sh --headless --max-wall-seconds 1500',
              'bash scripts/run_sh/half_warehouse_falcon_guided_demo.sh --headless --max-wall-seconds 1500']
    if deployed is not None:
        lines.append('bash scripts/run_sh/half_warehouse_exploration_demo.sh --headless --max-wall-seconds 1500')
    lines += ['```', '', 'ROS parameter snapshots: `frontier_15cm_rosparams.yaml`, '
              '`coverage_guided_rosparams.yaml`' +
              (', `original_20cm_rosparams.yaml`' if deployed is not None else '') +
              '. Runtime source hashes: `source_sha256.txt`.', '',
              '![Progress versus simulation time](progress_comparison.png)', '',
              '![Frontier 15 cm map and trajectory](frontier_map_and_trajectory.png)', '',
              '![Coverage-guided map and trajectory](coverage_guided_map_and_trajectory.png)', '']
    if deployed is not None:
        shutil.copyfile(deployed['prefix'] + '_map_and_trajectory.png', output / 'original_20cm_map_and_trajectory.png')
        lines += ['![Original 20 cm map and trajectory](original_20cm_map_and_trajectory.png)', '']
    shutil.copyfile(a['prefix'] + '_map_and_trajectory.png', output / 'frontier_map_and_trajectory.png')
    shutil.copyfile(b['prefix'] + '_map_and_trajectory.png', output / 'coverage_guided_map_and_trajectory.png')
    (output / 'comparison.md').write_text('\n'.join(lines), encoding='utf-8')
    summary = {'frontier_result': str(a['path']), 'guided_result': str(b['path']),
               'deployed_result': str(deployed['path']) if deployed is not None else None,
               'condition_problems': problems,
               'metrics': {key: dict({'frontier': metric(a, key), 'guided': metric(b, key)},
                                    **({'deployed': metric(deployed, key)} if deployed is not None else {}))
                           for key, _ in keys},
               'deployed_task_success': deployed['result'].get('task_success') if deployed is not None else None,
               'frontier_task_success': a['result'].get('task_success'),
               'guided_task_success': b['result'].get('task_success')}
    (output / 'comparison.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    plot_curves(a, b, output / 'progress_comparison.png', deployed)
    print(output / 'comparison.md')
    if problems:
        raise SystemExit('Comparison condition audit failed; see report')


if __name__ == '__main__':
    main()
