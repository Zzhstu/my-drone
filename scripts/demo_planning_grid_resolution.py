#!/usr/bin/env python3
"""Offline, same-input 20 cm / 15 cm demo using the real exploration grid.

This is a deterministic planner illustration, not a Gazebo/PX4 flight or a
measurement of the half-warehouse geometry. Existing SITL flights are reported
separately as context.
"""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BRIDGE_SCRIPTS = ROOT / 'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'
sys.path.insert(0, str(BRIDGE_SCRIPTS))
from exploration_grid import ExplorationGrid  # noqa: E402
from exploration_native import SEARCH  # noqa: E402


BOUNDS = (-4., -3., 4., 3.)
RESOLUTIONS = (.20, .15)
CLEARANCE = .80
UNKNOWN_MARGIN = .40
GAP_CENTRE_Y = .18
GAP_WIDTH_Y = 1.80
WALL_X_AT_ZERO = -.15
WALL_SLOPE = -.15
NARROW_START = (-2.5, GAP_CENTRE_Y)
NARROW_GOAL = (2.5, GAP_CENTRE_Y)
SPARSE_START = (-2.5, 0.)
SPARSE_GOAL = (2.5, 0.)


def point_grid(step):
    x, y = np.meshgrid(np.arange(-3.9, 3.91, step),
                       np.arange(-2.9, 2.91, step))
    return np.column_stack((x.ravel(), y.ravel()))


def observations():
    wall_y = np.arange(-3., 3.001, .05)
    wall_y = wall_y[np.abs(wall_y-GAP_CENTRE_Y) >= GAP_WIDTH_Y/2.]
    wall = np.column_stack((WALL_X_AT_ZERO+WALL_SLOPE*wall_y, wall_y))
    dense_free = point_grid(.05)
    # A free sample is not placed on the synthetic wall. The SAME surviving
    # samples are rasterized at both planning resolutions.
    near_wall = (np.abs(dense_free[:, 0] -
                        (WALL_X_AT_ZERO+WALL_SLOPE*dense_free[:, 1])) < .10)
    on_wall_span = np.abs(dense_free[:, 1]-GAP_CENTRE_Y) >= GAP_WIDTH_Y/2.
    dense_free = dense_free[~(near_wall & on_wall_span)]
    angles = np.linspace(0., 2.*np.pi, 32, endpoint=False)
    person = np.column_stack((WALL_X_AT_ZERO+WALL_SLOPE*GAP_CENTRE_Y+
                              .20*np.cos(angles),
                              GAP_CENTRE_Y+.20*np.sin(angles)))
    sparse_free = point_grid(.16)
    return dense_free, sparse_free, wall, person


def route_point_clearance(path_xy, obstacles):
    if len(path_xy) < 2 or not len(obstacles):
        return None
    samples = []
    for a, b in zip(path_xy[:-1], path_xy[1:]):
        count = max(2, int(np.ceil(np.linalg.norm(b-a)/.02))+1)
        samples.extend(a + t*(b-a) for t in np.linspace(0., 1., count))
    samples = np.asarray(samples)
    nearest = np.linalg.norm(samples[:, None, :]-obstacles[None, :, :], axis=2)
    return float(nearest.min())


def evaluate(grid, obstacles, start_xy, goal_xy):
    grid.update_obstacles(obstacles)
    start, goal = grid.cell(start_xy), grid.cell(goal_xy)
    distances, parents = grid.search(start, clearance_weight=1.50)
    reachable = bool(grid.inside(goal) and np.isfinite(distances[goal]))
    path_cells = grid.path(parents, start, goal) if reachable else []
    path_xy = np.asarray([grid.xy(c) for c in path_cells], dtype=float).reshape(-1, 2)
    if len(path_xy) > 1 and any(not grid.segment_safe(tuple(a), tuple(b))
                                for a, b in zip(path_xy[:-1], path_xy[1:])):
        raise RuntimeError('Search returned a path with a grid-unsafe segment')
    # Path length uses geometric waypoints; Dijkstra's distance is a weighted
    # cost and cannot be reported as actual flight distance.
    length = (float(np.linalg.norm(np.diff(path_xy, axis=0), axis=1).sum())
              if len(path_xy) > 1 else None)
    cells = np.zeros(grid.shape, dtype=np.uint8)
    cells[grid.known] = 1
    cells[grid.safe] = 2
    cells[grid.occupied] = 3
    metrics = dict(resolution_m=grid.resolution,
                   grid_cells=int(np.prod(grid.shape)),
                   known_cells=int(grid.known.sum()),
                   known_area_raster_estimate_m2=float(grid.known.sum()*grid.resolution**2),
                   safe_cells=int(grid.safe.sum()),
                   start_safe=bool(grid.inside(start) and grid.safe[start]),
                   goal_safe=bool(grid.inside(goal) and grid.safe[goal]),
                   reachable=reachable,
                   path_length_m=length,
                   minimum_obstacle_point_clearance_m=route_point_clearance(path_xy, obstacles))
    return dict(metrics=metrics, cells=cells, path_xy=path_xy,
                obstacles=obstacles.copy(), start=start_xy, goal=goal_xy)


def run_synthetic():
    dense_free, sparse_free, wall, person = observations()
    cases = {}
    for resolution in RESOLUTIONS:
        grid = ExplorationGrid(BOUNDS, resolution, CLEARANCE, UNKNOWN_MARGIN)
        grid.observe_free(dense_free)
        occupied_with_person = np.vstack((wall, person))
        key = '{:.2f}'.format(resolution)
        cases[key] = {
            'open_aisle': evaluate(grid, wall, NARROW_START, NARROW_GOAL),
            'person_in_aisle': evaluate(grid, occupied_with_person, NARROW_START, NARROW_GOAL),
            'person_cleared': evaluate(grid, wall, NARROW_START, NARROW_GOAL),
        }
        sparse_grid = ExplorationGrid(BOUNDS, resolution, CLEARANCE, UNKNOWN_MARGIN)
        sparse_grid.observe_free(sparse_free)
        cases[key]['sparse_free_open_room'] = evaluate(
            sparse_grid, np.empty((0, 2)), SPARSE_START, SPARSE_GOAL)
    # Fail loudly if an implementation change invalidates the illustration.
    expected = {
        '0.20': (False, False, False, True),
        '0.15': (True, False, True, False),
    }
    stages = ('open_aisle', 'person_in_aisle', 'person_cleared', 'sparse_free_open_room')
    for resolution, outcomes in expected.items():
        actual = tuple(cases[resolution][s]['metrics']['reachable'] for s in stages)
        if actual != outcomes:
            raise RuntimeError('Synthetic scene no longer illustrates the stated effect: '
                               '{} expected {}, got {}'.format(resolution, outcomes, actual))
    clearance = cases['0.15']['open_aisle']['metrics']['minimum_obstacle_point_clearance_m']
    if clearance is None or clearance < CLEARANCE:
        raise RuntimeError('The 15 cm example path does not meet the point-clearance check')
    return cases, (dense_free, sparse_free, wall, person)


def draw_panel(ax, result, title):
    colors = ListedColormap(('#e8ecf1', '#f2c4b1', '#9fd3a8', '#374151'))
    ax.imshow(result['cells'], origin='lower', interpolation='nearest',
              extent=(BOUNDS[0], BOUNDS[2], BOUNDS[1], BOUNDS[3]),
              cmap=colors, vmin=-.5, vmax=3.5)
    path = result['path_xy']
    if len(path):
        ax.plot(path[:, 0], path[:, 1], color='#1463d6', linewidth=2.4, zorder=5)
    ax.scatter(*result['start'], marker='o', s=55, c='#164e9c',
               edgecolors='white', linewidths=.9, zorder=6)
    ax.scatter(*result['goal'], marker='*', s=100, c='#a01f8b',
               edgecolors='white', linewidths=.7, zorder=6)
    m = result['metrics']
    status = 'ROUTE' if m['reachable'] else ('START UNSAFE' if not m['start_safe'] else 'NO ROUTE')
    ax.set_title('{}\n{} | known {:.2f} m²'.format(
        title, status, m['known_area_raster_estimate_m2']), fontsize=10)
    ax.set_xlim(BOUNDS[0], BOUNDS[2])
    ax.set_ylim(BOUNDS[1], BOUNDS[3])
    ax.set_aspect('equal')
    ax.grid(alpha=.15)


def draw_figure(cases, path):
    columns = (('open_aisle', '1.8 m gap, no person'),
               ('person_in_aisle', 'Person blocks gap'),
               ('person_cleared', 'Person clears gap'),
               ('sparse_free_open_room', 'Sparse free evidence'))
    fig, axes = plt.subplots(2, 4, figsize=(20, 9.5), sharex=True, sharey=True)
    for row, resolution in enumerate(('0.20', '0.15')):
        for col, (key, title) in enumerate(columns):
            ax = axes[row, col]
            draw_panel(ax, cases[resolution][key], '{} m: {}'.format(resolution, title))
            if col == 0:
                ax.set_ylabel('y (m)')
            if row == 1:
                ax.set_xlabel('x (m)')
    legend = [Patch(facecolor='#e8ecf1', label='unknown'),
              Patch(facecolor='#f2c4b1', label='known, unsafe'),
              Patch(facecolor='#9fd3a8', label='safe'),
              Patch(facecolor='#374151', label='occupied'),
              Line2D([0], [0], color='#1463d6', linewidth=2.4, label='planner route')]
    fig.legend(handles=legend, loc='lower center', ncol=5, frameon=False)
    fig.suptitle('Same observations, project ExplorationGrid; offline synthetic example',
                 fontsize=15, weight='bold')
    fig.tight_layout(rect=(0, .06, 1, .94), h_pad=3.)
    fig.savefig(str(path), dpi=170, bbox_inches='tight')
    plt.close(fig)


def previous_flights():
    path = ROOT / 'planner_comparison_runs/2026-10-19/comparison.json'
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('condition_problems'):
        raise RuntimeError('Prior SITL condition audit failed: {}'.format(data['condition_problems']))
    m = data['metrics']
    return dict(source=str(path),
                original_20cm=dict(sim_seconds=m['mission_sim_seconds']['deployed'],
                                   wall_seconds=m['total_wall_seconds']['deployed'],
                                   planner_p95_ms=m['compute_p95_ms']['deployed'],
                                   online_surface_quality_pct=m['surface_quality_pct']['deployed'],
                                   shelf_side_support_pct=m['shelf_side_pct']['deployed']),
                frontier_15cm=dict(sim_seconds=m['mission_sim_seconds']['frontier'],
                                   wall_seconds=m['total_wall_seconds']['frontier'],
                                   planner_p95_ms=m['compute_p95_ms']['frontier'],
                                   online_surface_quality_pct=m['surface_quality_pct']['frontier'],
                                   shelf_side_support_pct=m['shelf_side_pct']['frontier']))


def report(summary, output_dir):
    lines = [
        '# 二维探索规划格同输入对照 demo', '',
        '![合成栅格与路径](grid_resolution_demo.png)', '',
        '运行：`python3 scripts/demo_planning_grid_resolution.py`。脚本调用本项目的 '
        '`ExplorationGrid`，保存本次合成输入、图和 JSON；不启动 ROS、Gazebo 或 PX4。', '',
        '## 同一组合成观测的结果', '',
        '| 场景 | 0.20 m | 0.15 m |', '|---|---|---|',
    ]
    labels = (('open_aisle', '斜向 1.80 m 开口、行人未进入'),
              ('person_in_aisle', '行人占据开口'),
              ('person_cleared', '行人离开、当前障碍更新'),
              ('sparse_free_open_room', '无障碍、自由观测点间隔 0.16 m'))
    for key, label in labels:
        cells = []
        for resolution in ('0.20', '0.15'):
            m = summary['synthetic'][resolution][key]
            status = '可达' if m['reachable'] else ('起点不安全' if not m['start_safe'] else '无路径')
            if m['path_length_m'] is not None:
                status += '，栅格路径 {:.2f} m'.format(m['path_length_m'])
            cells.append(status)
        lines.append('| {} | {} | {} |'.format(label, *cells))
    lines.extend(['',
        '开口样本中，0.15 m 栅格路径对合成障碍点的最小净距为 '
        '{:.3f} m，高于本项目的 0.80 m 规划净距；0.20 m 在相同输入下未找到路径。'
        .format(summary['synthetic']['0.15']['open_aisle']['minimum_obstacle_point_clearance_m']),
        '稀疏自由观测样本则相反：0.15 m 的未知孔洞导致起点不安全。'
        '这些是特定栅格原点和合成几何的构造性示例，不能推出真实半仓库也有同一条窄通道，'
        '更不能作为碰撞安全或完整三维覆盖证明。', '',
        '已知面积由已知格数乘分辨率平方得到，是**栅格估计值**；不同分辨率不能把它当作同一真值面积直接比较。', '',
    ])
    real = summary['existing_half_warehouse_sitl']
    if real is not None:
        a, b = real['original_20cm'], real['frontier_15cm']
        prior_report = ROOT / 'planner_comparison_runs/2026-10-19/实验结论.md'
        report_link = os.path.relpath(str(prior_report), str(output_dir))
        lines.extend(['## 已有半仓库真实 SITL 航次，仅作背景', '',
                      '| 指标 | 原 frontier 0.20 m | 原 frontier 0.15 m |',
                      '|---|---:|---:|',
                      '| 任务仿真时间 | {:.2f} s | {:.2f} s |'.format(a['sim_seconds'], b['sim_seconds']),
                      '| 启动至结果写入墙钟 | {:.2f} s | {:.2f} s |'.format(a['wall_seconds'], b['wall_seconds']),
                      '| 规划 P95 | {:.2f} ms | {:.2f} ms |'.format(a['planner_p95_ms'], b['planner_p95_ms']),
                      '| 已观测表面质量 | {:.2f}% | {:.2f}% |'.format(a['online_surface_quality_pct'], b['online_surface_quality_pct']),
                      '| 货架侧面支持代理 | {:.2f}% | {:.2f}% |'.format(a['shelf_side_support_pct'], b['shelf_side_support_pct']),
                      '',
                      '这两轮使用同场景和对照配置，但各自的传感器与行人时序独立；'
                      '0.20 m 由宽松地图验收结束，0.15 m 由探索预算结束。'
                      '两轮都返航保存且均非全部探索或全三维完整。'
                      '详细条件与轨迹见[原实验结论]({})。'.format(report_link), ''])
    lines.extend(['## 使用边界', '',
                  '- 该 demo 的“行人”仅是加入与移除一组模拟障碍点，用于检查栅格更新；没有真实移动行人、无人机控制或动态检测。',
                  '- 规划器分辨率在节点初始化时创建；不能把该脚本当作飞行中自动切换栅格的实现。',
                  '- 若要判断真实半仓库中某条窄通道是否因 0.20 m 量化而漏走，必须记录同一时间的实测自由/障碍点及位姿做回放，并检查实际安全净距。',
                  ''])
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=None,
                        help='new directory for PNG, JSON, observations and report')
    args = parser.parse_args()
    now = datetime.now(timezone(timedelta(hours=8)))
    out = args.output_dir or ROOT / 'planner_grid_resolution_demo_runs' / (
        'grid_resolution_'+now.strftime('%Y%m%d_%H%M%S_%f'))
    out = out.resolve()
    if out.exists():
        parser.error('output directory exists; choose a new --output-dir: {}'.format(out))
    cases, arrays = run_synthetic()
    real = previous_flights()
    planner_source = BRIDGE_SCRIPTS/'exploration_grid.py'
    summary = dict(kind='offline_synthetic_same_input_planner_demo',
                   created_at=now.isoformat(),
                   planner_source=str(planner_source),
                   planner_source_sha256=hashlib.sha256(planner_source.read_bytes()).hexdigest(),
                   native_search_available=SEARCH is not None,
                   assumptions=dict(bounds_m=BOUNDS, resolutions_m=RESOLUTIONS,
                                    clearance_m=CLEARANCE, unknown_margin_m=UNKNOWN_MARGIN,
                                    gap_y_width_m=GAP_WIDTH_Y,
                                    gap_centre_y_m=GAP_CENTRE_Y,
                                    wall_x_at_y_zero_m=WALL_X_AT_ZERO,
                                    wall_slope_dx_per_dy=WALL_SLOPE,
                                    narrow_start_xy_m=NARROW_START,
                                    narrow_goal_xy_m=NARROW_GOAL,
                                    sparse_start_xy_m=SPARSE_START,
                                    sparse_goal_xy_m=SPARSE_GOAL,
                                    dense_free_sample_step_m=.05,
                                    sparse_free_sample_step_m=.16,
                                    static_wall_point_step_m=.05,
                                    simulated_person_point_radius_m=.20),
                   synthetic={r: {name: case['metrics'] for name, case in runs.items()}
                              for r, runs in cases.items()},
                   existing_half_warehouse_sitl=real)
    out.mkdir(parents=True)
    np.savez_compressed(str(out/'observations.npz'), dense_free=arrays[0],
                        sparse_free=arrays[1], static_wall=arrays[2], person=arrays[3])
    draw_figure(cases, out/'grid_resolution_demo.png')
    (out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n',
                                    encoding='utf-8')
    (out/'README.md').write_text(report(summary, out), encoding='utf-8')
    print('Demo:', out)
    for name in ('grid_resolution_demo.png', 'summary.json', 'README.md', 'observations.npz'):
        print('  ', out/name)
    print('Narrow aisle:', {r: cases[r]['open_aisle']['metrics']['reachable'] for r in cases})
    print('Sparse free:', {r: cases[r]['sparse_free_open_room']['metrics']['reachable'] for r in cases})


if __name__ == '__main__':
    main()
