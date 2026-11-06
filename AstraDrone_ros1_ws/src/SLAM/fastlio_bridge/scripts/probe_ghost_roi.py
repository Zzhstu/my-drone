#!/usr/bin/env python3
"""Read-only live snapshot of a labelled pedestrian ROI; never publishes ROS data."""
import argparse
import json
from pathlib import Path

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix')
    parser.add_argument('--x', type=float, default=-8.)
    parser.add_argument('--y', type=float, default=-4.)
    parser.add_argument('--radius', type=float, default=.4)
    args = parser.parse_args()
    prefix = Path(args.prefix)
    alignment = json.loads(Path(str(prefix) + '_alignment.json').read_text())
    rotation = np.asarray(alignment['map_from_world_rotation'])
    translation = np.asarray(alignment['map_from_world_translation'])
    rospy.init_node('ghost_roi_evaluator', anonymous=True)

    def count(topic):
        message = rospy.wait_for_message(topic, PointCloud2, timeout=10.)
        points = np.asarray(list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=float).reshape((-1, 3))
        world = (points - translation) @ rotation
        near = (np.linalg.norm(world[:, :2] - [args.x, args.y], axis=1) < args.radius) & \
               (world[:, 2] > .3) & (world[:, 2] < 1.9)
        return int(near.sum()), message.header.stamp.to_sec()

    map_count, stamp = count('/uav1/static_map')
    suspect_count, suspect_stamp = count('/uav1/map/ghost_suspects')
    states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5.)
    person = states.pose[states.name.index('0')].position
    mission = json.loads(rospy.wait_for_message('/uav1/exploration/status', String,
                                                timeout=5.).data)
    row = dict(map_sim_stamp=stamp, suspect_sim_stamp=suspect_stamp,
               world_roi_xy=[args.x, args.y], radius_m=args.radius,
               height_m=[.3, 1.9], map_points_in_roi=map_count,
               suspect_points_in_roi=suspect_count,
               pedestrian_world_xyz=[person.x, person.y, person.z],
               mission_phase=mission.get('mission_phase'),
               mission_sim_seconds=mission.get('mission_sim_seconds'),
               ghost_status=mission.get('ghost_revisit', {}).get('map_status'))
    with Path(str(prefix) + '_ghost_roi_samples.jsonl').open('a') as stream:
        stream.write(json.dumps(row) + '\n')
    print(json.dumps(row, indent=2))


if __name__ == '__main__':
    main()
