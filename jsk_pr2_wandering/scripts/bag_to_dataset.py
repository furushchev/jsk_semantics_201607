#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import collections
import cPickle as pickle
import math
import os
import os.path as osp
from Queue import deque

import numpy as np
from skimage.color import label2rgb
from skimage.io import imsave

import cv_bridge
from jsk_recognition_utils.color import labelcolormap
import rosbag
import rospkg
import rospy


def compute_pose_delta(pose1, pose2):
    delta_x = pose1.position.x - pose2.position.x
    delta_y = pose1.position.y - pose2.position.y
    delta_z = pose1.position.z - pose2.position.z
    delta_pos = np.linalg.norm([delta_x, delta_y, delta_z])
    delta_x = pose1.orientation.x - pose2.orientation.x
    delta_y = pose1.orientation.y - pose2.orientation.y
    delta_z = pose1.orientation.z - pose2.orientation.z
    delta_w = pose1.orientation.w - pose2.orientation.w
    delta_ori = np.linalg.norm([delta_x, delta_y, delta_z, delta_w])
    return delta_pos, delta_ori


class BagToDataset(object):

    def __init__(self, bagfile, save_dir):
        self.bag = rosbag.Bag(bagfile)
        self.save_dir = save_dir

    def convert(self, target_topic, saving_topics,
                camera_moving_related_joints):
        """Save topics with synchronization to the target_topic"""
        # Validate
        for saving_topic in saving_topics:
            assert isinstance(saving_topic, dict)
            assert isinstance(saving_topic['name'], str)
            assert 'save_type' in saving_topic
            assert 'filename' in saving_topic
            if saving_topic['save_type'] == 'colorized_label_image':
                assert 'n_labels' in saving_topic

        queue = collections.defaultdict(deque)
        queue_size = 100
        last_saved_pose = None
        topics = ['/joint_states', '/tf_to_pose/output'] + \
                 [saving_topic['name'] for saving_topic in saving_topics]
        for topic, msg, stamp in self.bag.read_messages(topics=topics):
            # Cache to queue
            if len(queue[topic]) > queue_size:
                queue[topic].popleft()
            queue[topic].append((stamp, msg))
            if topic != target_topic:
                continue
            # Save with synchronization to the target topic
            latest_msgs = {}
            for topic in topics:
                if len(queue[topic]) == 0:
                    break
                # Run approximate synchronization to the target topic
                t_delta, t, msg = sorted(
                    (stamp - t, t, m) for t, m in queue[topic])[0]
                latest_msgs[topic] = (t, msg)
                # Consider time delta
                if t_delta > rospy.Duration(0.1):
                    break
                # Consider camera moving to skip image with blur
                if topic == '/joint_states':
                    for i_joint in xrange(len(msg.name)):
                        joint_name = msg.name[i_joint]
                        joint_vel = msg.velocity[i_joint]
                        if ((joint_name in camera_moving_related_joints) and
                                (abs(joint_vel) > 1)):
                            print('WARNING: Skipping because of camera moving'
                                  '. stamp: {}, velocity: {}.'
                                  .format(stamp, abs(joint_vel)))
                            camera_is_moving = True
                            break
                    else:
                        camera_is_moving = False
                    if camera_is_moving:
                        break
                # Consider camera world pose to skip similar images
                if ((topic == '/tf_to_pose/output') and
                      (last_saved_pose is not None)):
                    delta_pos, delta_ori = compute_pose_delta(
                        last_saved_pose, msg.pose)
                    if delta_pos < 0.01 and delta_ori < 0.01:
                        print(
                            'WARNING: Skipping because camera is not moved. '
                            'stamp: {}, delta_pos: {}, delta_ori: {}.'
                            .format(stamp, delta_pos, delta_ori)
                        )
                        break
            else:
                stamp, _ = latest_msgs[target_topic]
                save_dir = osp.join(self.save_dir, str(stamp.to_nsec()))
                last_saved_pose = latest_msgs['/tf_to_pose/output'][1].pose
                # import time
                # date = time.strftime('%Y-%m-%d-%H-%M-%S',
                #                      time.gmtime(stamp.to_nsec() * 1e-9))
                print("Saving topics to '{}'".format(save_dir))
                if not osp.exists(save_dir):
                    os.makedirs(save_dir)
                for saving_topic in saving_topics:
                    _, msg = latest_msgs[saving_topic['name']]
                    if saving_topic['save_type'] == 'color_image':
                        bridge = cv_bridge.CvBridge()
                        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                        fname = osp.join(save_dir, saving_topic['filename'])
                        imsave(fname, img)
                    elif saving_topic['save_type'] == 'depth_image':
                        bridge = cv_bridge.CvBridge()
                        depth = bridge.imgmsg_to_cv2(msg)
                        fname = osp.join(save_dir, saving_topic['filename'])
                        pickle.dump(depth, open(fname, 'wb'))
                    elif saving_topic['save_type'] == 'label_image':
                        bridge = cv_bridge.CvBridge()
                        label = bridge.imgmsg_to_cv2(msg)
                        assert label.ndim == 2
                        assert label.dtype == np.int32
                        fname = osp.join(save_dir, saving_topic['filename'])
                        imsave(fname, label)
                    elif saving_topic['save_type'] == 'colorized_label_image':
                        bridge = cv_bridge.CvBridge()
                        label = bridge.imgmsg_to_cv2(msg)
                        assert label.ndim == 2
                        assert label.dtype == np.int32
                        n_labels = saving_topic['n_labels']
                        cmap = labelcolormap(N=n_labels)
                        label_viz = label2rgb(label, colors=cmap, bg_label=0)
                        fname = osp.join(save_dir, saving_topic['filename'])
                        imsave(fname, label_viz)
                    else:
                        raise ValueError

    def __del__(self):
        self.bag.close()


def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('bagfile')
    # parser.add_argument('dataset_dir')
    # args = parser.parse_args()
    # bagfile = args.bagfile
    # save_dir = args.dataset_dir

    rp = rospkg.RosPack()
    pkg_path = rp.get_path('jsk_pr2_wandering')

    bagfile = osp.join(
        pkg_path, 'raw_data/data_with_camera_pose_2016-07-19-07-27-20.bag')
    save_dir = osp.join(
        pkg_path, 'raw_data/dataset_2016-07-19-07-27-20')

    converter = BagToDataset(bagfile, save_dir)
    saving_topics = [
        {
            'name': '/kinect_head/rgb/image_raw',
            'save_type': 'color_image',
            'filename': 'image.png',
        },
        {
            'name': '/kinect_head/depth_registered/image_raw',
            'save_type': 'depth_image',
            'filename': 'depth.pkl',
        },
        {
            'name': '/kinect_head_c2/filtered_point_decomposer/label',
            'save_type': 'label_image',
            'filename': 'label.png',
        },
        {
            'name': '/kinect_head_c2/filtered_point_decomposer/label',
            'save_type': 'colorized_label_image',
            'filename': 'label_viz.png',
            'n_labels': 5,  # labels other than background (0)
        },
    ]
    target_topic = '/kinect_head_c2/filtered_point_decomposer/label'
    camera_moving_related_joints = [
        'head_tilt_joint',
        'head_pan_joint',
        'fl_caster_rotation_joint',
        'fl_caster_l_wheel_joint',
        'fl_caster_r_wheel_joint',
        'fr_caster_rotation_joint',
        'fr_caster_l_wheel_joint',
        'fr_caster_r_wheel_joint',
        'bl_caster_rotation_joint',
        'bl_caster_l_wheel_joint',
        'bl_caster_r_wheel_joint',
        'br_caster_rotation_joint',
        'br_caster_l_wheel_joint',
        'br_caster_r_wheel_joint',
        'torso_lift_joint',
        'torso_lift_motor_screw_joint',
    ]
    converter.convert(
        target_topic,
        saving_topics,
        camera_moving_related_joints,
    )


if __name__ == '__main__':
    main()
