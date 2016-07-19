#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import collections
import cPickle as pickle
import os
import os.path as osp
from Queue import deque

import numpy as np
from skimage.color import label2rgb
from skimage.io import imsave

import cv_bridge
from jsk_recognition_utils.color import labelcolormap
import rosbag


class BagToDataset(object):

    def __init__(self, bagfile, save_dir):
        self.bag = rosbag.Bag(bagfile)
        self.save_dir = save_dir

    def convert(self, target_topic, saving_topics):
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
        queue_size = 10
        topics = [saving_topic['name'] for saving_topic in saving_topics]
        for topic, msg, stamp in self.bag.read_messages(topics=topics):
            # Cache to queue
            if len(queue[topic]) > queue_size:
                queue[topic].popleft()
            queue[topic].append((stamp, msg))
            if topic != target_topic:
                continue
            # Save with synchronization to the target topic
            saving_msgs = {}
            for saving_topic in saving_topics:
                if len(queue[saving_topic['name']]) == 0:
                    break
                # Run approximate synchronization to the target topic
                t_delta, t, msg = sorted(
                    (stamp - t, t, m) for t, m in queue[saving_topic['name']])[0]
                saving_msgs[saving_topic['name']] = (t, msg)
            else:
                stamp, _ = saving_msgs[target_topic]
                save_dir = osp.join(self.save_dir, str(stamp.to_nsec()))
                # import time
                # date = time.strftime('%Y-%m-%d-%H-%M-%S',
                #                      time.gmtime(stamp.to_nsec() * 1e-9))
                print("Saving topics to '{}'".format(save_dir))
                os.makedirs(save_dir)
                for saving_topic in saving_topics:
                    _, msg = saving_msgs[saving_topic['name']]
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
    parser = argparse.ArgumentParser()
    parser.add_argument('bagfile')
    parser.add_argument('dataset_dir')
    args = parser.parse_args()

    bagfile = args.bagfile
    save_dir = args.dataset_dir

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
    converter.convert(target_topic, saving_topics)


if __name__ == '__main__':
    main()
