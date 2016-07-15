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
import time

from skimage.io import imsave

import cv_bridge
import rosbag


class BagToDataset(object):

    def __init__(self, bagfile, save_dir):
        self.bag = rosbag.Bag(bagfile)
        self.save_dir = save_dir

    def convert(self, target_topic, topics):
        """Save topics with synchronization to the target_topic"""
        # Validate
        for topic_name, saving_info in topics.items():
            assert isinstance(topic_name, str)
            assert isinstance(saving_info, dict)
            assert 'save_type' in saving_info
            assert 'filename' in saving_info

        queue = collections.defaultdict(deque)
        queue_size = 10
        for topic, msg, stamp in self.bag.read_messages(topics=topics.keys()):
            # Cache to queue
            if len(queue[topic]) > queue_size:
                queue[topic].popleft()
            queue[topic].append((stamp, msg))
            if topic != target_topic:
                continue
            # Save with synchronization to the target topic
            saving_msgs = {}
            for saving_topic in topics:
                if len(queue[saving_topic]) == 0:
                    break
                # Run approximate synchronization to the target topic
                t_delta, t, msg = sorted(
                    (stamp - t, t, m) for t, m in queue[saving_topic])[0]
                saving_msgs[saving_topic] = (t, msg)
            else:
                stamp, _ = saving_msgs[target_topic]
                save_dir = osp.join(self.save_dir, str(stamp.to_nsec()))
                date = time.strftime('%Y-%m-%d-%H-%M-%S',
                                     time.gmtime(stamp.to_nsec() * 1e-9))
                print("Saving topics to '{}'".format(save_dir))
                os.makedirs(save_dir)
                for topic_name, saving_info in topics.items():
                    _, msg = saving_msgs[topic_name]
                    if saving_info['save_type'] == 'color_image':
                        bridge = cv_bridge.CvBridge()
                        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
                        fname = osp.join(save_dir, saving_info['filename'])
                        imsave(fname, img)
                    elif saving_info['save_type'] == 'depth_image':
                        bridge = cv_bridge.CvBridge()
                        depth = bridge.imgmsg_to_cv2(msg)
                        fname = osp.join(save_dir, saving_info['filename'])
                        pickle.dump(depth, open(fname, 'wb'))
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
    save_dir = args.save_dir

    converter = BagToDataset(bagfile, save_dir)
    topics = {
        '/kinect_head/rgb/image_raw': {'save_type': 'color_image',
                                       'filename': 'image.png'},
        '/kinect_head/depth_registered/image_raw': {'save_type': 'depth_image',
                                                    'filename': 'depth.pkl'},
        '/kinect_head_c2/rgb/mask_fridge': {'save_type': 'color_image',
                                            'filename': 'mask.png'},
    }
    target_topic = '/kinect_head_c2/rgb/mask_fridge'
    converter.convert(target_topic, topics)


if __name__ == '__main__':
    main()
