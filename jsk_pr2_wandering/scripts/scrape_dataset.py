#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import os
import os.path as osp
import time

import numpy as np
from skimage.io import imread
from skimage.color import label2rgb
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset_dir')
    parser.add_argument('--no-show', dest='show', action='store_false')
    args = parser.parse_args()

    dataset_dir = args.dataset_dir
    show = args.show

    target_names = [
        'background',
        'room73b2-hitachi-fiesta-refrigerator',
        'room73b2-karimoku-table',
        'room73b2-hrp2-parts-drawer',
        'room73b2-door-left',
        'room73b2-door-right',
    ]

    stat = {
        'total_num': 0,
        'class_num': {cls: 0 for cls in target_names},
    }
    for dir_ in sorted(os.listdir(dataset_dir)):
        img_file = osp.join(dataset_dir, dir_, 'image.png')
        label_file = osp.join(dataset_dir, dir_, 'label.png')
        img = imread(img_file)
        label = imread(label_file, as_grey=True)
        labelviz = label2rgb(label, img, bg_label=0)

        stat['total_num'] += 1
        unique_labels = np.unique(label)
        for label_val in unique_labels:
            stat['class_num'][target_names[label_val]] += 1

        if show:
            print(time.strftime('%Y-%m-%d-%H-%M-%S',
                                time.gmtime(int(dir_) * 1e-9)))
            plt.imshow(labelviz)
            plt.axis('off')
            plt.show()

    print('Total number of images:', stat['total_num'])
    print('Number of images for each class:')
    for cls in target_names:
        num = stat['class_num'][cls]
        print('  - {}: {} ({:%})'.format(cls, num, num / stat['total_num']))


if __name__ == '__main__':
    main()
