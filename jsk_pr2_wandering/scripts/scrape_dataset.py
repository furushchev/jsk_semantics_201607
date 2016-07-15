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
    args = parser.parse_args()

    dataset_dir = args.dataset_dir

    for dir_ in sorted(os.listdir(dataset_dir)):
        img_file = osp.join(dataset_dir, dir_, 'image.png')
        mask_file = osp.join(dataset_dir, dir_, 'mask.png')
        img = imread(img_file)
        mask = imread(mask_file, as_grey=True)
        label = np.zeros(mask.shape, dtype=np.int32)
        label[mask > 0.5] = 1
        labelviz = label2rgb(label, img, bg_label=0)
        print(time.strftime('%Y-%m-%d-%H-%M-%S', time.gmtime(int(dir_) * 1e-9)))
        plt.imshow(labelviz)
        plt.axis('off')
        plt.show()


if __name__ == '__main__':
    main()
