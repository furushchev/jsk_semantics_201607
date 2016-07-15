#!/usr/bin/env python

import argparse

from jsk_data import download_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'jsk_pr2_wandering'

    download_data(
        pkg_name=PKG,
        path='raw_data/data_2016-07-14-20-23-12.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vMUVSdGt2YlJQV3c',
        md5='aa3c55d18f5f26302c17a3081ca30c4c',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
