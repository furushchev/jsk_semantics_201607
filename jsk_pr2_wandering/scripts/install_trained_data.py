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
        path='trained_data/fcn32s_2000_2016-07-20-22-52-51.chainermodel',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vQWV0QUxiZ3BhSUE',
        md5='1e7a815f4ba8e0747340f9861d59b06e',
        quiet=quiet,
    )

    download_data(
        pkg_name=PKG,
        path='trained_data/fcn32s_4900.chainermodel',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vSUswWDNNNnhIVUE',
        md5='a9b72901f162b1d6d4e6232a979a9625',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
