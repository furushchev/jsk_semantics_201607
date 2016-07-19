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
        path='raw_data/data_2016-07-19-07-27-20.bag',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vMGZIUFdlbDBQRTA',
        md5='fc03d13b34010f4610a74d02fc847e5a',
        quiet=quiet,
    )


if __name__ == '__main__':
    main()
