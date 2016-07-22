# jsk_pr2_wandering


## Chapter4 (@wkentaro)


### Log files

https://drive.google.com/drive/u/1/folders/0B9P1L--7Wd2va1k0dHBSbnNWVEE


### Convert bagfile to dataset

```bash
$ rosrun jsk_pr2_wandering install_raw_data.py

$ roslaunch jsk_pr2_wandering bag_to_dataset.launch

# Visualize dataset
$ roscd jsk_pr2_wandering/scripts
$ ./scrape_dataset.py ../raw_data/dataset_2016-07-19-07-27-20
Total number of images: 89
Number of images for each class:
  - background: 89 (100.000000%)
  - room73b2-hitachi-fiesta-refrigerator: 23 (25.842697%)
  - room73b2-karimoku-table: 25 (28.089888%)
  - room73b2-hrp2-parts-drawer: 12 (13.483146%)
  - room73b2-door-left: 14 (15.730337%)
  - room73b2-door-right: 1 (1.123596%)
```

## Experiments

### pr2

- setup

```bash
# pr1012
$ robot start
$ roslaunch jsk_pr2_startup pr2_minimum.launch # only in my fork
$ roslaunch cmd_vel_smoother cmd_vel_smoother.launch # only in my fork
$ roslaunch jsk_pr2_wandering pr2_localize_object_with_fcn.launch ON_C2:=true rviz:=false
```

```bash
# anaconda
$ rossetmaster pr1012
$ roslaunch jsk_pr2_wandering pr2_localize_object_with_fcn.launch ON_C2:=false rviz:=false
```

- run
- 
```bash
# pr1012
$ roslaunch jsk_pr2_wandering record_data_for_demo.launch # starts demo
```
