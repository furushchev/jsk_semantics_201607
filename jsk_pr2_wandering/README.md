# jsk_pr2_wandering


## Chapter4 (@wkentaro)


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
