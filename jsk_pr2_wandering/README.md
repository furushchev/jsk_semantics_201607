# jsk_pr2_wandering


## Chapter4 (@wkentaro)


### Convert bagfile to dataset

```bash
rosrun jsk_pr2_wandering install_raw_data.py

roscd jsk_pr2_wandering/scripts
./bag_to_dataset.py ../raw_data/data_2016-07-14-20-23-12.bag ../raw_data/dataset_2016-07-14-20-23-12

# Visualize dataset
./scrape_dataset.py ../raw_data/dataset_2016-07-14-20-23-12
```
