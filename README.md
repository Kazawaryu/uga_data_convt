# uga_data_convt

## Export the uga data from rosbag

FYI:
```shell
https://github.com/shao-lab-uga/Vehicle_Data_-_Digital_Twin/tree/main/uga_dataset_tools
```

## Download dataset for testing

The original waymo motion dataset can be downloaded at [here](https://console.cloud.google.com/storage/browser/waymo_open_dataset_motion_v_1_1_0).
Better follow the instructions of this [repo](https://github.com/waymo-research/waymo-open-dataset.git).
The structure of the dirs should be like this, each scenario contains 20 second data.
``` 
- uncompressed/
    - occupancy_flow_challenge/
    - tf_example/
    - scenario/
        - training/ 
        - validation/ 
        - testing/ 
        - validation_interactive/ 
        - testing_interactive/ 
        - training_20s/ 
```

## Show the format of the scenario

```python
import os, glob, pickle
import tensorflow as tf
from waymo_open_dataset.protos import scenario_pb2

raw_data_path = 'training_20s/'
process_data_path = 'training_20s_process/'

raw_data = glob.glob(os.path.join(raw_data_path, '*.tfrecord*'))
raw_data.sort()

for data_file in raw_data:
    dataset = tf.data.TFRecordDataset(data_file, compression_type='')
    for cnt, data in enumerate(dataset):
        info = {}
        scenario = scenario_pb2.Scenario()
        scenario.ParseFromString(bytearray(data.numpy()))
		
		# with open('./one_scenario.txt', 'w+') as f:
        #     f.write(str(scenario))
        # f.close()
        # print("--------------------- save successfully! ---------------------")
        # quit()
        
        print(type(scenario))
        info['scenario_id'] = scenario.scenario_id
        info['timestamps_seconds'] = list(scenario.timestamps_seconds)  # list of int of shape (91)
        info['current_time_index'] = scenario.current_time_index  # int, 10
        info['sdc_track_index'] = scenario.sdc_track_index  # int
        info['objects_of_interest'] = list(scenario.objects_of_interest)  # list, could be empty list

        info['tracks_to_predict'] = {
            'track_index': [cur_pred.track_index for cur_pred in scenario.tracks_to_predict],
            'difficulty': [cur_pred.difficulty for cur_pred in scenario.tracks_to_predict]
        }  # for training: suggestion of objects to train on, for val/test: need to be predicted
        info['tracks'] = list(scenario.tracks)
        info['dynamic_map_states'] = list(scenario.dynamic_map_states)
        
        output_file = os.path.join(process_data_path, f'sample_{scenario.scenario_id}.pkl')
        with open(output_file, 'wb') as f:
            pickle.dump(info, f)

```

And for each scenario, the data format is like:
``` 
- scenario.scenario_id：the only id of the scenario
- scenario.timestamps_seconds： from 0
- scenario.current_time_index： current time token
- scenario.tracks： contain each object
    - id：id of the object
    - object_type：type
    - states：at current time stamp
        - center_x: x
        - center_y: y
        - center_z: z
        - length: l
        - width: w
        - height: h
        - heading: angle of heading
        - velocity_x: v_x
        - velocity_y: v_y
        - valid: if visible in current frame

- scenario.dynamic_map_states： traffic state
    - lane_states：contains the traffic light state and the vehicle id controlled by it
- scenario.map_features：map
    - lane centers： map
    - lane boundaries： map
    - road boundaries： map
    - crosswalks： map
    - speed bumps： position
    - stop signs： position

- scenario.sdc_track_index： ego vehicle token in the scenario
- scenario.objects_of_interest： might useful objects
- scenario.tracks_to_predict： objects in train and val set
```


## Visualization

```python
import os
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from waymo_open_dataset.protos import scenario_pb2


plt.figure(figsize=(30,30))
plt.rcParams['axes.facecolor']='grey'

data_path = '/data1/training_20s/validation'
file_list = os.listdir(data_path)

for cnt_file, file in enumerate(file_list):
    file_path = os.path.join(data_path, file)
    dataset = tf.data.TFRecordDataset(file_path, compression_type='')
    for cnt_scenar, data in enumerate(dataset):
        scenario = scenario_pb2.Scenario()
        scenario.ParseFromString(bytearray(data.numpy()))
        print(scenario.scenario_id)

        for i in range(len(scenario.map_features)):
            if str(scenario.map_features[i].lane) != '':
                line_x = [z.x for z in scenario.map_features[i].lane.polyline]
                line_y = [z.y for z in scenario.map_features[i].lane.polyline]
                plt.scatter(line_x, line_y, c='g', s=5)
            # plt.text(line_x[0], line_y[0], str(scenario.map_features[i].id), fontdict={'family': 'serif', 'size': 20, 'color': 'black'})
            
            if str(scenario.map_features[i].road_edge) != '':
                road_edge_x = [polyline.x for polyline in scenario.map_features[i].road_edge.polyline]
                road_edge_y = [polyline.y for polyline in scenario.map_features[i].road_edge.polyline]
                plt.scatter(road_edge_x, road_edge_y)
                # plt.text(road_edge_x[0], road_edge_y[0], scenario.map_features[i].road_edge.type, fontdict={'family': 'serif', 'size': 20, 'color': 'black'})
                if scenario.map_features[i].road_edge.type == 2:
                    plt.scatter(road_edge_x, road_edge_y, c='k')
                    
                elif scenario.map_features[i].road_edge.type == 3:
                    plt.scatter(road_edge_x, road_edge_y, c='purple')
                    print(scenario.map_features[i].road_edge)
                else:
                    plt.scatter(road_edge_x, road_edge_y, c='k')
            
            if str(scenario.map_features[i].road_line) != '':
                road_line_x = [j.x for j in scenario.map_features[i].road_line.polyline]
                road_line_y = [j.y for j in scenario.map_features[i].road_line.polyline]
                if scenario.map_features[i].road_line.type == 7:  
                    plt.plot(road_line_x, road_line_y, c='y')
                elif scenario.map_features[i].road_line.type == 8:  
                    plt.plot(road_line_x, road_line_y, c='y') 
                elif scenario.map_features[i].road_line.type == 6:  
                    plt.plot(road_line_x, road_line_y, c='y')
                elif scenario.map_features[i].road_line.type == 1:  
                    for i in range(int(len(road_line_x)/7)):
                        plt.plot(road_line_x[i*7:5+i*7], road_line_y[i*7:5+i*7], color='w')
                elif scenario.map_features[i].road_line.type == 2:  
                    plt.plot(road_line_x, road_line_y, c='w')
                else:
                    plt.plot(road_line_x, road_line_y, c='w')
        
        for i in range(len(scenario.tracks)):
            if i==scenario.sdc_track_index:
                traj_x = [center.center_x for center in scenario.tracks[i].states if center.center_x != 0.0]
                traj_y = [center.center_y for center in scenario.tracks[i].states if center.center_y != 0.0]
                head = [center.heading for center in scenario.tracks[i].states if center.center_y != 0.0]
                plt.scatter(traj_x[0], traj_y[0], s=140, c='r', marker='s')
                # plt.imshow(img1,extent=[traj_x[0]-3, traj_x[0]+3,traj_y[0]-1.5, traj_y[0]+1.5])
                plt.scatter(traj_x, traj_y, s=14, c='r')
            else:
                traj_x = [center.center_x for center in scenario.tracks[i].states if center.center_x != 0.0]
                traj_y = [center.center_y for center in scenario.tracks[i].states if center.center_y != 0.0]
                head = [center.heading for center in scenario.tracks[i].states if center.center_y != 0.0]
                plt.scatter(traj_x[0], traj_y[0], s=140, c='k', marker='s')
                # plt.imshow(img1,extent=[traj_x[0]-3, traj_x[0]+3,traj_y[0]-1.5, traj_y[0]+1.5])
                plt.scatter(traj_x, traj_y, s=14, c='b')
        break
    break

```

## Todo

Current goal:
Refers to the waymo dataset toolkit, convert the uga motion data to waymo format, or write a uga toolkit to make sure the algorithm can run on our own dataset.
For the algorithms, you can test if these algorithms work:
- https://github.com/YoushaaMurhij/OFMPNet
- https://github.com/OpenDriveLab/BeTop
- https://github.com/georgeliu233/STrajNet