# Visual Localization Using AR Tags as Landmarks  

## Overview

This repository contains the code that does data collection and the mapping of AR Tags location
to set up the visual/landmark based locazliation. The goal was to improve the current localization
system within the Segbots which currently use Monte Carlo Localization and measure the effectiveness
by comparing to the ground truth.

## How to run

```
1. Launch Segbot
roslaunch bwi_launch segbot_v2.launch
2. Launch AR_Track_Alvar to let the Segbot to track/detect AR Tags
roslaunch ar_track_alvar pr2_indiv.launch
3. Compute to create a network/mapping of tags
rosrun ar_localization data_collection
```

## How the system works

## Effectiveness of the system
![https://github.com/JGOOSH/visual_ar_localization/blob/master/images/result.png]


## Additional Information
Link to the <a href="https://docs.google.com/a/utexas.edu/presentation/d/1mpIMkadw2VO_SGtUqd75TwJIoKmEbtQJO_xW6HXf0Lg/edit?usp=sharing">slide</a> 
