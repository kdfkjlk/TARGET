#!/bin/bash

#model=$1
#3 5 6 7 10
for model in lav
do
  # right_of_way
  for (( i=1; i<=18; i++ ))
  do
      for town in 1 2
      do
#         sh /home/yao/Documents/carla/carla_0.9.13/CarlaUE4.sh -RenderOffScreen &
         sh /opt/carla-simulator/CarlaUE4.sh &
#         sh /opt/carla-simulator/CarlaUE4.sh -RenderOffScreen &
         sleep 3
         pid=$(fuser 2000/tcp 2>/dev/null)
         sleep 5
         python recording_collect.py -f generated_yaml/rule$i.yaml -m $model -t $town

         sleep 5
         # Kill it
         kill -INT $pid

      done
  done

done