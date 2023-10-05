#!/bin/bash

#model=$1

for model in lav
do
  # right_of_way
  for i in 1 2 3 7 8 9 10
  do
    for (( c=1; c<=1; c++ ))
    do
      for town in 1 2 3 4 5 6 7 10
      do
  #       sh /home/yao/Documents/carla/carla_0.9.13/CarlaUE4.sh -RenderOffScreen &
         sh /opt/carla-simulator/CarlaUE4.sh &
#         sh /opt/carla-simulator/CarlaUE4.sh -RenderOffScreen &
         sleep 3
         pid=$(fuser 2000/tcp 2>/dev/null)
         sleep 5
         python scenario_parser.py -f scenario_config/right_of_way/rule$i.yaml -m $model -t $town
         sleep 5
         # Kill it
         kill -INT $pid
      done
    done
  done

  # signs
  for (( i=1; i<=6; i++ ))
  do
      for (( c=1; c<=1; c++ ))
    do
      for town in 1 2 3 4 5 6 7 10
      do
  #       sh /home/yao/Documents/carla/carla_0.9.13/CarlaUE4.sh -RenderOffScreen &
         sh /opt/carla-simulator/CarlaUE4.sh -RenderOffScreen &
         PID=$!
         sleep 5
         python scenario_parser.py -f scenario_config/signs/rule$i.yaml -m $model -t $town
         sleep 5
         # Kill it
         kill -INT $PID
       done
    done
  done

  # speed
  for (( i=1; i<=5; i++ ))
  do
      for (( c=1; c<=1; c++ ))
    do
      for town in 1 2 3 4 5 6 7 10
      do
  #      sh /home/yao/Documents/carla/carla_0.9.13/CarlaUE4.sh -RenderOffScreen &
         sh /opt/carla-simulator/CarlaUE4.sh -RenderOffScreen &

         PID=$!
         sleep 5
         python scenario_parser.py -f scenario_config/speed/rule$i.yaml -m $model -t $town
         sleep 5
         # Kill it
         kill -INT $PID
        done
    done
  done

done