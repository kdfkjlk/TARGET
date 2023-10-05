#!/usr/bin/env bash

# Launch the robot
source /opt/ros/noetic/setup.bash
source /home/yao/carla-ros-bridge/devel/setup.bash

echo "Launching application, please wait!"


function run_carla_autoware() {
  rule=$1
  sh /opt/carla-simulator/CarlaUE4.sh &
  sleep 10

#  sleep 10
  docker exec ba02 bash test.sh $2 &
  sleep 10
#  roslaunch carla_manual_control carla_manual_control.launch &
#  sleep 10
  python scenario_parser.py -f $rule -m autoware -r 0 -t $2 &
  PID2=$!
  sleep 10
  roslaunch carla_waypoint_publisher carla_waypoint_publisher.launch &
  sleep 90
  rosnode kill --all
  sleep 10
#  pid=$(fuser 2000/tcp 2>/dev/null)
#  echo "carla process" $pid
#  kill -INT $pid
#  kill -INT $PID2
#  sleep 20
}



#declare -A supportMaps=( ["rule1"]="3 4 5 7 10" ["rule2"]="3 4 5 7 10" ["rule3"]="1 3 4 5 10" ["rule7"]="3 5 6 7 10" ["rule8"]="3 5 6 7 10" ["rule9"]="3 5 6 10" ["rule10"]="3")
#  for i in 1 2 3 7 8 9 10
#  do
#    supportTowns=${supportMaps[rule$i]}
#    for town in ${supportTowns[@]}
#    do
#      echo "test town" $town
#
#      for (( c=1; c<=5; c++ ))
#      do
  for (( i=4; i<=4; i++ ))
  do

#      for town in 3 5 7 10
      for town in 10
      do
      echo "test town" $town
#         python scenario_parser.py -f scenario_config/right_of_way/rule$i.yaml -m $model
         run_carla_autoware generated_yaml/rule$i.yaml $town
         sleep 5
      done
    done
#
#
  # signs
#  declare -A supportMaps=( ["rule1"]="1 2 3 5 6 7 10" ["rule2"]="5 7 10" ["rule3"]="1 2 6 7" ["rule4"]="3 5 6 7 10" ["rule5"]="6" ["rule6"]="6")
#  for (( i=5; i<=6; i++ ))
#  do
#    supportTowns=${supportMaps[rule$i]}
#    for town in ${supportTowns[@]}
#    do
#      for (( c=1; c<=5; c++ ))
#      do
#  for (( i=1; i<=6; i++ ))
#  do
#    for (( c=1; c<=1; c++ ))
#    do
#      for town in 1 2 3 4 5 6 7 10
#      do
##         python scenario_parser.py -f scenario_config/right_of_way/rule$i.yaml -m $model
#         run_carla_autoware scenario_config/signs/rule$i.yaml $town
#         sleep 5
#      done
#    done
#  done

  # speed
#  declare -A supportMaps=( ["rule1"]="1 2 3 5 6 7 10" ["rule2"]="1 2 3 5 6 7 10" ["rule3"]="1 2 5 6 7 10" ["rule4"]="1 2 5 6 7 10" ["rule5"]="3")
#  for (( i=1; i<=5; i++ ))
#  do
#    supportTowns=${supportMaps[rule$i]}
#    for town in ${supportTowns[@]}
#    do
#      for (( c=1; c<=5; c++ ))
#      do
#  for (( i=1; i<=5; i++ ))
#  do
#    for (( c=1; c<=1; c++ ))
#    do
#      for town in 1 2 3 4 5 6 7 10
#      do
##         python scenario_parser.py -f scenario_config/right_of_way/rule$i.yaml -m $model
#         run_carla_autoware scenario_config/speed/rule$i.yaml $town
#         sleep 5
#      done
#    done
#  done