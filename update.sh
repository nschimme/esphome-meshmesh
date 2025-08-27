#!/bin/bash


declare -a arr=("esphome" "meshmesh" "meshmesh_direct" "network" "socket" "ota")

for item in "${arr[@]}"; do
  echo "Update: ${item}"
  rsync -ar ../esphome-pub/esphome/components/${item}/ components/${item} --exclude __pycache__ --delete
done


