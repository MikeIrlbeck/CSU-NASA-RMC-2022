#!/bin/bash
# created by Mike Irlbeck on 4/14/2022.
roscd moony_gazebo/models

# cmd to find where to copy model to:
#	sudo find / -iname "*setup.sh" | grep gazebo
sudo cp -r Block_Competition_Arena/ /usr/share/gazebo-11/models/
