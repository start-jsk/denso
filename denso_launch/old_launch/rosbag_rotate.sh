#!/bin/sh
set -e
BAGS=`find $HOME/.ros -name "open_controller_*.bag"`
BACKUP_SERVER_IP="aries.jsk.t.u-tokyo.ac.jp"
BACKUP_SERVER_USER="eus"
BACKUP_DIRECTORY="/home/jsk/ros/data/open_controllers"
for bag in $BAGS
do
    scp $bag $BACKUP_SERVER_USER@$BACKUP_SERVER_IP:$BACKUP_DIRECTORY
    rm $bag
done