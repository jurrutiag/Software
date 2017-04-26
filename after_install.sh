#!/bin/bash

#One time only
DUCKIETOWN_ROOT=$HOME/duckietown
echo export DUCKIETOWN_ROOT=$DUCKIETOWN_ROOT >> $HOME/.bashrc
echo source $DUCKIETOWN_ROOT/vehicle_name.sh >> $HOME/.bashrc
echo source $DUCKIETOWN_ROOT/environment.sh >> $HOME/.bashrc
