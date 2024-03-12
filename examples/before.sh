#!/usr/bin/env bash

# to run applications without sudo privileges, connect the glasses,
# make sure that permission to device files are set to 777 (it will be reverted after next login):

sudo chmod 777 /dev/hidraw*
