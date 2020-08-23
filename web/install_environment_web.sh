#!/bin/bash

#############################################################################
#
#############################################################################

## Bash script that install Node.js v10.22.x and NPM v6.14.x

#############################################################################
# 				Prepare
#############################################################################
## Check if the script is running as root
if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

## Delete exisitng ROS
echo "!  Remove existing Node.js and related packages"
sudo apt-get purge nodejs
sudo apt-get autoremove

#############################################################################
# 				Node.js
#		https://www.digitalocean.com/community/tutorials/how-to-install-node-js-on-ubuntu-18-04
#############################################################################
## Install Node.js
echo "!  Install Node.js"
curl -sL https://deb.nodesource.com/setup_10.x -o nodesource_setup.sh
sudo bash nodesource_setup.sh # This script identify distro, add apt source and run apt-get update
sudo apt install nodejs

## Check instaled version
node_expected_version="v10.22.0"
node_version=`nodejs -v`
if [[ $node_version == $node_expected_version ]]; then
    echo "!  Node.js $node_version installed "
else
    echo "ERROR: Incorrect Node.js version - $node_version (expected $node_expected_version)"
fi
npm_expected_version="6.14.6"
npm_version=`npm -v`
if [[ $npm_version == $npm_expected_version ]]; then
    echo "!  NPM v$npm_version installed"
else
    echo "ERROR: Incorrect NPM version - v$npm_version (expected v$npm_expected_version)"
fi

## Clean up
echo "!  Clean up"
rm -f nodesource_setup.sh
