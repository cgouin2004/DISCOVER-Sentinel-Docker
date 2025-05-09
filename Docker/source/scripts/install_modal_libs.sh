#!/bin/bash

# list dependencies here. Should match those provided by VOXL sdk-1.0 here:
# http://voxl-packages.modalai.com/dists/qrb5165/sdk-1.0/binary-arm64/

DEPS="
libmodal-json
libmodal-pipe
librc-math
libvoxl-cutils
voxl-mpa-tools
voxl-mavlink"


echo "installing modal libs for qrb5165 using sdk-1.0 debian repo"
PLATFORM=qrb5165
SECTION=sdk-1.0

DPKG_FILE="/etc/apt/sources.list.d/modalai.list"
LINE="deb [trusted=yes] http://voxl-packages.modalai.com/ ./dists/$PLATFORM/$SECTION/binary-arm64/"
sudo echo "${LINE}" > ${DPKG_FILE}

## make sure we have the latest package index
## only pull from voxl-packages to save time
sudo apt-get update -o Dir::Etc::sourcelist="sources.list.d/modalai.list" -o Dir::Etc::sourceparts="-" -o APT::Get::List-Cleanup="0"

## install the list of dependencies
echo "installing: $DEPS"
sudo apt install -y $DEPS

echo ""
echo "Done installing dependencies"
echo ""
exit 0
