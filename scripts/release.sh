#!/bin/sh

set -e

cd `dirname $0`/..

DIR=release/`date +%Y-%m-%d`
rm -rf $DIR

# amd64 build
mkdir -p $DIR/amd64
cd $DIR/amd64
cmake -DCMAKE_TOOLCHAIN_FILE=etc/cmake/release-amd64.conf ../../..
make package
mv netctrl*.tar.gz ../
cd ../../..

# i386 build
# mkdir -p $DIR/i386
# cd $DIR/i386
# cmake -DCMAKE_TOOLCHAIN_FILE=etc/cmake/release-i386.conf ../../..
# make package
# mv netctrl*.tar.gz ../
# cd ../../..

