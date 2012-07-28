#!/bin/sh

PLATFORMS="amd64 i386"

set -e

cd `dirname $0`/..

DIR=release/`date +%Y-%m-%d`
rm -rf $DIR

for PLATFORM in $PLATFORMS; do
	# amd64 build
	mkdir -p $DIR/$PLATFORM
	cd $DIR/$PLATFORM
	cmake -DCMAKE_TOOLCHAIN_FILE=etc/cmake/release-${PLATFORM}.conf ../../..
	make package
	mv netctrl*.tar.gz ../
	cd ../../..
done

echo "Build completed."
echo "The release tarballs are to be found in $DIR."
echo ""
echo "Libraries that the executables link to:"

for PLATFORM in $PLATFORMS; do
	fname=$DIR/$PLATFORM/src/ui/netctrl
	echo $fname
	ldd $fname
done
