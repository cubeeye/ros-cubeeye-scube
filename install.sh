#!/bin/sh

LOCAL_DIR=$PWD
CUBEEYE_SHARE_DIR=/usr/local/share/cubeeye
CUBEEYE_UDEV_RULES=$LOCAL_DIR/udev/98-CubeEye.rules

if [ ! -e $CUBEEYE_SHARE_DIR ]; then
	mkdir -p $CUBEEYE_SHARE_DIR
fi

if [ -e $CUBEEYE_SHARE_DIR ]; then
	cp -r $LOCAL_DIR/conf $CUBEEYE_SHARE_DIR
	cp -r $LOCAL_DIR/fw $CUBEEYE_SHARE_DIR
fi

if [ -f $CUBEEYE_UDEV_RULES ]; then
	cp $CUBEEYE_UDEV_RULES /etc/udev/rules.d/
fi
