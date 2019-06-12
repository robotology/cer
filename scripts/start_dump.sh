if pgrep "yarpdatadumper" ; then
	echo "already running"
else
	echo "starting dump"
	yarpdatadumper --type image --downsample 3 --connect /depthCamera/rgbImage:o --jpeg &
fi
