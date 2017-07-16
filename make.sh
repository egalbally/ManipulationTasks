set -e

# Build cs225a
mkdir -p build
cd build
cmake ..
make -j4
cd ..

# Download resource files
mkdir -p resources
cd resources
if [ ! -d "kuka_iiwa_graphics" ]; then
	curl -L http://cs.stanford.edu/groups/manips/teaching/cs225a/resources/kuka_iiwa_graphics.zip -o kuka_iiwa_graphics.zip
	unzip kuka_iiwa_graphics.zip
	rm kuka_iiwa_graphics.zip
fi
cd ..

cd bin
if [ -f "demo_project" ]; then
	cd resources/demo_project
	if [ ! -e "kuka_iiwa_graphics" ]; then
		ln -s ../../../resources/kuka_iiwa_graphics .
	fi
	cd ../..
fi
if [ -f "kuka_iiwa_driver" ]; then
	cd resources/kuka_iiwa_driver
	if [ ! -e "kuka_iiwa_graphics" ]; then
		ln -s ../../../resources/kuka_iiwa_graphics .
	fi
	cd ../..
fi
if [ -f "kuka_hold_pos" ]; then
	cd resources/kuka_hold_pos
	if [ ! -e "kuka_iiwa_graphics" ]; then
		ln -s ../../../resources/kuka_iiwa_graphics .
	fi
	cd ../..
fi
cd ..

# Insert helper scripts into bin directory
cd bin

# Make script
cat <<EOF > make.sh
cd ..
mkdir -p build
cd build
cmake ..
make -j4
cd ../bin
EOF
chmod +x make.sh

# Run generic controller script
cat <<EOF > run_controller.sh
if [ "\$#" -lt 4 ]; then
	cat <<EOM
This script calls ./visualizer, ./simulator, and the specified controller simultaneously.
All the arguments after the controller will be passed directly to it.

Usage: sh run.sh <controller-executable> <path-to-world.urdf> <path-to-robot.urdf> <robot-name> <extra_controller_args>
EOM
else
	trap 'kill %1; kill %2' SIGINT
	trap 'kill %1; kill %2' EXIT
	./simulator \$2 \$3 \$4 > simulator.log & ./visualizer \$2 \$3 \$4 > visualizer.log & ./"\$@"
	# ./visualizer \$2 \$3 \$4 & ./simulator \$2 \$3 \$4 & ./"\$@"
fi
EOF
chmod +x run_controller.sh

cd ..
