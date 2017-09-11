# TODO
- Need to streamline installation of robotics library
- Package rl library inside external instead of installing system-wide

## Install rl dependencies
- sudo apt-get install libcoin80-dev libcgal-dev libxml2-dev

## Download rl and compile source:
- https://www.roboticslibrary.org/download
- cmake .. -DBUILD_RL_PLAN=OFF -DBUILD_DEMOS=OFF -DBUILD_TESTS=OFF
- make
- sudo make install
