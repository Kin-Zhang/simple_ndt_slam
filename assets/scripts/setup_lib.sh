
# ros package
apt install -y ros-$ROS_DISTRO-tf2-geometry-msgs
apt install -y libomp-dev

# install glog
mkdir -p /workspace/lib
cd /workspace/lib
git clone https://github.com/google/glog.git
cd glog
git fetch --all --tags
git checkout tags/v0.4.0 -b v0.4.0
mkdir build && cd build
cmake .. && make
make install

git clone https://github.com/gflags/gflags.git
cd gflags
mkdir build && cd build
cmake .. && make
make install
cd ~
rm -rf /workspace/lib