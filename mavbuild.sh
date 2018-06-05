echo "Building Offline "

cd Examples/MAVMono/Offline
rm -r build
mkdir build
cd build
cmake ..
make -j2
