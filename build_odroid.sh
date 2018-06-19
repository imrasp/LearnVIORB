
echo "Configuring and building ORB_SLAM2 ..."

mkdir /home/odroid/workspace/LearnVIORB/build
cd /home/odroid/workspace/LearnVIORB/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
echo "Building Offline SLAM"

cd /home/odroid/workspace/LearnVIORB/Examples/MAVMono/Offline
rm -r build
mkdir build
cd build
cmake ..
make -j2


echo "Building Online SLAM"

cd /home/odroid/workspace/LearnVIORB/Examples/MAVMono/Online
rm -r build
mkdir build
cd build
cmake ..
make -j2

