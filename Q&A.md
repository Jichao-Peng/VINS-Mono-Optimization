# 1 Compile
```$bash
cd line_feature_tracker/line_descriptor
rm -rf build
mkdir build
cd build
cmake ..
make

cd XX_ws
catkin_make -j8
```