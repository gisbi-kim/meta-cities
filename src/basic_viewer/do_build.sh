mkdir build 
cd build
rm -rf *
cmake ..
make 
./iterative_closest_point
cd ..