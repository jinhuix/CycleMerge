rm -rf build/
mkdir build
cd build
cmake ..
make
./test2 -f ../dataset/gen/gen_case1_io10000_gaussian.txt