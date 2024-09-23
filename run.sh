rm -rf build/
mkdir build
cd build
cmake ..
make
./test2 -f ../dataset/case_5.txt