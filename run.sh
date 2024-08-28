rm -rf build/
mkdir build
cd build
cmake ..
make
./project_hw -f ../dataset/case_1.txt