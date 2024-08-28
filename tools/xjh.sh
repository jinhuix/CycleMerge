rm -rf ../build/
mkdir ../build
cd ../build
cmake ..
make
./xjh -f ../dataset/case_5.txt