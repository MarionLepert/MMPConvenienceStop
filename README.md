# MMPConvenienceStop
Central computer for the Mobile Manipulation Platforms running convenience stop. 

## Current functionality
Polls the convenience stop (big yellow button) and updates a Redis key with this value.

## Steps to run the code: 
Build the project in the build directory:
```
mkdir build
cd build
cmake .. && make 
```
Then run the executable in the bin folder: 
```
sudo ./master
```
