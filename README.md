# Lab 6 Computer Vision, Grandin, Cescon
This is the github page of the lab 6 of computer vision 2021, unipd, it's a collaboration between the students Matteo Grandin and Riccardo Cescon.

## Compiling and executing
Download or clone the repository, in addition to the files in the repo, a folder named "Data" is required at the level of the CMakeLists.txt file. This can be retrieved from https://drive.google.com/file/d/1NvbI_85bMulQOzZ2JD3u8dxKdOCfHrlB/view. 
The structure of the lab folder should be:
Lab6_folder
  ├─Data
  |   ├─ objects
  |   |   ├─ obj1.png
  |   |   ├─...
  |   |   └─ obj4.png
  |   └─ video.mov    
  ├─ CMakeLists.txt
  ├─ matcher.h
  ├─ matcher.cpp
  ├─ tracker.h
  ├─ tracker.cpp
  └─ lab6_main.cpp
  
To compile, cmake is required.
To run, just execute "lab6_main" without any command line argument.

