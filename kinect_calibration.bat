cd
rem kinect_capture  -w 7 -h 5 -ir 0
rem kinect_capture  -w 7 -h 5 -ir 1

::if havd "-s" option, the program will do calibration from files. 
::    In this case, if the value of "-ir" option ,then read infrared images to calibration
rem kinect_capture -w 7 -h 5 -s 0.03 -ir 0 -dir calib/intrinsic -o camera.ymal
rem kinect_capture -w 7 -h 5 -s 0.03 -ir 1 -dir calib/intrinsic -o ir_camera.ymal

:: the the value of "ex" option is  1,then the program will do extrinsic calibration. You should provide image_pair.txt file in this case. 
rem kinect_capture  -w 7 -h 5 -s 0.03 -ex 1 -ip image_pairs.xml -o ex_camera.ymal
kinect_capture  -w 7 -h 5 -s 0.03 -ex 1 -ip image_pairs.txt -o ex_camera.ymal

pause
