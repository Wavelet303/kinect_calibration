cd

:: to capture only 
kinect_calibration -w 7 -h 5 -ir 0 -dir calib_images
::带有-s的表示直接从文件读取图像进行标定，如果此时-ir 值是1则表示读取红外图像对红外相机标定
rem kinect_calibration -w 7 -h 5 -s 0.03 -ir 0 -dir calib_images -o camera.ymal
rem kinect_calibration -w 7 -h 5 -s 0.03 -ir 1 -dir calib_images -o ir_camera.ymal

::ex 1表示进行外部标定，求得2个相机（彩色和红外相机）的相对位置
rem kinect_calibration  -w 7 -h 5 -s 0.03 -ex 1 -ip image_pairs.xml -o ex_camera.ymal
kinect_calibration  -w 7 -h 5 -s 0.03 -ex 1 -ip image_pairs.txt -o ex_camera.ymal

pause
