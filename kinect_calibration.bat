cd
rem kinect_capture  -w 7 -h 5 -ir 0
rem kinect_capture  -w 7 -h 5 -ir 1

::����-s�ı�ʾֱ�Ӵ��ļ���ȡͼ����б궨�������ʱ-ir ֵ��1���ʾ��ȡ����ͼ��Ժ�������궨
rem kinect_capture -w 7 -h 5 -s 0.03 -ir 0 -dir calib/intrinsic -o camera.ymal
rem kinect_capture -w 7 -h 5 -s 0.03 -ir 1 -dir calib/intrinsic -o ir_camera.ymal

::ex 1��ʾ�����ⲿ�궨�����2���������ɫ�ͺ�������������λ��
rem kinect_capture  -w 7 -h 5 -s 0.03 -ex 1 -ip image_pairs.xml -o ex_camera.ymal
kinect_capture  -w 7 -h 5 -s 0.03 -ex 1 -ip image_pairs.txt -o ex_camera.ymal

pause