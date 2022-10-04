# kitti_tracking
数据存放地址，要到kitti.py中修改路径：
DATA_PATH='/home/chen/Downloads/kittidata/2011_09_26/2011_09_26_drive_0005_sync/'  
TRACKING_DATA_PATH="/home/chen/Downloads/kittidata/2011_09_26/2011_09_26_drive_0005_sync/training/label_02/0000.txt"  
CALIBRATION_PATH='/home/chen/Downloads/kittidata/2011_09_26'  


roscore


cd  
mkdir kitti_tracking  
cd kitti_tracking/  
mkdir src  
cd src/  
catkin_create_pkg kitti_tracking rospy
cd kitti_tracking/
mkdir scripts  
cd scripts  
git clone  
git clone git@github.com:ChenChengkai/kitti_tracking.git  
chmod +x ./*.py
cd 
cd kitti_tracking/  
catkin_make  
source devel/setup.bash  
rosrun kitti_tracking kitti.py
![image](https://github.com/ChenChengkai/kitti_tracking/blob/master/pic/rviz_display.gif)
