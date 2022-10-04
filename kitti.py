#!/usr/bin/env python


from collections import deque
import os
from data_utils import *

from publish_utils import *
from kitti_utils import *

DATA_PATH='/home/chen/Downloads/kittidata/2011_09_26/2011_09_26_drive_0005_sync/'
TRACKING_DATA_PATH="/home/chen/Downloads/kittidata/2011_09_26/2011_09_26_drive_0005_sync/training/label_02/0000.txt"
CALIBRATION_PATH='/home/chen/Downloads/kittidata/2011_09_26'
class Object:
    def __init__(self,center):
        self.len=20
        self.locations=deque(maxlen=self.len)
        self.locations.appendleft(center)
    def update(self,center,displacement,yaw_change):
        for i in range(len(self.locations)):
            x0,y0=self.locations[i]
            x1=x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1=-x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            self.locations[i]=np.array([x1,y1])
        if center is not None:
            self.locations.appendleft(center)
    def reset(self):
        self.locations=deque(maxlen=self.len)


def compute_3d_box_cam2(h,w,l,x,y,z,yaw):
    """
    Return : 3xn in cam2 coordinate
    """
    R=np.array([[np.cos(yaw),0,np.sin(yaw)],[0,1,0],[-np.sin(yaw),0,np.cos(yaw)]])
    x_corners=[ l/2, l/2,-l/2,-l/2,
                l/2, l/2,-l/2,-l/2]
    y_corners=[0,0,0,0,-h,-h,-h,-h]
    z_corners=[ w/2,-w/2,-w/2, w/2,
                w/2,-w/2,-w/2, w/2,]
    corners_3d_cam2=np.dot(R,np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2+=np.vstack([x,y,z])
    return corners_3d_cam2



if __name__=="__main__":
    rospy.init_node('kitti_node',anonymous=True)
    # publisher
    pcl_pub=rospy.Publisher('kitti_pcl',PointCloud2,queue_size=10)
    box3d_pub=rospy.Publisher('kitti_box3d',MarkerArray,queue_size=10)
    loc_pub=rospy.Publisher('kitti_loc',MarkerArray,queue_size=10)
    # load tracking information
    df_tracking=read_tracking(TRACKING_DATA_PATH)
    calib=Calibration(CALIBRATION_PATH,from_video=True)

    frame=0
    tracker={}
    prev_imu_data=None
    while not rospy.is_shutdown():
        df_tracking_frame=df_tracking[df_tracking["frame"]==frame]
        types=np.array(df_tracking[df_tracking["frame"]==frame]['type'])
        boxes_3d=np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
        track_ids=np.array(df_tracking_frame['track_id'])
        
        
        corners_3d_velos=[]
        centers={}
        # project 3d points to velodyne coordinate system from camera coordinate system
        for track_id,box_3d in zip(track_ids,boxes_3d):
            corners_3d_cam2=compute_3d_box_cam2(*box_3d)
            corners_3d_velo=calib.project_rect_to_velo(corners_3d_cam2.T)
            corners_3d_velos+=[corners_3d_velo]
            centers[track_id]=np.mean(corners_3d_velo,axis=0)[:2]        

        # data loading
        pcl_data=read_point_cloud(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame))
        imu_data=read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))

        if prev_imu_data is None:
            for track_id in centers:
                tracker[track_id]=Object(centers[track_id])
        else:
            displacement=0.1*np.linalg.norm(imu_data[['vf','vl']])
            yaw_change=float(imu_data.yaw-prev_imu_data.yaw)
            for track_id in centers:
                if track_id in tracker:
                    # update all previous position
                    tracker[track_id].update(centers[track_id],displacement,yaw_change)
                else:
                    # generate a new tracking object
                    tracker[track_id]=Object(centers[track_id])
            for track_id in tracker:
                if track_id not in centers:
                    # update ,but don't add center to the deque
                    tracker[track_id].update(None,displacement,yaw_change)

        prev_imu_data=imu_data
        rate=rospy.Rate(10)
        # publish
        publish_pcl(pcl_pub,pcl_data)
        publish_3dbox(box3d_pub,corners_3d_velos,types,track_ids)
        publish_loc(loc_pub,tracker,centers)
        rospy.loginfo('published!')
        rate.sleep()
        frame+=1
        if frame==154:
            frame=0
            for track_id in tracker:
                tracker[track_id].reset()