
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pcl2
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
import numpy as np



FRAME_ID='map'
LIFETIME=0.2

LINES =[[0,1],[1,2],[2,3],[3,0]] #lower face
LINES+=[[4,5],[5,6],[6,7],[7,4]]#upper face
LINES+=[[4,0],[5,1],[6,2],[7,3]]#connect lower face upper face
LINES+=[[4,1],[5,0]]            # front face

DETECTION_COLOR_DATA={'Car':(255,255,0),'Pedestrian':(0,226,255),'Cyclist':(141,40,255)}

def publish_pcl(pcl_pub,pcl_data):
    header=Header()
    header.stamp=rospy.Time.now()
    header.frame_id=FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header,pcl_data[:,:3]))

def publish_3dbox(box3d_pub,corners_3d_velos,object_types,track_ids):
    marker_array=MarkerArray()
    for i,corners_3d_velo in enumerate(corners_3d_velos):
        marker=Marker()
        marker.header.frame_id=FRAME_ID
        marker.header.stamp=rospy.Time.now()
        marker.id=i
        marker.action=Marker.ADD
        marker.lifetime=rospy.Duration(LIFETIME)
        marker.type=Marker.LINE_STRIP
        b,g,r=DETECTION_COLOR_DATA[object_types[i]]

        marker.color.r=r/255.0
        marker.color.g=g/255.0
        marker.color.b=b/255.0

        marker.color.a=1.0
        marker.scale.x=0.1

        marker.points=[]
        for l in LINES:
            p1=corners_3d_velo[l[0]]
            marker.points.append(Point(p1[0],p1[1],p1[2]))
            p2=corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0],p2[1],p2[2]))
        marker_array.markers.append(marker)


        # publish txt
        text_marker=Marker()
        text_marker.header.frame_id=FRAME_ID
        text_marker.header.stamp=rospy.Time.now()
        text_marker.id=i+1000
        text_marker.action=Marker.ADD
        text_marker.lifetime=rospy.Duration(LIFETIME)
        text_marker.type=Marker.TEXT_VIEW_FACING

        p4=np.mean(corners_3d_velo,axis=0)

        text_marker.pose.position.x=p4[0]
        text_marker.pose.position.y=p4[1]
        text_marker.pose.position.z=p4[2]+2

        text_marker.text=str(track_ids[i])

        text_marker.scale.x=1
        text_marker.scale.y=1
        text_marker.scale.z=1

        b,g,r=DETECTION_COLOR_DATA[object_types[i]]
        text_marker.color.r=r/255.0
        text_marker.color.g=g/255.0
        text_marker.color.b=b/255.0
        text_marker.color.a=1.0
        marker_array.markers.append(text_marker)

    box3d_pub.publish(marker_array)


def publish_loc(loc_pub,tracker,centers):
    marker_array=MarkerArray()
    for track_id in centers:
        marker=Marker()
        marker.header.frame_id=FRAME_ID
        marker.header.stamp=rospy.Time.now()
        marker.action=Marker.ADD
        marker.lifetime=rospy.Duration(LIFETIME)
        marker.type=Marker.LINE_STRIP
        marker.id=track_id
        marker.color.r=1.0
        marker.color.g=0.0
        marker.color.b=0.0
        marker.color.a=1.0
        marker.scale.x=0.2

        marker.points=[]
        for p in tracker[track_id].locations:
            marker.points.append(Point(p[0],p[1],0))
        
        marker_array.markers.append(marker)
    loc_pub.publish(marker_array)