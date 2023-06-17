import rospy
from detection_msgs.msg import BoundingBoxes
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy as ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from custom_msgs.msg import Coord
import open3d as o3d
from transforms3d.euler import mat2euler
from transforms3d.euler import euler2mat

class vision():

    def __init__(self):
        ## Rotation matrix from Camera to World frame
        self.w_R_c = np.array([[0.0, -0.49948, 0.86632],
                               [-1.0, 0.0, 0.0],
                              [-0.0, -0.86632, -0.49948]])
        ## Traslation vector from Robot to Camera frame
        self.x_c = np.array([-0.4, 0.59, 1.4])
        #self.base_offset = np.array([0.5, 0.35, 1.75])
        self.point_cloud = PointCloud2()
        self.voxel_size = 0.003
        self.models="/home/lucaboschiero/ros_ws/src/locosim/ros_impedance_controller/worlds/models"
        self.model_names=np.array(["X1-Y2-Z1", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2", "X1-Y3-Z2-FILLET", "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2", "X2-Y2-Z2-FILLET"])


    def pointcloud_callback(self, msg):
        self.point_cloud = msg

    def image_callback(self, msg):
        bridge = CvBridge()
        self.img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


    def pointCloudRegion(self, xmin, ymin, xmax, ymax):
        x_min = int(np.ceil(xmin))
        y_min = int(np.ceil(ymin))
        x_max = int(np.floor(xmax))
        y_max = int(np.floor(ymax))

        uv = [(x, y) for y in range(y_min, y_max + 1) for x in range(x_min, x_max + 1)]

        points_region = []
        for data in point_cloud2.read_points(self.point_cloud, field_names=['x', 'y', 'z'], skip_nans=False, uvs=uv):
            points_region.append([data[0], data[1], data[2]])

        return points_region

    def process_pointCloud(self, pcd):

        pcd_down = o3d.geometry.PointCloud.voxel_down_sample(pcd, self.voxel_size)

        radius_normal = self.voxel_size * 2
        o3d.geometry.PointCloud.estimate_normals(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = self.voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def registration(self, source_down, target_down, source_fpfh, target_fpfh):

        distance_threshold = self.voxel_size * 1.5

        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 500))
        
        distance_threshold = self.voxel_size * 0.4

        result1 = o3d.pipelines.registration.registration_icp(
            source_down, target_down, distance_threshold, result.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        
        return result1

    def pose_estimation(self, object_points):
        object_points = np.asarray(object_points)
        source = o3d.geometry.PointCloud()
        source.points = o3d.utility.Vector3dVector(object_points)

        # Read the mesh data from the STL file
        stl_file_path = self.models + "/model1/meshes/X1-Y2-Z1.stl"  # Update the file path accordingly
        try:
            mesh = o3d.io.read_triangle_mesh(stl_file_path)
        except Exception as e:
            print(f"Error reading STL file: {str(e)}")
            return None

        # Sample points uniformly from the mesh to create a point cloud
        n_points = int(np.size(object_points) * 1.5)

        mesh_points = mesh.sample_points_uniformly(number_of_points=n_points)
        target = o3d.geometry.PointCloud()
        target.points = mesh_points.points

        source_down, source_fpfh = self.process_pointCloud(source)
        target_down, target_fpfh = self.process_pointCloud(target)

        result = self.registration(source_down, target_down, source_fpfh, target_fpfh)

        R = result.transformation[:3, :3].copy()
        return R

    def detection_callback(self, msg):
        pubco = ros.Publisher("/coordinates", Coord, queue_size=100)
        points_list = []
        coordinate = Coord()

        for i in range(10):
            if 50 < msg.bounding_boxes[i].xmin < 2000:
                x_min = msg.bounding_boxes[i].xmin
                x_max = msg.bounding_boxes[i].xmax
                y_min = msg.bounding_boxes[i].ymin
                y_max = msg.bounding_boxes[i].ymax
                u = int((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2)
                v = int((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax) / 2)
                cl = (int)(msg.bounding_boxes[i].Class)

                for data in point_cloud2.read_points(self.point_cloud, field_names=['x', 'y', 'z'], skip_nans=False, uvs=[(u, v)]):
                    points_list.append([data[0], data[1], data[2]])
                    print("Data Optical frame: ", points_list)
                pointW = self.w_R_c.dot(points_list[0]) + self.x_c
                #print("Data World frame: ", pointW)
                coordinate.x = pointW[0]
                coordinate.y = pointW[1]
                coordinate.z = pointW[2]
                coordinate.cl = cl

                point_region = self.pointCloudRegion(x_min, y_min, x_max, y_max)

                # Transform from Camera to World coordinate system
                points = []
                for point in point_region:
                    points.append(self.w_R_c.dot(point) + self.x_c)

                # Get middle point
                point_middle = np.mean(points, axis=0)
                print(" Center : ", point_middle)

                # Apply offset to Object center
                points_traslated = []
                for point in points:
                    points_traslated.append((point - point_middle))

                # Get the rotation matrix and translation vector from the detected pointcloud and the associated .stl model
                rotation_matrix = self.pose_estimation(points_traslated)

                print("Rotation matrix: ",rotation_matrix)
                points_traslatedR = []
                for point in points_traslated:
                    points_traslatedR.append(rotation_matrix.dot(point))

                roll, pitch, yaw = mat2euler(rotation_matrix, 'rzyx')  # 'rzyx' specifies the rotation order
                coordinate.roll= roll
                coordinate.pitch = pitch
                coordinate.yaw = yaw

            pubco.publish(coordinate)




def main(p):
    global pub
    ros.init_node("listener")
    ros.Subscriber("/yolov5/detections", BoundingBoxes, p.detection_callback)
    ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback=p.image_callback, queue_size=1)

    ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, p.pointcloud_callback)
    ros.spin()


if __name__ == '__main__':

    p = vision()
    try:
        main(p)
    except ros.ROSInterruptException:
        pass