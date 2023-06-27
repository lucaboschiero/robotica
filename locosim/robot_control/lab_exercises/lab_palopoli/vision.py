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
import time

class vision():

    def __init__(self):
        ## Matrice di rotazione da camera frame a world frame
        self.w_R_c = np.array([[0.0, -0.49948, 0.86632],
                               [-1.0, 0.0, 0.0],
                              [-0.0, -0.86632, -0.49948]])
        ## Vettore di traslazione da robot frame a camera frame
        self.x_c = np.array([-0.4, 0.59, 1.4])

        self.point_cloud = PointCloud2()
        self.voxel_size = 0.003
        self.models="/home/lucaboschiero/ros_ws/src/locosim/ros_impedance_controller/worlds/models"
        #self.model_names=np.array(["X1-Y1-Z2", "X1-Y2-Z1", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2", "X1-Y3-Z2-FILLET", "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2", "X2-Y2-Z2-FILLET"])

    def pointcloud_callback(self, msg):
        # Salva la pointcloud della camera
        self.point_cloud = msg

    def process_pointCloud(self, pointCloud):

        #La pointcloud viene sottocampionata
        pointCloud_down = o3d.geometry.PointCloud.voxel_down_sample(pointCloud, self.voxel_size)

        # Calcolare le normali per ogni punto nella point cloud ridotta 
        radius_normal = self.voxel_size * 2
        o3d.geometry.PointCloud.estimate_normals(
            pointCloud_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        # Calcola le feature FPFH sulla point cloud ridotta 
        radius_feature = self.voxel_size * 5
        pointCloud_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pointCloud_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        return pointCloud_down, pointCloud_fpfh

    def registration(self, inputPointCloud_down, stlPointCloud_down, inputPointCloud_fpfh, stlPointCloud_fpfh):
        # Esegue la registrazione tra due point cloud utilizzando Open3D


        # Esegue la registrazione RANSAC basata sulla corrispondenza delle feature
        distance_threshold = self.voxel_size * 1.5

        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            inputPointCloud_down, stlPointCloud_down, inputPointCloud_fpfh, stlPointCloud_fpfh, True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4, [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                    distance_threshold)
            ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 500))
        

        # Esegue la registrazione ICP (Iterative Closest Point) tra le due point cloud. Questo passaggio serve per ottimizzare la registrazione
        distance_threshold = self.voxel_size * 0.4

        result1 = o3d.pipelines.registration.registration_icp(
            inputPointCloud_down, stlPointCloud_down, distance_threshold, result.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        
        return result1
    
    def getModelPath(self, cl):

        # In base alla classe del blocco ritorna il path del modello stl

        if cl == "1" :
            return "/model0/meshes/X1-Y1-Z2.stl"

        elif cl == "2" :
            return "/model1/meshes/X1-Y2-Z1.stl"

        elif cl == "3" :
            return "/model2/meshes/X1-Y2-Z2.stl"
        
        elif cl == "4" :
            return "/model3/meshes/X1-Y2-Z2-CHAMFER.stl"
        
        elif cl == "5" :
            return "/model4/meshes/X1-Y2-Z2-TWINFILLET.stl"
        
        elif cl == "6" :
            return "/model5/meshes/X1-Y3-Z2.stl"
        
        elif cl == "6-FILLER" :
            return "/model6/meshes/X1-Y3-Z2-FILLET.stl"
        
        elif cl == "7" :
            return "/model7/meshes/X1-Y4-Z1.stl"
        
        elif cl == "8" :
            return "/model8/meshes/X1-Y4-Z2.stl"
        
        elif cl == "9" :
            return "/model9/meshes/X2-Y2-Z2.stl"
        
        elif cl == "9-FILLER" :
            return "/model10/meshes/X2-Y2-Z2-FILLET.stl"
        

    def pointCloudRegion(self, xmin, ymin, xmax, ymax):
        x_min = int(xmin)
        y_min = int(ymin)
        x_max = int(xmax)
        y_max = int(ymax)

        # Genera un vettore di punti 
        uv = [(x, y) for y in range(y_min, y_max + 1) for x in range(x_min, x_max + 1)]

        # Trasforma i punti in coordinate x, y e z rispetto al camera frame
        points = []
        for data in point_cloud2.read_points(self.point_cloud, field_names=['x', 'y', 'z'], skip_nans=False, uvs=uv):
            points.append([data[0], data[1], data[2]])

        return points

    def pose_estimation(self, points, object_class):
        points = np.asarray(points)
        inputPointCloud = o3d.geometry.PointCloud()
        inputPointCloud.points = o3d.utility.Vector3dVector(points)

        # Ottiene il path in base al tipo di blocco 
        model_path = self.getModelPath(object_class)
        # Legge il file .stl
        stl_file_path = self.models + model_path  # Update the file path accordingly
        try:
            print(stl_file_path)
            mesh = o3d.io.read_triangle_mesh(stl_file_path)
        except Exception as e:
            print(f"Error reading STL file: {str(e)}")
            return None

        # Crea una pointcloud a partire da una mesh tridimensionale utilizzando un campionamento uniforme dei punti sulla mesh.
        nPoints = int (np.size(points))

        mesh_points = mesh.sample_points_uniformly(number_of_points=nPoints)
        stlPointCloud = o3d.geometry.PointCloud()
        stlPointCloud.points = mesh_points.points

        # Sottodimensiona le due pointcloud e estrae le fpfh features
        inputPointCloud_down, inputPointCloud_fpfh = self.process_pointCloud(inputPointCloud)
        stlPointCloud_down, stlPointCloud_fpfh = self.process_pointCloud(stlPointCloud)

        # Esegue la registrazione delle pointcloud
        result = self.registration(inputPointCloud_down, stlPointCloud_down, inputPointCloud_fpfh, stlPointCloud_fpfh)

        # Ottiene la matrice di rotazione
        R = result.transformation[:3, :3]

        return R

    def detection_callback(self, msg):

        # Topic dove pubblica le coordinate dei blocchi e la classe
        pubco = ros.Publisher("/coordinates", Coord, queue_size=100)

        # Crea un messaggio dove salvare i dati da pubblicare
        coordinate = Coord()

        print("Detection time: ",msg.time)

        #salva il tempo di inizio
        start_time = time.time()

        # Scorre tra gli elementi trovati da Yolo
        for i in range(msg.count):
            points_list = []

            x_min = msg.bounding_boxes[i].xmin
            x_max = msg.bounding_boxes[i].xmax
            y_min = msg.bounding_boxes[i].ymin
            y_max = msg.bounding_boxes[i].ymax

            # Riceve i punti della bounding box del blocco e calcola u e v 
            u = int((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2)
            v = int((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax) / 2)
            cl = (msg.bounding_boxes[i].Class)

            # Trasnforma u e v in coordinate del camera frame
            for data in point_cloud2.read_points(self.point_cloud, field_names=['x', 'y', 'z'], skip_nans=False, uvs=[(u, v)]):
                points_list.append([data[0], data[1], data[2]])
                print("Data Optical frame: ", points_list)
            
            # Trasforma le coordinate nel world frame
            pointW = self.w_R_c.dot(points_list[0]) + self.x_c

            # Salva x, y, z e la classe del blocco per poi pubblicarle
            coordinate.x = pointW[0]
            coordinate.y = pointW[1]
            coordinate.z = pointW[2]
            coordinate.cl = cl
            
            # Ottiene i punti della pointcloud del blocco
            point_region = self.pointCloudRegion(x_min, y_min, x_max, y_max)

            # Trasforma i punti dal frame della camera al world frame
            points = []
            for point in point_region:
                points.append(self.w_R_c.dot(point) + self.x_c)

            # Ottiene il centro dei punti
            pointMiddle = np.mean(points, axis=0)

            # Applica l'offset di tutti i punti dal centro
            pointsOffset = []
            for point in points:
                pointsOffset.append((point - pointMiddle))

            # Ottiene la matrice di rotazione 
            rotation_matrix = self.pose_estimation(pointsOffset, cl)
            print("Rotation matrix: ",rotation_matrix)

            # Inserisce nel messaggio la matrice di rotazione
            coordinate.R = rotation_matrix.flatten().tolist()

            end_time = time.time()
            # Calcola il tempo trascorso in secondi
            total_time = end_time - start_time

            print("Orientation detection time:", total_time, "secondi")

            # Pubblica il messaggio
            pubco.publish(coordinate)




def main(p):

    # Vengono definiti i topic a cui è sottoscritto il codice
    ros.init_node("listener")

    ros.Subscriber("/yolov5/detections", BoundingBoxes, p.detection_callback)
    ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, p.pointcloud_callback)

    ros.spin()


if __name__ == '__main__':

    p = vision()
    try:
        main(p)
    except ros.ROSInterruptException:
        pass