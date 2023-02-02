#!/bin/python3
'''
    Provides different plug and play visualizers from an urdf
    can be used as script to visualize the models from the command line
    author: G. Fadini
'''
import pybullet as pyb
import numpy as np
import pinocchio
from pinocchio.visualize import MeshcatVisualizer, GepettoVisualizer
import argparse, os

def setTransparency(visualModel, alpha):
    for geometryObject in visualModel.geometryObjects:
        color = geometryObject.meshColor
        color[-1] = alpha
        geometryObject.meshColor = color

def visualizePinocchio(urdf, root_joint = None, robot_index = '', alpha = 1.0):
    viz = visualizeMeshcat(urdf, root_joint, robot_index, alpha)
    vizG = visualizeGepetto(urdf, root_joint, robot_index, alpha)
    return [viz, vizG]

def visualizeGepetto(urdf, root_joint = None, robot_index = '', alpha = 1.0):
    absPathURDF = os.path.abspath(urdf)
    example_robot_data_install = absPathURDF.split('/model')[0] # the model meshes and the urdf must be in a parent folder called urdf
    if root_joint is not None:
        model = pinocchio.buildModelFromUrdf(urdf, root_joint = root_joint)
    else:
        model = pinocchio.buildModelFromUrdf(urdf)
    visual = pinocchio.buildGeomFromUrdf(model, urdf, pinocchio.VISUAL, example_robot_data_install)
    setTransparency(visual, alpha)
    collision = pinocchio.buildGeomFromUrdf(model, urdf, pinocchio.COLLISION, example_robot_data_install)
    q0 = pinocchio.neutral(model)
#    viz = MeshcatVisualizer(model, collision, visual)
#    viz.initViewer()
#    viz.loadViewerModel(f"pinocchio{robot_index}")
#    viz.display(q0)
    viz = GepettoVisualizer(model, collision, visual)
    viz.initViewer()
    viz.loadViewerModel(f"pinocchio{robot_index}")
    viz.display(q0)
    return viz

def visualizeMeshcat(urdf, root_joint = None, robot_index = '', alpha = 1.0):
    absPathURDF = os.path.abspath(urdf)
    example_robot_data_install = absPathURDF.split('/model')[0] # the model meshes and the urdf must be in a parent folder called urdf
    if root_joint is not None:
        model = pinocchio.buildModelFromUrdf(urdf, root_joint = root_joint)
    else:
        model = pinocchio.buildModelFromUrdf(urdf)
    visual = pinocchio.buildGeomFromUrdf(model, urdf, pinocchio.VISUAL, example_robot_data_install)
    setTransparency(visual, alpha)
    collision = pinocchio.buildGeomFromUrdf(model, urdf, pinocchio.COLLISION, example_robot_data_install)
    q0 = pinocchio.neutral(model)
    viz = MeshcatVisualizer(model, collision, visual)
    viz.initViewer()
    viz.loadViewerModel(f"pinocchio{robot_index}")
    viz.display(q0)
    return viz


def visualizePyBullet(urdf):
    pyb.connect(pyb.GUI)
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_GUI, 0)  # disable the boxes from the GUI
    basePosition, baseOrientation = [0,0,0.0], [0,0,0,1]
    robotId = pyb.loadURDF(urdf, basePosition, baseOrientation)
    constraintId = pyb.createConstraint(robotId, -1, -1, -1, pyb.JOINT_FIXED, [1, 1, 1], [0, 0, 0], [0, 0, 0])
    pyb.setRealTimeSimulation(True)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Tests the robot URDF model in pybullet, pinocchio and the visualizer')
    parser.add_argument('urdf_name', metavar='N', type=str, nargs='+', help='urdf to test')

    args = parser.parse_args()

    for urdf in args.urdf_name:
        visualizePinocchio(urdf)
        visualizePyBullet(urdf)
