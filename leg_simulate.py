# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.physics.constraints import FixedBox
from softrobots.actuators import PullingCable
from stlib3.physics.collision import CollisionMesh
from splib3.loaders import loadPointListFromFile
import Sofa.Gui
from SofaRuntime import Timer
import random
import pandas as pd
import numpy as np

USE_GUI = False

def run_simulation(youngmodulus: float, poison: float, cable_inputs: list) -> tuple:
    '''
    Run the simulation with the given stiffness, poison ratio and cable inputs

    Args:
        - youngmodulus: float (young modulus of the material)
        - poison: float (poison ratio of the material)
        - cable_inputs: list (cable inputs for the leg)

    Returns:
        leg_output: tuple (position of the leg after simulation)

    '''
    root = Sofa.Core.Node("root")
    createScene(root, poissonRatio=poison, youngModulus=youngmodulus) 
    Sofa.Simulation.init(root)
    
    root.Leg.leg.PullingCable1.CableConstraint.value = [cable_inputs[0]]
    root.Leg.leg.PullingCable2.CableConstraint.value = [cable_inputs[1]]
    root.Leg.leg.PullingCable3.CableConstraint.value = [cable_inputs[2]]
    Sofa.Simulation.animate(root, root.dt.value)
    leg_output = root.Leg.leg.CollisionMesh.MechanicalObject.position.value[-34]

    return leg_output

class TimerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        # This is needed to avoid a conflict with the timer of runSofa
        self.use_sofa_profiler_timer = False

    def onAnimateBeginEvent(self, event):
        pass
        # print("Position")
        # print(rootNode.Finger.leg.sphere.mstate.position.value)

    def onAnimateEndEvent(self, event):
        pass

class FingerController1(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.J:
            displacement += 1.

        elif e["key"] == Key.U:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController2(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.K:
            displacement += 1.

        elif e["key"] == Key.I:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

class FingerController3(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.cable = args[0]
        self.name = "FingerController"

    def onKeypressedEvent(self, e):
        displacement = self.cable.CableConstraint.value[0]
        if e["key"] == Key.L:
            displacement += 1.

        elif e["key"] == Key.P:
            displacement -= 1.
            if displacement < 0:
                displacement = 0
        self.cable.CableConstraint.value = [displacement]

def Leg(parentNode=None, name="Leg",
           rotation=[90.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0],
           fixingBox=[30.0, -30.0, 5.0, -30.0, 30.0, -10.0], pullPointLocation=[0.0, 0.0, 0.0]
           , poissonRatio=0.3, youngModulus=18000):
    
    leg = parentNode.addChild(name)
    eobject = ElasticMaterialObject(leg,
                                    volumeMeshFileName="mesh/Solid_Cylinder_Coarse.vtk",
                                    poissonRatio=poissonRatio,
                                    youngModulus=youngModulus,
                                    totalMass=0.5,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="mesh/Solid_Cylinder.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    name="leg")

    leg.addChild(eobject)

    FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    cable1 = PullingCable(eobject,
                         "PullingCable1",
                         pullPointLocation=pullPointLocation,
                         rotation=[0.0, 90.0, -90.0],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController1(cable1))

    cable2 = PullingCable(eobject,
                         "PullingCable2",
                         pullPointLocation=pullPointLocation,
                         rotation=[120.0, 90.0, -90.0],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController2(cable2))

    cable3 = PullingCable(eobject,
                         "PullingCable3",
                         pullPointLocation=pullPointLocation,
                         rotation=[-120.0, 90.0, -90],
                         translation=[0.0, 0.0, 0.0],
                         cableGeometry=loadPointListFromFile("mesh/string.json"));

    eobject.addObject(FingerController3(cable3))

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName="mesh/Solid_Cylinder.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])

    return leg


def createScene(rootNode, poissonRatio, youngModulus):
    from stlib3.scene import MainHeader, ContactHeader
    rootNode.dt = 0.01

    rootNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Solver", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.StateContainer", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Visual", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select", printLog=False)
    rootNode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective")
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    MainHeader(rootNode, gravity=[0.0, 0.0, -981.0], plugins=["SoftRobots"])
    ContactHeader(rootNode, alarmDistance=4, contactDistance=3, frictionCoef=0.08)
    rootNode.VisualStyle.displayFlags = "showBehavior showCollisionModels"
    rootNode.addObject( TimerController() )

    Leg(rootNode, translation=[0.0, 0.0, 0.0], poissonRatio=poissonRatio, youngModulus=youngModulus)

    return rootNode
