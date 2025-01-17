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

USE_GUI = True

def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)
    # print(root.Leg.leg.PullingCable1.MechanicalObject.position.value[-1])
    # print(root.Leg.leg.CollisionMesh.MechanicalObject.position.value[-34])

    if not USE_GUI:
        print(root.Leg.leg.CollisionMesh.MechanicalObject.position.value[-34])        
        for i in range(10):
            root.Leg.leg.PullingCable1.CableConstraint.value = [i]
            Sofa.Simulation.animate(root, root.dt.value)
            print(root.Leg.leg.CollisionMesh.MechanicalObject.position.value[-34])
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()

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
           fixingBox=[30.0, 5.0, -30.0, -30.0, -10.0, 30.0], pullPointLocation=[0.0, 0.0, 0.0]):
    # attach_one_way = parentNode.addChild('attach_one_way')
    leg = parentNode.addChild(name)
    eobject = ElasticMaterialObject(leg,
                                    volumeMeshFileName="mesh/Robot.vtk",
                                    poissonRatio=0.3,
                                    youngModulus=18000,
                                    totalMass=0.5,
                                    surfaceColor=[0.0, 0.8, 0.7, 1.0],
                                    surfaceMeshFileName="mesh/Robot.stl",
                                    rotation=rotation,
                                    translation=translation,
                                    name="leg")
    # eobject.addChild('MechanicalObject', template="Rigid3d", name="mstate")

    leg.addChild(eobject)

    

    # FixedBox(eobject, atPositions=fixingBox, doVisualization=True)

    # cable1 = PullingCable(eobject,
    #                      "PullingCable1",
    #                      pullPointLocation=pullPointLocation,
    #                      rotation=[0.0, 0.0, -90],
    #                      translation=[0.0, 0.0, 0.0],
    #                      cableGeometry=loadPointListFromFile("mesh/string.json"));

    # eobject.addObject(FingerController1(cable1))

    # cable2 = PullingCable(eobject,
    #                      "PullingCable2",
    #                      pullPointLocation=pullPointLocation,
    #                      rotation=[120.0, 0.0, -90],
    #                      translation=[0.0, 0.0, 0.0],
    #                      cableGeometry=loadPointListFromFile("mesh/string.json"));

    # eobject.addObject(FingerController2(cable2))

    # cable3 = PullingCable(eobject,
    #                      "PullingCable3",
    #                      pullPointLocation=pullPointLocation,
    #                      rotation=[-120.0, 0.0, -90],
    #                      translation=[0.0, 0.0, 0.0],
    #                      cableGeometry=loadPointListFromFile("mesh/string.json"));

    # eobject.addObject(FingerController3(cable3))

    CollisionMesh(eobject, name="CollisionMesh",
                  surfaceMeshFileName="mesh/Robot.stl",
                  rotation=rotation, translation=translation,
                  collisionGroup=[1, 2])
    
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]

    # Creating the floor object
    floor = leg.addChild("floor")

    floor.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0.0, 0.0, -200.0], rotation2=[90., 0., 0.], showObjectScale=1.0)
    floor.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])

    #### Collision subnode for the floor
    floorCollis = floor.addChild('collision')
    floorCollis.addObject('MeshOBJLoader', name="loader", filename="mesh/floor.obj", triangulate="true", scale3d=[5.0]*3)
    floorCollis.addObject('MeshTopology', src="@loader")
    floorCollis.addObject('MechanicalObject')
    floorCollis.addObject('TriangleCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('RigidMapping')

    #### Visualization subnode for the floor
    floorVisu = floor.addChild("VisualModel")
    floorVisu.loader = floorVisu.addObject('MeshOBJLoader', name="loader", filename="mesh/floor.obj")
    floorVisu.addObject('OglModel', name="model", src="@loader", scale3d=[5.0]*3, color=[1., 1., 0.], updateNormals=False)
    floorVisu.addObject('RigidMapping')

    # sphere = leg.addChild('sphere')
    # a = sphere.addChild("Articulation")
    # a.addObject('Articulation', translation=False, rotation=False, articulationIndex=1)
    # sphere = eobject.addChild("sphere")
    # sphere.addObject('EulerImplicitSolver', name='odesolver')
    # sphere.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    # sphere.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[-175., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=1)
    # sphere.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    # sphere.addObject('UncoupledConstraintCorrection')

    # #### Collision subnode for the sphere
    # collision = sphere.addChild('collision')
    # collision.addObject('MeshOBJLoader', name="loader", filename="mesh/ball.obj", triangulate="true", scale=5.0)
    # collision.addObject('MeshTopology', src="@loader")
    # collision.addObject('MechanicalObject')
    # collision.addObject('TriangleCollisionModel')
    # collision.addObject('LineCollisionModel')
    # collision.addObject('PointCollisionModel')
    # collision.addObject('RigidMapping')

    # #### Visualization subnode for the sphere
    # sphereVisu = sphere.addChild("VisualModel")
    # sphereVisu.loader = sphereVisu.addObject('MeshOBJLoader', name="loader", filename="mesh/ball.obj")
    # sphereVisu.addObject('OglModel', name="model", src="@loader", scale=1.0, color=[0., 1., 0.], updateNormals=False)
    # sphereVisu.addObject('RigidMapping')

    # finger.addObject('AttachConstraint', object1="@leg", object2="@sphere",  indices1="144 145 146 147 148 149 150 151 152 153 154 155 156 157 158 159", indices2="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15")

    return leg


def createScene(rootNode):
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

    Leg(rootNode, translation=[0.0, 0.0, 0.0])

    return rootNode

if __name__ == '__main__':
    main()