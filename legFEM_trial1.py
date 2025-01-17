import Sofa
import Sofa.Gui


def main():
    # Call the SOFA function to create the root node
    root = Sofa.Core.Node("root")

    # Call the createScene function, as runSofa does
    createScene(root)

    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Launch the GUI (qt or qglviewer)
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 800)

    # Initialization of the scene will be done here
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()

    # Run the simulation for 10 steps
    for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)

    # Print the position of the falling sphere
    # print(root.sphere.mstate.position.value)

    # Increase the gravity
    root.gravity.value = [0, 0, 0]

    # # Run the simulation for 10 steps MORE
    # for iteration in range(10):
    #         Sofa.Simulation.animate(root, root.dt.value)

    # Print the position of the falling sphere
    # print(root.sphere.mstate.position.value)
    # print("hi")


def createScene(rootNode):

    rootNode.addObject("VisualGrid", nbSubdiv=10, size=1000)

    # Define the root node properties
    rootNode.gravity=[0.0,-9.81,0.0]
    rootNode.dt=0.01

    # Loading all required SOFA modules
    confignode = rootNode.addChild("Config")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.AnimationLoop", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Solver", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Mass", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.StateContainer", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Visual", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic", printLog=False)
    confignode.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D", printLog=False)
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")


    # Collision pipeline
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="1000")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(0.0), name='Response', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', alarmDistance=10, contactDistance=5, angleCone=0.01)


    totalMass = 1.0
    volume = 1.0
    inertiaMatrix=[1., 0., 0., 0., 1., 0., 0., 0., 1.]


    sphere = rootNode.addChild("sphere")
    sphere.addObject('EulerImplicitSolver', name='odesolver')
    sphere.addObject('CGLinearSolver', name='Solver', iterations=25, tolerance=1e-05, threshold=1e-05)
    sphere.addObject('MeshOBJLoader', name="loader1", filename="data/mesh/TetraCylinder.obj")
    sphere.addObject('MeshTopology', src="@loader1")
    sphere.addObject('MechanicalObject', name="mstate", translation2=[0., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=1)
    sphere.addObject('UniformMass', name="mass", vertexMass=[totalMass, volume, inertiaMatrix[:]])
    sphere.addObject('UncoupledConstraintCorrection')
    sphere.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="1000", poissonRatio="0.3", computeGlobalMatrix="false", method="polar")

    #### Collision subnode for the sphere
    collision = sphere.addChild('collision')
    collision.addObject('MeshOBJLoader', name="loader", filename="data/mesh/TetraCylinder.obj", triangulate="true", scale=1.0)
    collision.addObject('MeshTopology', src="@loader")
    collision.addObject('MechanicalObject')
    collision.addObject('TriangleCollisionModel', )
    collision.addObject('LineCollisionModel', )
    collision.addObject('PointCollisionModel', )
    collision.addObject('BarycentricMapping', )

    #### Visualization subnode for the sphere
    sphereVisu = sphere.addChild("VisualModel")
    sphereVisu.addObject('MeshOBJLoader', name="meshLoader_10", filename="data/mesh/TetraCylinder.obj", handleSeams="1")
    sphereVisu.addObject('OglModel', name="Visual", src="@meshLoader_10")
    sphereVisu.addObject('BarycentricMapping', input="@..", output="@Visual")
    # sphereVisu.addObject('RigidMapping')


    # Creating the floor object
    floor = rootNode.addChild("floor")

    floor.addObject('MechanicalObject', name="mstate", template="Rigid3", translation2=[0.0, -10.0, 0.0], rotation2=[0., 0., 0.], showObjectScale=1.0)
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


    return rootNode


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()