import Sofa
import Sofa.Gui
from stlib3.physics.constraints import FixedBox


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

def createScene(root_node):

   root = root_node.addChild('root', dt="0.02")

   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Algorithm")
   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Detection.Intersection")
   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry")
   root.addObject('RequiredPlugin', name="Sofa.Component.Collision.Response.Contact")
   root.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh")
   root.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative")
   root.addObject('RequiredPlugin', name="Sofa.Component.Mapping.Linear")
   root.addObject('RequiredPlugin', name="Sofa.Component.Mapping.NonLinear")
   root.addObject('RequiredPlugin', name="Sofa.Component.Mass")
   root.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward")
   root.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic")
   root.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.Spring")
   root.addObject('RequiredPlugin', name="Sofa.Component.StateContainer")
   root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Constant")
   root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Grid")
   root.addObject('RequiredPlugin', name="Sofa.Component.Visual")
   root.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D")
   root.addObject('DefaultAnimationLoop', )
   root.addObject('VisualStyle', displayFlags="showBehaviorModels showMappings")
   root.addObject('CollisionPipeline', depth="6", verbose="0", draw="0")
   root.addObject('BruteForceBroadPhase', )
   root.addObject('BVHNarrowPhase', )
   root.addObject('MinProximityIntersection', name="Proximity", alarmDistance="0.3", contactDistance="0.2")
   root.addObject('CollisionResponse', name="Response", response="PenalityContactForceField")
   root.addObject('DefaultAnimationLoop', )

   chain = root.addChild('chain')

   torus_fixed = chain.addChild('torus_fixed')

   torus_fixed.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
   torus_fixed.addObject('MeshTopology', src="@loader")
   torus_fixed.addObject('MechanicalObject', src="@loader")
   torus_fixed.addObject('TriangleCollisionModel', simulated="0", moving="0")
   torus_fixed.addObject('LineCollisionModel', simulated="0", moving="0")
   torus_fixed.addObject('PointCollisionModel', simulated="0", moving="0")
   torus_fixed.addObject('MeshOBJLoader', name="meshLoader_19", filename="mesh/torus2.obj", handleSeams="1")
   torus_fixed.addObject('OglModel', name="Visual", src="@meshLoader_19", color="gray")

   torus_fem = chain.addChild('torus_fem')

   torus_fem.addObject('EulerImplicitSolver', rayleighStiffness="0.01", rayleighMass="0.1")
   torus_fem.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
   torus_fem.addObject('MeshGmshLoader', name="loader", filename="data/mesh/Solid_Cylinder.msh")
   torus_fem.addObject('MeshTopology', src="@loader")
   torus_fem.addObject('MechanicalObject', src="@loader", dx="2.5")
   torus_fem.addObject('UniformMass', vertexMass="0.1")
   torus_fem.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="1000", poissonRatio="0.3", computeGlobalMatrix="false", method="polar")

   visu = torus_fem.addChild('Visu')

   visu.addObject('MeshOBJLoader', name="meshLoader_3", filename="data/mesh/Solid_Cylinder.obj", handleSeams="1")
   visu.addObject('OglModel', name="Visual", src="@meshLoader_3", color="red", dx="2.5")
   visu.addObject('BarycentricMapping', input="@..", output="@Visual")

   surf2 = torus_fem.addChild('Surf2')

   surf2.addObject('MeshOBJLoader', name="loader", filename="data/mesh/Solid_Cylinder.obj")
   surf2.addObject('MeshTopology', src="@loader")
   surf2.addObject('MechanicalObject', src="@loader", dx="2.5")
   surf2.addObject('TriangleCollisionModel', )
   surf2.addObject('LineCollisionModel', )
   surf2.addObject('PointCollisionModel', )
   surf2.addObject('BarycentricMapping', )

   # torus_spring = chain.addChild('torus_spring')

   # torus_spring.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
   # torus_spring.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
   # torus_spring.addObject('MeshGmshLoader', name="loader", filename="mesh/torus2_low_res.msh")
   # torus_spring.addObject('MeshTopology', src="@loader")
   # torus_spring.addObject('MechanicalObject', src="@loader", dx="5")
   # torus_spring.addObject('UniformMass', totalMass="5")
   # torus_spring.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="400", tetrasDamping="4")

   # visu = torus_spring.addChild('Visu')

   # visu.addObject('MeshOBJLoader', name="meshLoader_8", filename="mesh/torus2.obj", handleSeams="1")
   # visu.addObject('OglModel', name="Visual", src="@meshLoader_8", dx="5", color="green")
   # visu.addObject('BarycentricMapping', input="@..", output="@Visual")

   # surf2 = torus_spring.addChild('Surf2')

   # surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
   # surf2.addObject('MeshTopology', src="@loader")
   # surf2.addObject('MechanicalObject', src="@loader", dx="5")
   # surf2.addObject('TriangleCollisionModel', )
   # surf2.addObject('LineCollisionModel', )
   # surf2.addObject('PointCollisionModel', )
   # surf2.addObject('BarycentricMapping', )

#    torus_ffd = chain.addChild('torus_ffd')

#    torus_ffd.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_ffd.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_ffd.addObject('MechanicalObject', dx="7.5")
#    torus_ffd.addObject('UniformMass', totalMass="5")
#    torus_ffd.addObject('RegularGridTopology', nx="6", ny="2", nz="5", xmin="-2.5", xmax="2.5", ymin="-0.5", ymax="0.5", zmin="-2", zmax="2")
#    torus_ffd.addObject('RegularGridSpringForceField', name="Springs", stiffness="200", damping="2")

#    visu = torus_ffd.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_13", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_13", color="yellow")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_ffd.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_rigid = chain.addChild('torus_rigid')

#    torus_rigid.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_rigid.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_rigid.addObject('MechanicalObject', template="Rigid3", dx="10")
#    torus_rigid.addObject('UniformMass', filename="BehaviorModels/torus2.rigid")

#    visu = torus_rigid.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_17", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_17", color="gray")
#    visu.addObject('RigidMapping', input="@..", output="@Visual")

#    surf2 = torus_rigid.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('RigidMapping', )

#    chain_fem = root.addChild('chain_fem')

#    torus_fixed = chain_fem.addChild('TorusFixed')

#    torus_fixed.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    torus_fixed.addObject('MeshTopology', src="@loader")
#    torus_fixed.addObject('MechanicalObject', src="@loader", dz="6")
#    torus_fixed.addObject('TriangleCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('LineCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('PointCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('MeshOBJLoader', name="meshLoader_21", filename="mesh/torus2.obj", handleSeams="1")
#    torus_fixed.addObject('OglModel', name="Visual", src="@meshLoader_21", color="gray", dz="6")

#    torus_fem1 = chain_fem.addChild('torus_fem1')

#    torus_fem1.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_fem1.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_fem1.addObject('MeshGmshLoader', name="loader", filename="mesh/torus_low_res.msh")
#    torus_fem1.addObject('MeshTopology', src="@loader")
#    torus_fem1.addObject('MechanicalObject', src="@loader", dx="2.5", dz="6")
#    torus_fem1.addObject('UniformMass', totalMass="5")
#    torus_fem1.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="1000", poissonRatio="0.3", computeGlobalMatrix="false", method="polar")

#    visu = torus_fem1.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_23", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_23", color="red", dx="2.5", dz="6")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_fem1.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="2.5", dz="6")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_fem2 = chain_fem.addChild('torus_fem2')

#    torus_fem2.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_fem2.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_fem2.addObject('MeshGmshLoader', name="loader", filename="mesh/torus2_low_res.msh")
#    torus_fem2.addObject('MeshTopology', src="@loader")
#    torus_fem2.addObject('MechanicalObject', src="@loader", dx="5", dz="6")
#    torus_fem2.addObject('UniformMass', totalMass="5")
#    torus_fem2.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="1000", poissonRatio="0.3", computeGlobalMatrix="false", method="polar")

#    visu = torus_fem2.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_1", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_1", color="red", dx="5", dz="6")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_fem2.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="5", dz="6")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_fem3 = chain_fem.addChild('torus_fem3')

#    torus_fem3.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_fem3.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_fem3.addObject('MeshGmshLoader', name="loader", filename="mesh/torus_low_res.msh")
#    torus_fem3.addObject('MeshTopology', src="@loader")
#    torus_fem3.addObject('MechanicalObject', src="@loader", dx="7.5", dz="6")
#    torus_fem3.addObject('UniformMass', totalMass="5")
#    torus_fem3.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="1000", poissonRatio="0.3", computeGlobalMatrix="false", method="polar")

#    visu = torus_fem3.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_6", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_6", color="red", dx="7.5", dz="6")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_fem3.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="7.5", dz="6")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_fem4 = chain_fem.addChild('torus_fem4')

#    torus_fem4.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_fem4.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_fem4.addObject('MeshGmshLoader', name="loader", filename="mesh/torus2_low_res.msh")
#    torus_fem4.addObject('MeshTopology', src="@loader")
#    torus_fem4.addObject('MechanicalObject', src="@loader", dx="10", dz="6")
#    torus_fem4.addObject('UniformMass', totalMass="5")
#    torus_fem4.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="1000", poissonRatio="0.3", computeGlobalMatrix="false", method="polar")

#    visu = torus_fem4.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_10", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_10", color="red", dx="10", dz="6")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_fem4.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="10", dz="6")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    chain_spring = root.addChild('chain_spring')

#    torus_fixed = chain_spring.addChild('TorusFixed')

#    torus_fixed.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    torus_fixed.addObject('MeshTopology', src="@loader")
#    torus_fixed.addObject('MechanicalObject', src="@loader", dz="12")
#    torus_fixed.addObject('TriangleCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('LineCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('PointCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('MeshOBJLoader', name="meshLoader_14", filename="mesh/torus2.obj", handleSeams="1")
#    torus_fixed.addObject('OglModel', name="Visual", src="@meshLoader_14", dz="12", color="gray")

#    torus_spring1 = chain_spring.addChild('torus_spring1')

#    torus_spring1.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_spring1.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_spring1.addObject('MeshGmshLoader', name="loader", filename="mesh/torus_low_res.msh")
#    torus_spring1.addObject('MeshTopology', src="@loader")
#    torus_spring1.addObject('MechanicalObject', src="@loader", dx="2.5", dz="12")
#    torus_spring1.addObject('UniformMass', totalMass="5")
#    torus_spring1.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="400", tetrasDamping="4")

#    visu = torus_spring1.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_18", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_18", dx="2.5", dz="12", color="green")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_spring1.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="2.5", dz="12")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_spring2 = chain_spring.addChild('torus_spring2')

#    torus_spring2.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_spring2.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_spring2.addObject('MeshGmshLoader', name="loader", filename="mesh/torus2_low_res.msh")
#    torus_spring2.addObject('MeshTopology', src="@loader")
#    torus_spring2.addObject('MechanicalObject', src="@loader", dx="5", dz="12")
#    torus_spring2.addObject('UniformMass', totalMass="5")
#    torus_spring2.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="400", tetrasDamping="4")

#    visu = torus_spring2.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_22", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_22", dx="5", dz="12", color="green")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_spring2.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="5", dz="12")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_spring3 = chain_spring.addChild('torus_spring3')

#    torus_spring3.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_spring3.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_spring3.addObject('MeshGmshLoader', name="loader", filename="mesh/torus_low_res.msh")
#    torus_spring3.addObject('MeshTopology', src="@loader")
#    torus_spring3.addObject('MechanicalObject', src="@loader", dx="7.5", dz="12")
#    torus_spring3.addObject('UniformMass', totalMass="5")
#    torus_spring3.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="400", tetrasDamping="4")

#    visu = torus_spring3.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_0", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_0", dx="7.5", dz="12", color="green")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_spring3.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="7.5", dz="12")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_spring4 = chain_spring.addChild('torus_spring4')

#    torus_spring4.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_spring4.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_spring4.addObject('MeshGmshLoader', name="loader", filename="mesh/torus2_low_res.msh")
#    torus_spring4.addObject('MeshTopology', src="@loader")
#    torus_spring4.addObject('MechanicalObject', src="@loader", dx="10", dz="12")
#    torus_spring4.addObject('UniformMass', totalMass="5")
#    torus_spring4.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="400", tetrasDamping="4")

#    visu = torus_spring4.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_5", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_5", dx="10", dz="12", color="green")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_spring4.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader", dx="10", dz="12")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    chain_ffd = root.addChild('chain_ffd')

#    torus_fixed = chain_ffd.addChild('torus_fixed')

#    torus_fixed.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    torus_fixed.addObject('MeshTopology', src="@loader")
#    torus_fixed.addObject('MechanicalObject', src="@loader", dz="18")
#    torus_fixed.addObject('TriangleCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('LineCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('PointCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('MeshOBJLoader', name="meshLoader_9", filename="mesh/torus2.obj", handleSeams="1")
#    torus_fixed.addObject('OglModel', name="Visual", src="@meshLoader_9", dz="18", color="gray")

#    torus_ffd1 = chain_ffd.addChild('TorusFFD1')

#    torus_ffd1.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_ffd1.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_ffd1.addObject('MechanicalObject', dx="2.5", dz="18")
#    torus_ffd1.addObject('UniformMass', totalMass="5")
#    torus_ffd1.addObject('RegularGridTopology', nx="6", ny="2", nz="5", xmin="-2.5", xmax="2.5", ymin="-0.5", ymax="0.5", zmin="-2", zmax="2")
#    torus_ffd1.addObject('RegularGridSpringForceField', name="Springs", stiffness="200", damping="2")

#    visu = torus_ffd1.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_11", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_11", color="yellow")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_ffd1.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_ffd2 = chain_ffd.addChild('torus_ffd2')

#    torus_ffd2.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_ffd2.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_ffd2.addObject('MechanicalObject', dx="5", dz="18")
#    torus_ffd2.addObject('UniformMass', totalMass="5")
#    torus_ffd2.addObject('RegularGridTopology', nx="6", ny="5", nz="2", xmin="-2.5", xmax="2.5", ymin="-2", ymax="2", zmin="-0.5", zmax="0.5")
#    torus_ffd2.addObject('RegularGridSpringForceField', name="Springs", stiffness="200", damping="2")

#    visu = torus_ffd2.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_15", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_15", color="yellow")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_ffd2.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_ffd3 = chain_ffd.addChild('torus_ffd3')

#    torus_ffd3.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_ffd3.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_ffd3.addObject('MechanicalObject', dx="7.5", dz="18")
#    torus_ffd3.addObject('UniformMass', totalMass="5")
#    torus_ffd3.addObject('RegularGridTopology', nx="6", ny="2", nz="5", xmin="-2.5", xmax="2.5", ymin="-0.5", ymax="0.5", zmin="-2", zmax="2")
#    torus_ffd3.addObject('RegularGridSpringForceField', name="Springs", stiffness="200", damping="2")

#    visu = torus_ffd3.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_20", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_20", color="yellow")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_ffd3.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    torus_ffd4 = chain_ffd.addChild('torus_ffd4')

#    torus_ffd4.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
#    torus_ffd4.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_ffd4.addObject('MechanicalObject', dx="10", dz="18")
#    torus_ffd4.addObject('UniformMass', totalMass="5")
#    torus_ffd4.addObject('RegularGridTopology', nx="6", ny="5", nz="2", xmin="-2.5", xmax="2.5", ymin="-2", ymax="2", zmin="-0.5", zmax="0.5")
#    torus_ffd4.addObject('RegularGridSpringForceField', name="Springs", stiffness="200", damping="2")

#    visu = torus_ffd4.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_24", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_24", color="yellow")
#    visu.addObject('BarycentricMapping', input="@..", output="@Visual")

#    surf2 = torus_ffd4.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('BarycentricMapping', )

#    chain_rigid = root.addChild('ChainRigid')

#    torus_fixed = chain_rigid.addChild('torus_fixed')

#    torus_fixed.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    torus_fixed.addObject('MeshTopology', src="@loader")
#    torus_fixed.addObject('MechanicalObject', src="@loader", dz="24")
#    torus_fixed.addObject('TriangleCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('LineCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('PointCollisionModel', simulated="0", moving="0")
#    torus_fixed.addObject('MeshOBJLoader', name="meshLoader_2", filename="mesh/torus2.obj", handleSeams="1")
#    torus_fixed.addObject('OglModel', name="Visual", src="@meshLoader_2", dz="24", color="gray")

#    torus_rigid1 = chain_rigid.addChild('torus_rigid1')

#    torus_rigid1.addObject('EulerImplicitSolver', rayleighStiffness="0")
#    torus_rigid1.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_rigid1.addObject('MechanicalObject', template="Rigid3", dx="2.5", dz="24")
#    torus_rigid1.addObject('UniformMass', filename="BehaviorModels/torus.rigid")

#    visu = torus_rigid1.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_4", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_4", color="gray")
#    visu.addObject('RigidMapping', input="@..", output="@Visual")

#    surf2 = torus_rigid1.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('RigidMapping', )

#    torus_rigid2 = chain_rigid.addChild('torus_rigid2')

#    torus_rigid2.addObject('EulerImplicitSolver', rayleighStiffness="0")
#    torus_rigid2.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_rigid2.addObject('MechanicalObject', template="Rigid3", dx="5", dz="24")
#    torus_rigid2.addObject('UniformMass', filename="BehaviorModels/torus2.rigid")

#    visu = torus_rigid2.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_7", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_7", color="gray")
#    visu.addObject('RigidMapping', input="@..", output="@Visual")

#    surf2 = torus_rigid2.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('RigidMapping', )

#    torus_rigid3 = chain_rigid.addChild('torus_rigid3')

#    torus_rigid3.addObject('EulerImplicitSolver', rayleighStiffness="0")
#    torus_rigid3.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_rigid3.addObject('MechanicalObject', template="Rigid3", dx="7.5", dz="24")
#    torus_rigid3.addObject('UniformMass', filename="BehaviorModels/torus.rigid")

#    visu = torus_rigid3.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_12", filename="mesh/torus.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_12", color="gray")
#    visu.addObject('RigidMapping', input="@..", output="@Visual")

#    surf2 = torus_rigid3.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('RigidMapping', )

#    torus_rigid4 = chain_rigid.addChild('torus_rigid4')

#    torus_rigid4.addObject('EulerImplicitSolver', rayleighStiffness="0")
#    torus_rigid4.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
#    torus_rigid4.addObject('MechanicalObject', template="Rigid3", dx="10", dz="24")
#    torus_rigid4.addObject('UniformMass', filename="BehaviorModels/torus2.rigid")

#    visu = torus_rigid4.addChild('Visu')

#    visu.addObject('MeshOBJLoader', name="meshLoader_16", filename="mesh/torus2.obj", handleSeams="1")
#    visu.addObject('OglModel', name="Visual", src="@meshLoader_16", color="gray")
#    visu.addObject('RigidMapping', input="@..", output="@Visual")

#    surf2 = torus_rigid4.addChild('Surf2')

#    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/torus2_for_collision.obj")
#    surf2.addObject('MeshTopology', src="@loader")
#    surf2.addObject('MechanicalObject', src="@loader")
#    surf2.addObject('TriangleCollisionModel', )
#    surf2.addObject('LineCollisionModel', )
#    surf2.addObject('PointCollisionModel', )
#    surf2.addObject('RigidMapping', )
   
   return root

if __name__ == '__main__':
    main()