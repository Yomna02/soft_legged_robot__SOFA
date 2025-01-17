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

    root.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Projective")
    root.addObject('RequiredPlugin', name="Sofa.Component.Engine.Select")
    root.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Direct")
    root.addObject('RequiredPlugin', name="Sofa.Component.LinearSolver.Iterative")
    root.addObject('RequiredPlugin', name="Sofa.Component.Mass")
    root.addObject('RequiredPlugin', name="Sofa.Component.ODESolver.Backward")
    root.addObject('RequiredPlugin', name="Sofa.Component.SolidMechanics.FEM.Elastic")
    root.addObject('RequiredPlugin', name="Sofa.Component.StateContainer")
    root.addObject('RequiredPlugin', name="Sofa.Component.Topology.Container.Grid")
    root.addObject('RequiredPlugin', name="Sofa.Component.Visual")
    root.addObject('RequiredPlugin', name="Sofa.Component.IO.Mesh")
    root.addObject('RequiredPlugin', name="Sofa.Component.Constraint.Lagrangian.Correction")
    root.addObject('RequiredPlugin', name="Sofa.GL.Component.Rendering3D")
    root.addObject('VisualStyle', displayFlags="showBehaviorModels showForceFields")
    root.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    root.addObject('DefaultAnimationLoop', )

    # attach_one_way = root.addChild('attach_one_way')

    root.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false", rayleighStiffness="0.1", rayleighMass="0.1")
    root.addObject('CGLinearSolver', iterations="25", name="linear solver", tolerance="1.0e-9", threshold="1.0e-9")

    m1 = root.addChild('M1')

    m1.addObject('MeshOBJLoader', name="loader", filename="data/mesh/Solid_Cylinder.obj")
    m1.addObject('MeshTopology', src="@loader")
    m1.addObject('MechanicalObject', showObject="1", rotation2=[0., 0., 0.])
    m1.addObject('UniformMass', vertexMass="1")
    # m1.addObject('RegularGridTopology', nx="4", ny="4", nz="10", xmin="1", xmax="4", ymin="0", ymax="3", zmin="0", zmax="9")
    # m1.addObject('FixedConstraint')
    FixedBox(m1, atPositions="0.0 -30.0 -30.0 10.0 30.0 60.0", doVisualization=True)
    m1.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3")

    surf2 = m1.addChild('Surf2')

    surf2.addObject('MeshOBJLoader', name="loader", filename="data/mesh/Solid_Cylinder.obj")
    surf2.addObject('MeshTopology', src="@loader")
    surf2.addObject('MechanicalObject', src="@loader")
    surf2.addObject('TriangleCollisionModel', )
    surf2.addObject('LineCollisionModel', )
    surf2.addObject('PointCollisionModel', )
    surf2.addObject('BarycentricMapping', )

    visu1 = m1.addChild('visu')

    visu1.addObject('MeshOBJLoader', name="meshLoader_10", filename="data/mesh/Solid_Cylinder.obj", handleSeams="1")
    visu1.addObject('OglModel', name="Visual", src="@meshLoader_10")
    visu1.addObject('BarycentricMapping', input="@..", output="@Visual")

    sphere = root.addChild("sphere")
    # sphere.addObject('MeshOBJLoader', name="loader", filename="data/mesh/ball.obj", scale=5.0)
    # sphere.addObject('MeshTopology', src="@loader")
    sphere.addObject('MechanicalObject', translation2=[-175., 0., 0.], rotation2=[0., 0., 0.], showObjectScale=1)
    sphere.addObject('UniformMass', name="mass", vertexMass=0.1)
    sphere.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3")

    #### Collision subnode for the sphere
    collision = sphere.addChild('collision')
    collision.addObject('MeshOBJLoader', name="loader", filename="data/mesh/ball.obj", scale=5.0)
    collision.addObject('MeshTopology', src="@loader")
    collision.addObject('MechanicalObject')
    collision.addObject('TriangleCollisionModel', )
    collision.addObject('LineCollisionModel', )
    collision.addObject('PointCollisionModel', )
    collision.addObject('BarycentricMapping', )

    visu = sphere.addChild('visu')

    visu.addObject('MeshOBJLoader', name="meshLoader_10", filename="data/mesh/ball.obj", handleSeams="1")
    visu.addObject('OglModel', name="Visual", src="@meshLoader_10")
    visu.addObject('BarycentricMapping', input="@..", output="@Visual")


    # m2 = attach_one_way.addChild('M2')
    # m2.addObject('MechanicalObject', )
    # m2.addObject('UniformMass', vertexMass="1")
    # m2.addObject('RegularGridTopology', nx="4", ny="4", nz="10", xmin="1", xmax="4", ymin="0", ymax="3", zmin="9", zmax="18")
    # m2.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3")

    # attach_one_way.addObject('AttachConstraint', object1="@M1", object2="@sphere", indices1="100", indices2="100")

    # attach_one_way2 = root.addChild('attach_one_way2')

    # attach_one_way2.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false")
    # attach_one_way2.addObject('EigenSimplicialLDLT', template="CompressedRowSparseMatrixMat3x3")

    # m1 = attach_one_way2.addChild('M1')

    # m1.addObject('MechanicalObject', )
    # m1.addObject('UniformMass', vertexMass="1")
    # m1.addObject('RegularGridTopology', nx="4", ny="4", nz="10", xmin="-4", xmax="-1", ymin="0", ymax="3", zmin="0", zmax="9")
    # # m1.addObject('FixedConstraint')
    # m1.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3")
    # # FixedBox(m1, atPositions="-4.1 -0.9 -0.1 4.1 3.1 0.1", doVisualization=True)

    # m2 = attach_one_way2.addChild('M2')

    # m2.addObject('MechanicalObject', )
    # m2.addObject('UniformMass', vertexMass="1")
    # m2.addObject('RegularGridTopology', nx="4", ny="4", nz="10", xmin="-4", xmax="-1", ymin="0", ymax="3", zmin="9", zmax="18")
    # m2.addObject('TetrahedronFEMForceField', name="FEM", youngModulus="4000", poissonRatio="0.3")

    # attach_one_way2.addObject('AttachConstraint', object1="@M1", object2="@M2", indices1="144 145 146 147 148 149 150 151 152 153 154 155 156 157 158 159", indices2="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15")
    
    return root 

if __name__ == '__main__':
    main()