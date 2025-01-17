# Required import for python
import Sofa
from Sofa.Helper import msg_info
import numpy as np


def createScene(root_node):

    root_node.addObject("VisualGrid", nbSubdiv=10, size=1000)
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
    root.addObject('RuleBasedContactManager', responseParams="mu="+str(0.0), name='Response', response='FrictionContactConstraint')

    # Attach the leg to the ground
    attach_one_way = root.addChild('attach_one_way')

    attach_one_way.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="false", rayleighStiffness="0.1", rayleighMass="0.1")
    attach_one_way.addObject('CGLinearSolver', iterations="25", name="linear solver", tolerance="1.0e-9", threshold="1.0e-9")

    # Add ground floor
    ground = attach_one_way.addChild('ground')

    ground.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
    ground.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
    ground.addObject('MechanicalObject', template="Rigid3", dx="-250")
    ground.addObject('UniformMass', totalMass="5")
    ground.addObject('FixedConstraint')

    visu = ground.addChild('Visu')

    visu.addObject('MeshOBJLoader', name="meshLoader_17", filename="mesh/ground.obj", handleSeams="1")
    visu.addObject('OglModel', name="Visual", src="@meshLoader_17", color="gray")
    visu.addObject('RigidMapping', input="@..", output="@Visual")

    surf = ground.addChild('Surf')

    surf.addObject('MeshOBJLoader', name="loader", filename="mesh/ground.obj")
    surf.addObject('MeshTopology', src="@loader")
    surf.addObject('MechanicalObject', src="@loader")
    surf.addObject('TriangleCollisionModel', simulated="0", moving="0")
    surf.addObject('LineCollisionModel', simulated="0", moving="0")
    surf.addObject('PointCollisionModel', simulated="0", moving="0")
    surf.addObject('RigidMapping', )

    # Add the leg
    leg = attach_one_way.addChild('leg')

    leg.addObject('EulerImplicitSolver', rayleighStiffness="0.01")
    leg.addObject('CGLinearSolver', iterations="100", threshold="0.00000001", tolerance="1e-5")
    leg.addObject('MeshOBJLoader', name="loader", filename="mesh/leg.obj")
    leg.addObject('MeshTopology', src="@loader")
    leg.addObject('MechanicalObject', src="@loader", translation2=[0., 11., 250.], rotation2=[0., 0., 0.])
    leg.addObject('MeshSpringForceField', name="Springs", tetrasStiffness="4000", tetrasDamping="40")
    # leg.addObject('MechanicalObject', template="Rigid3", translation2=[0., 5.5, 0.], rotation2=[0., 0., 0.])
    leg.addObject('UniformMass', totalMass="100")

    visu2 = leg.addChild('Visu2')

    visu2.addObject('MeshOBJLoader', name="meshLoader_8", filename="mesh/leg.obj", handleSeams="1")
    visu2.addObject('OglModel', name="Visual", src="@meshLoader_8", dx="0", dy="5.5", dz="250", color="red")
    visu2.addObject('BarycentricMapping', input="@..", output="@Visual")
    # visu2.addObject('RigidMapping', input="@..", output="@Visual")

    surf2 = leg.addChild('Surf2')

    surf2.addObject('MeshOBJLoader', name="loader", filename="mesh/leg.obj")
    surf2.addObject('MeshTopology', src="@loader")
    surf2.addObject('MechanicalObject', src="@loader", translation2=[0., 5.5, 250.], rotation2=[0., 0., 0.])
    surf2.addObject('TriangleCollisionModel', )
    surf2.addObject('LineCollisionModel', )
    surf2.addObject('PointCollisionModel', )
    surf2.addObject('BarycentricMapping', )
    # surf2.addObject('RigidMapping', )

    attach_one_way.addObject('AttachConstraint', object1="@leg", object2="@ground")

# Choose in your script to activate or not the GUI
USE_GUI = True

def main():
    import SofaRuntime
    import Sofa.Gui
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")

    root = Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()

    print("End of simulation.")

if __name__ == '__main__':
    main()