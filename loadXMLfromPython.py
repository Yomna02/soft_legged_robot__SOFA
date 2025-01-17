import Sofa
import tempfile
import os


def createScene(root):
	# Call the above function to create the scene graph
	scene="""
			<Node name="mstate" dt="0.005" gravity="0 0 0">
                                <RequiredPlugin name="Sofa.GL.Component.Rendering2D" />
                                <RequiredPlugin name="Sofa.Component.StateContainer" />

				<Node name="child1">
					<MechanicalObject name="pose" template="Rigid3d" position="0 0 0   0 0 0 1" showObject="1"/>
				</Node>

				<Node name="child2">
					<MechanicalObject template="Rigid3d" position="1 1 1   0 0 0 1" showObject="1"/>
				</Node>
			</Node>
		"""
	tf = tempfile.NamedTemporaryFile(mode="w+t", suffix=".scn", delete=False)
	tf.write(scene)
	tf.flush()
	tf.close()
	loaded_node = Sofa.Simulation.load(tf.name)
	root.addChild(loaded_node)
	os.remove(tf.name)


def main():
	import SofaRuntime
	import Sofa.Gui

	root = Sofa.Core.Node("root")
	createScene(root)
	Sofa.Simulation.init(root)
	print(root.mstate.child1.pose.position.value)
	# Find out the supported GUIs
	print ("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
	# Launch the GUI (qt or qglviewer)
	Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
	Sofa.Gui.GUIManager.createGUI(root, __file__)
	Sofa.Gui.GUIManager.SetDimension(1080, 1080)
	# Initialization of the scene will be done here
	Sofa.Gui.GUIManager.MainLoop(root)
	Sofa.Gui.GUIManager.closeGUI()
	print("GUI was closed")
	print("Simulation is done.")


# Function used only if this script is called from a python environment
if __name__ == '__main__':
	main()
