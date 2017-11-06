model = getCurrentModel()
state = model.initSystem()

Pointer_Link2 = model.getBodySet().get('Pointer_Link2')

# COM of humerus in local frame.
com_local = modeling.Vec3(0)
Pointer_Link2.getMassCenter(com_local)

# COM of humerus in Ground frame.
com_ground = modeling.Vec3(0)
model.getSimbodyEngine().transformPosition(state, Pointer_Link2, com_local, model.getGroundBody(), com_ground)

win = swing.JFrame("Results")
dLabel = swing.JLabel("Center of mass in local frame: " + str(com_local) + ", in Ground frame: " + str(com_ground))
win.getContentPane().add(dLabel)
win.pack()
win.show()
