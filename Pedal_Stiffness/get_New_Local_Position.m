import org.opensim.modeling.*

osimModel = Model ('Pedal_Stiffness-20171114.osim');
s = osimModel.initSystem();

G_Pedal_Comp = osimModel.getComponent('G_Pedal')
G_Pedal_Frame = PhysicalFrame.safeDownCast(G_Pedal_Comp)

Target_Body = osimModel.getBodySet.get('Pointer_Toe_R01')

Target_Body.findStationLocationInAnotherFrame(s, Vec3(0,0,0), G_Pedal_Frame)

