/* --------------------------------------------------------------------------*
* Title: Active Compliance Control of the Driver leg Model
* 
* -------------------------------------------------------------------------- *
*/

#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "Simbody.h"
#include "C:\OpenSim 3.3\sdk\include\OpenSim\Simulation\Control\Controller.h"


using namespace OpenSim;
using namespace SimTK;

// Computes the norm of the force-torque vector
double vecNorm(Vector value) {
	double norm;
	double sum = square(value(0)) + square(value(1)) + square(value(2)) + square(value(3)) + square(value(4)) + square(value(5));
	return norm = sqrt(sum);
}

// Computes the position of the pedal in the ground frame based on the inputted pedal Angle
Vec3 pedalPositioninG(State& s, std::string pedalName, double pedalAng, Model& model) {
	Vec3 location;
	Vec3 station(0, 0, 0); // The center of mass of the Pointer_link2

	CoordinateSet& modelSet = model.updCoordinateSet();

	const Coordinate& GP = model.getCoordinateSet().get("G_Pedal_tilt");
	const Coordinate& BP = model.getCoordinateSet().get("B_Pedal_tilt");

//	double GPVal = GP.getValue(s);
//	double BPVal = BP.getValue(s);

	if (pedalName == "GasPedal") {
		GP.setValue(s, pedalAng * Pi / 180);
		const MobilizedBody& endlink = model.getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(15));
		location = endlink.findStationLocationInGround(s, station);
//		GP.setValue(s, GPVal);
	}
	else
	{
		BP.setValue(s, pedalAng * Pi / 180);
		const MobilizedBody& endlink = model.getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(16));
		location = endlink.findStationLocationInGround(s, station);
//		BP.setValue(s, BPVal);
	}

	return location;
}

// This class defines a custom controller class for computing the joint torques for the model
class PDController2 : public Controller
{
	OpenSim_DECLARE_CONCRETE_OBJECT(PDController2, Controller);

private:
	double kp;
	double kv;
	Vector cf;
	Vec3 desPos;

public:

	// Constructor
	PDController2::PDController2() : Controller() {
		setKp(200);
		setKv(100);
		Vector con(6, Real(0));
		Vec3 pos(0);
		setdesPosition(pos);
		setContactForces(con);
	}

	void PDController2::setKp(double kp_value) {
		kp = kp_value;
	}

	void PDController2::setKv(double kv_value) {
		kv = kv_value;
	}

	// Computes the velocity vector 
	Vector PDController2::VelocityVector(const State& s) const {

		Vector c;

		_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
		_model->getMatterSubsystem().calcResidualForceIgnoringConstraints(s, SimTK::Vector(0), SimTK::Vector_<SimTK::SpatialVec>(0), SimTK::Vector(0), c);

		return c;
	}

	// Computes the gravity vector
	Vector PDController2::GravityVector(const State& s) const {
		Vector g;

		_model->getMatterSubsystem().multiplyBySystemJacobianTranspose(s, _model->getGravityForce().getBodyForces(s), g);

		return g;
	}

	Vector PDController2::G61(const State& s) const {
		Vector g61(6, Real(0));
		Vector g81 = GravityVector(s);

		for (unsigned int iJ = 0; iJ < 6; iJ++) {
			g61[iJ] = g81[iJ+2];
		};

		return g61;
	}

	Matrix PDController2::MassMatrix(const State& s) const {
		int numMobilities = s.getNU();
		Matrix M(numMobilities, numMobilities);

		_model->getMatterSubsystem().calcM(s, M);

		return M;

		//		std::cout << "M: " << M << std::endl;
	}

	/*	Array<double> PDController2::ForceValue(const State& s) const {

	_model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
	ForceSet forces = _model->getForceSet();
	//	std::cout << "forceset: " << forces << std::endl;
	OpenSim::Force& contact = forces.get(3);

	//	std::cout << "contactforce: " << contact << std::endl;
	Array<double> F;

	//	Array<std::string> labels = contact.getRecordLabels();
	F = contact.getRecordValues(s);

	//	std::cout << "force labels: " << F << std::endl;
	return F;
	} */

	Vec3 PDController2::locationOfStationinG(const State& s, int i) const {
		Vec3 location;
		Vec3 station(0, 0, 0); // The center of mass of the Pointer_link2
		const MobilizedBody& endlink = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(i));
		return location = endlink.findStationLocationInGround(s, station);
	}

	Vec3 PDController2::velocityOfStationinG(const State& s) const {
		Vec3 velocity;
		Vec3 station(0, 0, 0); // The center of mass of the Pointer_link2
		const MobilizedBody& endlink = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(2));
		return velocity = endlink.findStationVelocityInGround(s, station);
	}

	Matrix PDController2::JT(const State& s, int i) const {
		Matrix J(3, 8);
		Vec3 station(0, 0, 0); // The center of mass of the Pointer_link2
		_model->getMatterSubsystem().calcStationJacobian(s, MobilizedBodyIndex(i), station, J);
		Matrix JT = J.transpose();
//		std::cout << "J: " << J << std::endl;
//		std::cout << "JT: " << JT << std::endl;
		//		return JT;
		Matrix JT63(6, 3);

		for (unsigned int iT = 0; iT < 6; iT++) {

			for (unsigned int iJ = 0; iJ < 3; iJ++) {
				JT63[iT][iJ] = JT[iT+2][iJ];
			};
		}; 
		return JT63; 
	}

/*	Matrix PDController2::Lambda(const State& s) const {
		Matrix J(3, 2);
		Vec3 station(1.0, 0, 0);
		_model->getMatterSubsystem().calcStationJacobian(s, MobilizedBodyIndex(2), station, J);
		Matrix JT = J.transpose();

		// M{-1} * J{T}

		Matrix J_MInv_JT(3, 3);
		Vector MInv_JTcol(2);
		Vector JTcol(2);
		Vector f_GP(3, Real(0));

		for (unsigned int iT = 0; iT < 3; iT++) {
			f_GP[iT] = 1;
			JTcol = JT * f_GP;
			f_GP[iT] = 0;

			_model->getMatterSubsystem().multiplyByMInv(s, JTcol, MInv_JTcol);

			J_MInv_JT(iT) = J * MInv_JTcol;
		}

		Matrix J_Minv_JTInv;
		SimTK::FactorSVD svd;
		svd.factor(J_MInv_JT);
		svd.inverse(J_Minv_JTInv);

		return J_Minv_JTInv;
	}

	Vec3 PDController2::JacobianDotTimesU(const State& s) const {
		Vec3 station(1.0, 0, 0);
		Vec3 JDotU = _model->getMatterSubsystem().calcBiasForStationJacobian(s, MobilizedBodyIndex(2), station);
		return JDotU;
	}

	Vec3 PDController2::controlLaw1(const State& s, Vec3 desPosition) const {
		//	Vec3 desPosition(-1, 1, 0);
		Vec3 position = locationOfStationinG(s, 1);
		Vec3 velocity = velocityOfStationinG(s);
		Mat33 KP(kp, 0, 0, 0, kp, 0, 0, 0, kp), KV(kv, 0, 0, 0, kv, 0, 0, 0, kv);
		Vec3 ax = -KP*(position - desPosition) - KV*(velocity);
		return ax;
	}
	*/

	Vector PDController2::stiffnessControl(const State& s, Vec3 desPosition, int kpx, int kpy, int kpz, int kv) const {

		int i = 14; // This defines the end effector body -- Pointer_link2

		Vec3 position = locationOfStationinG(s, i);
		Mat33 KP(kpx, 0, 0, 0, kpy, 0, 0, 0, kpz); 
		Matrix KV(6, 6);

		KV.updDiag() = Vector(6, kv);

		Vec3 ax = KP*(desPosition - position);
		Vector control(3, Real(0));
		control[0] = ax[0]; control[1] = ax[1]; control[2] = ax[2];

		const Coordinate& j1 = _model->getCoordinateSet().get("hip_flexion_r");
		const Coordinate& j2 = _model->getCoordinateSet().get("hip_adduction_r");
		const Coordinate& j3 = _model->getCoordinateSet().get("hip_rotation_r");
		const Coordinate& j4 = _model->getCoordinateSet().get("knee_angle_r");
		const Coordinate& j5 = _model->getCoordinateSet().get("ankle_angle_r");
		const Coordinate& j6 = _model->getCoordinateSet().get("subtalar_angle_r");
		

		double dq1 = j1.getSpeedValue(s);
		double dq2 = j2.getSpeedValue(s);
		double dq3 = j3.getSpeedValue(s);
		double dq4 = j4.getSpeedValue(s);
		double dq5 = j5.getSpeedValue(s);
		double dq6 = j6.getSpeedValue(s);
		
		Vector dq(6, Real(0));
		dq[0] = dq1; dq[1] = dq2; dq[2] = dq3; dq[3] = dq4; dq[4] = dq5; dq[5] = dq6;

		Vector tau = JT(s, i) * control - KV * dq - G61(s);

		return tau;
	}

	void PDController2::setContactForces(Vector con) {
		cf = con;
	}

	Vector PDController2::getContactForces() const {
		return cf;
	}

	void PDController2::setdesPosition(Vec3 pos) {
		desPos = pos;
	}

	Vec3 PDController2::getdesPosition() const {
		return desPos;
	}
	//_____________________________________________________________________________
	void PDController2::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
	{
		double t = s.getTime();

		Vector controlTorque;

		Vector contForce = getContactForces();
		Vec3 desPosition = getdesPosition();

		if (vecNorm(contForce) > 0)
			controlTorque = stiffnessControl(s, desPosition, 25000, 25000, 1000, 500);
		else
			controlTorque = stiffnessControl(s, desPosition, 5000, 5000, 1000, 300);

		const Actuator& j1_act = _model->getActuators().get("Act-hip_flexion_r");
		const Actuator& j2_act = _model->getActuators().get("Act-hip_adduction_r");
		const Actuator& j3_act = _model->getActuators().get("Act-hip_rotation_r");
		const Actuator& j4_act = _model->getActuators().get("Act-knee_angle_r");
		const Actuator& j5_act = _model->getActuators().get("Act-ankle_angle_r");
		const Actuator& j6_act = _model->getActuators().get("Act-subtalar_angle_r");
//		const Actuator& j7_act = _model->getActuators().get("Act-mtp_angle_r");

		double FOpt_1 = j1_act.getOptimalForce();
		double FOpt_2 = j2_act.getOptimalForce();
		double FOpt_3 = j3_act.getOptimalForce();
		double FOpt_4 = j4_act.getOptimalForce();
		double FOpt_5 = j5_act.getOptimalForce();
		double FOpt_6 = j6_act.getOptimalForce();
//		double FOpt_7 = j7_act.getOptimalForce();


		double control_1 = controlTorque.get(0) / FOpt_1;
		double control_2 = controlTorque.get(1) / FOpt_2;
		double control_3 = controlTorque.get(2) / FOpt_3;
		double control_4 = controlTorque.get(3) / FOpt_4;
		double control_5 = controlTorque.get(4) / FOpt_5;
		double control_6 = controlTorque.get(5) / FOpt_6;
//		double control_7 = controlTorque.get(6) / FOpt_7;

		//	std::cout << "control is " << controlTorque << std::endl;

		Vec7 control(control_1, control_2, control_3, control_4, control_5, control_6);

		Vector control1(1, control[0]);
		Vector control2(1, control[1]);
		Vector control3(1, control[2]);
		Vector control4(1, control[3]);
		Vector control5(1, control[4]);
		Vector control6(1, control[5]);
//		Vector control7(1, control[6]);

		j1_act.addInControls(control1, controls);
		j2_act.addInControls(control2, controls);
		j3_act.addInControls(control3, controls);
		j4_act.addInControls(control4, controls);
		j5_act.addInControls(control5, controls);
		j6_act.addInControls(control6, controls);
//		j7_act.addInControls(control7, controls);

	}
};

Vector force(const Storage& forcestorage, double finalTime) {
	Vector f(6, Real(0));
	Vector data(262, Real(0));
	forcestorage.getDataAtTime(finalTime, 262, data);

	for (int j = 0; j < 3; j++) {
		f[j] = data[j + 100 + 3];
		f[j + 3] = data[j + 100];
	}

	return f;
};

int main()
{
	try {

		double System_frequency = 1000.0;
		double System_timestep = 1.0 / System_frequency;
		double System_StartTim = 00.00;
		double System_End_Time = 1.50;
		int System_Num_step = (int)round((System_End_Time - System_StartTim) / System_timestep);

		// Create an OpenSim model and set its name
		//		Model osimModel("Two_Link_Arm.osim");
		Model osimModel("Driver_Leg-20171019_6DOF.osim");

		PDController2 *controller = new PDController2();
		controller->setName("my_controller");
		controller->setActuators(osimModel.updActuators());
		osimModel.addController(controller);

		ForceReporter *aForceReporter = new ForceReporter();
		BodyKinematics *aBodyKinematics = new BodyKinematics();
		PointKinematics *aPointKinematics = new PointKinematics();

		osimModel.addAnalysis(aForceReporter);
		osimModel.addAnalysis(aBodyKinematics);


		State& s = osimModel.initSystem();

		CoordinateSet& modelSet = osimModel.updCoordinateSet();

		std::cout << "System Initialized" << std::endl;

		std::cout << "getMultibodySystem Done" << std::endl;

		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-3);
		integrator.setMinimumStepSize(5e-7);
		Manager manager(osimModel, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 5;

		Vec3 A(0.845, -0.1968, 0.1788);
		Vec3 B(0.872, -0.2188, 0.1788);
		Vec3 C(0.887, -0.228, 0.1788);
		Vec3 D(0.8903, -0.1712, 0.01528);
		Vec3 E(0.9226, -0.1661, 0.01528);
		Vec3 F(0.6857, -0.3832, 0.1332);

		Vector contForce(6, Real(0));

		for (int j = 0; j < 4; j++)
		{
			double System_frequency = 1000.0;
			double System_timestep = 1.0 / System_frequency;
			int System_StartTim = j;
			double System_End_Time = 1.00;
			int System_Num_step = 2000;

			for (int i = 0; i < System_Num_step; i++)
			{
				// Define the initial and final simulation times
				initialTime = System_StartTim * 2 + System_timestep * i;
				if (i == System_Num_step - 1)
					finalTime = System_End_Time;
				else
					finalTime = initialTime + System_timestep;

				controller->setContactForces(contForce);

				switch (j)
				{
				case 0:
					controller->setdesPosition(A);
					break;
				case 1:
					controller->setdesPosition(B);
					break;
				case 2:
					controller->setdesPosition(D);
					break;
				case 3:
					controller->setdesPosition(F);
					break;
				}

				// Integrate from initial time to final time
				manager.setInitialTime(initialTime);
				manager.setFinalTime(finalTime);
				std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
				manager.integrate(s);
				std::cout << " Cycle [ " << i + 1 << " / " << System_Num_step << " ] has done! \n" << std::endl;

				const Storage& forcestorage = aForceReporter->getForceStorage();
				contForce = force(forcestorage, finalTime);

							std::cout << contForce << std::endl;

			}
		}
		
		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("./Results/driverleg-states_ABDF.sto");
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		osimModel.printControlStorage("./Results/driverleg-controls_ABDF.sto");

		aForceReporter->printResults("ForceReport_ABDF", "./Results/ForceData", 0.005, ".sto");
		aBodyKinematics->printResults("BodyKinematics_ABDF", "./Results/KinematicsData", 0.005, ".sto");


	}
	catch (OpenSim::Exception ex)
	{
		std::cout << ex.getMessage() << std::endl;
		return 1;
	}
	catch (std::exception ex)
	{
		std::cout << ex.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		return 1;
	}

	std::cout << "OpenSim example completed successfully.\n";
	std::cin.get();
	return 0;
}



