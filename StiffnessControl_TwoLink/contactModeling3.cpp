/* --------------------------------------------------------------------------*
*                  Two Link Controller Using ForceReporter Method            *
* -------------------------------------------------------------------------- *
*/

#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "Simbody.h"
#include "C:\OpenSim 3.3\sdk\include\OpenSim\Simulation\Control\Controller.h"


using namespace OpenSim;
using namespace SimTK;

double vecNorm(Vector value) {
	double norm;
	double sum = square(value(0)) + square(value(1)) + square(value(2)) + square(value(3)) + square(value(4)) + square(value(5));
	return norm = sqrt(sum);
}

class PDController2 : public Controller
{
	OpenSim_DECLARE_CONCRETE_OBJECT(PDController2, Controller);

private:
	double kp;
	double kv;
	Vector cf;
	Vec3 desPos;
		
public:

	PDController2::PDController2() : Controller() {
		setKp(20000);
		setKv(100);
		Vector con(6, Real(0));
		setContactForces(con);
		Vec3 pos(0);
		setdesPosition(pos);
	}

	void PDController2::setKp(double kp_value) {
		kp = kp_value;
	}

	void PDController2::setKv(double kv_value) {
		kv = kv_value;
	}

	Vector PDController2::VelocityVector(const State& s) const {

		Vector c;

		_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
		_model->getMatterSubsystem().calcResidualForceIgnoringConstraints(s, SimTK::Vector(0), SimTK::Vector_<SimTK::SpatialVec>(0), SimTK::Vector(0), c);

		return c;
	}

	Vector PDController2::C21(const State& s) const {
		Vector c21(2, Real(0));
		Vector c31 = VelocityVector(s);

		for (unsigned int iJ = 0; iJ < 2; iJ++) {
			c21[iJ] = c31[iJ];
		};

		return c21;
	}

	Vector PDController2::GravityVector(const State& s) const {
		Vector g;

		_model->getMatterSubsystem().multiplyBySystemJacobianTranspose(s, _model->getGravityForce().getBodyForces(s), g);

		return g;
	}

	Vector PDController2::G21(const State& s) const {
		Vector g21(2, Real(0));
		Vector g31 = GravityVector(s);

		for (unsigned int iJ = 0; iJ < 2; iJ++) {
			g21[iJ] = g31[iJ];
		};

		return g21;
	}

	Matrix PDController2::MassMatrix(const State& s) const {
		int numMobilities = s.getNU();
		Matrix M(numMobilities, numMobilities);

		_model->getMatterSubsystem().calcM(s, M);

		return M;

		//		std::cout << "M: " << M << std::endl;
	}

	Matrix PDController2::Mass22(const State& s) const {
		Matrix Mass33 = MassMatrix(s);
		//	Vector f(7, Real(0));
		Matrix Mass22(2,2);
		//	Vector Mass97col;

		for (unsigned int iT = 0; iT < 2; iT++) {

			for (unsigned int iJ = 0; iJ < 2; iJ++) {
				Mass22[iT][iJ] = Mass33[iT][iJ];
			};
		};
		return Mass22;
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
		Matrix J(3, 2);
		Vec3 station(0, 0, 0); // The center of mass of the Pointer_link2
		_model->getMatterSubsystem().calcStationJacobian(s, MobilizedBodyIndex(i), station, J);
		Matrix JT = J.transpose();
//		return JT;
		Matrix JT23(2, 3);

		for (unsigned int iT = 0; iT < 3; iT++) {

			for (unsigned int iJ = 0; iJ < 2; iJ++) {
				JT23[iT][iJ] = JT[iT][iJ];
			};
		};
		return JT23;
	}

	Matrix PDController2::JT26(const State& s) const {
		Matrix J63;
		Vec3 pos(1, 0, 0);
		Array_<Vec3> station(1, pos);
		Array_<MobilizedBodyIndex> mobodInd(1);

		mobodInd[0] = MobilizedBodyIndex(2);

		_model->getMatterSubsystem().calcFrameJacobian(s, mobodInd, station, J63);

		Matrix JT63 = J63.transpose();

		//	std::cout << "JT63: " << JT63 << std::endl;

		Matrix JT62(2, 6);

		for (unsigned int iT = 0; iT < 2; iT++) {

			for (unsigned int iJ = 0; iJ < 6; iJ++) {
				JT62[iT][iJ] = JT63[iT][iJ];
			};
		};

		//	std::cout << "JT62: " << JT62 << std::endl;
		return JT62;
	}

	Matrix PDController2::Lambda(const State& s) const {
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

	Vector PDController2::stiffnessControl(const State& s, Vec3 desPosition, int kpx, int kpy, int kv) const {
		
		int i = 4; // This defines the end effector body -- Pointer_link2

		Vec3 position = locationOfStationinG(s, i);
		Vec3 velocity = velocityOfStationinG(s);
		Mat33 KP(kpx, 0, 0, 0, kpy, 0, 0, 0, kp); // The gains are only in the x and y directions, there's no motion in the z direction
		Matrix KV(2,2);

		KV.updDiag() = Vector(2, kv);

		Vec3 ax = KP*(desPosition - position);
		Vector control(3, Real(0));
		control[0] = ax[0]; control[1] = ax[1]; control[2] = ax[2];

		const Coordinate& joint1 = _model->getCoordinateSet().get("joint_1");
		const Coordinate& joint2 = _model->getCoordinateSet().get("joint_2");

		double dq1 = joint1.getSpeedValue(s);
		double dq2 = joint2.getSpeedValue(s);

		Vector dq(2, Real(0));
		dq[0] = dq1; dq[1] = dq2;

		Vector tau = JT(s, i) * control - KV * dq - G21(s);

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

		std::cout << _model->getBodySet() << std::endl;
//		std::cout << _model->getCoordinateSet() << std::endl;

//		std::cout << "Time is: " << t << "  " << locationOfStationinG(s, 4) << std::endl;
		std::cout << locationOfStationinG(s, 4) << std::endl;
//		JT(s, 4);


/*		const Coordinate& joint1 = _model->getCoordinateSet().get("joint_1");
		const Coordinate& joint2 = _model->getCoordinateSet().get("joint_2");

		double j1 = joint1.getValue(s);
		double j1_u = joint1.getSpeedValue(s);
		double j2 = joint2.getValue(s);
		double j2_u = joint2.getSpeedValue(s);

		double  j1_des = 75 * Pi / 180;
		double  j1_u_des = 0.0;
		double  j2_des = -70 * Pi / 180;
		double  j2_u_des = 0.0;

		Vec2 controlLaw;
		Vec2 theta(j1, j2);
		Vec2 theta_u(j1_u, j2_u);
		Vec2 theta_d(j1_des, j2_des);
		Vec2 theta_u_d(j1_u_des, j2_u_des);

		Mat22 KP(kp, 0, 0, kp), KV(kv, 0, 0, kv);

//		const OpenSim::Model model = getModel();

		controlLaw = -KP*(theta - theta_d) - KV*(theta_u - theta_u_d);

//		std::cout << "controlLaw = " << controlLaw << std::endl;

//		std::cout << "In the controller:  " << cf << std::endl;

		Vector g;
		Vector C_Vector, G_Vector, controlTorque;
		Matrix Mass;

		C_Vector = VelocityVector(s);
		G_Vector = GravityVector(s);
		Mass = MassMatrix(s);
*/
		Vector controlTorque;

		Vec3 desPosition = getdesPosition();

//		Vec3 desPosition(0.42, 0.21, 0);

		Vector contForce = getContactForces();

//		double st = 16;
//		double ss = sqrt(st);
		

//		st = square(16);

//		std::cout << "contact force: " << st << std::endl;

//		std::cout << "Norm of contact force: " << vecNorm(contForce) << std::endl;

		if (vecNorm(contForce) > 0)
			controlTorque = stiffnessControl(s, desPosition, 25000, 3000, 400);
		else 
			controlTorque = stiffnessControl(s, desPosition, 200, 200, 20);


//		std::cout << "Torque_Force:  " << controlTorque << std::endl;

//		Vector controlTorque1 = Mass*Vector_<Real>(controlLaw) + C_Vector - G_Vector;

//		controlTorque = Mass*Vector_<Real>(controlLaw) + C_Vector - G_Vector + JT26(s) * cf;

//		controlTorque = Mass22(s) * Vector_<Real>(controlLaw) + C21(s) - G21(s) + JT26(s) * cf;

//		controlTorque = Mass22(s) * Vector_<Real>(controlLaw) + C21(s) - G21(s);

//		std::cout << "diff: " << controlTorque1 - controlTorque << std::endl;

//		std::cout << "Torque_NoForce:  " << controlTorque1 << std::endl;
//		std::cout << "Torque_Force:  " << controlTorque << std::endl;

/*		std::cout << "The code is running. The current runtime is: " << s.getTime() << std::endl;
		std::cout << "Mass is " << Mass << std::endl;
		std::cout << "The G_Vector is " << G_Vector << std::endl;
		std::cout << "The C_Vector is " << C_Vector << std::endl;
		std::cout << "controlTorque is " << controlTorque << std::endl;
		*/

		const Actuator& j1_act = _model->getActuators().get("joint_1_actuator");
		const Actuator& j2_act = _model->getActuators().get("joint_2_actuator");


		double FOpt_1 = j1_act.getOptimalForce();
		double FOpt_2 = j2_act.getOptimalForce();


		double control_1 = controlTorque.get(0) / FOpt_1;
		double control_2 = controlTorque.get(1) / FOpt_2;

		Vec7 control(control_1, control_2);

		Vector control1(1, control[0]);
		Vector control2(1, control[1]);

		j1_act.addInControls(control1, controls);
		j2_act.addInControls(control2, controls);

	}
};

Vector force(const Storage& forcestorage, double finalTime){
	Vector f(6, Real(0));
	Vector data(32, Real(0));
	forcestorage.getDataAtTime(finalTime, 32, data);

	for (int j = 0; j < 3; j++) {
		f[j] = data[j + 14 + 3];
		f[j + 3] = data[j + 14];
	}

	return f;
};

int main()
{
	try {

		double System_frequency = 100.0;
		double System_timestep = 1.0 / System_frequency;
		double System_StartTim = 00.00;
		double System_End_Time = 10.00;
		int System_Num_step = (int)round((System_End_Time - System_StartTim) / System_timestep);

		// Create an OpenSim model and set its name
//		Model osimModel("Two_Link_Arm.osim");
		Model osimModel("TwoLinkArmModel.osim");
//		osimModel.setUseVisualizer(true);

	//	double kp = 100.0;
	//	double kv = 20.0;

		PDController2 *controller = new PDController2();
		controller->setName("my_controller");
		controller->setActuators(osimModel.updActuators());
		osimModel.addController(controller);

		ForceReporter *aForceReporter = new ForceReporter();
		BodyKinematics *aBodyKinematics = new BodyKinematics();
		PointKinematics *aPointKinematics = new PointKinematics();

		osimModel.addAnalysis(aForceReporter);
		osimModel.addAnalysis(aBodyKinematics);

//		OpenSim::Body link_2 = osimModel.getBodySet().get("link2");

//		aPointKinematics->setBody(link_2);

		osimModel.addAnalysis(aPointKinematics);

		State& s = osimModel.initSystem();

		CoordinateSet& modelSet = osimModel.updCoordinateSet();

		const Coordinate& joint1 = osimModel.getCoordinateSet().get("joint_1");
		const Coordinate& joint2 = osimModel.getCoordinateSet().get("joint_2");
		const Coordinate& trans = osimModel.getCoordinateSet().get("transX");

		joint1.setValue(s, 120*Pi/180);
//		joint1.setValue(s, Pi / 2);
		joint1.setSpeedValue(s, 0);
//		joint2.setValue(s, -90*Pi/180);
		joint2.setValue(s, -120*Pi/180);
		joint2.setSpeedValue(s, 0);
//		trans.setValue(s, 0.5);
//		trans.setSpeedValue(s, 0);

		std::cout << "System Initialized" << std::endl;

		std::cout << "getMultibodySystem Done" << std::endl;

		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-3);
		integrator.setMinimumStepSize(5e-7);
		Manager manager(osimModel, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 5;

		
//		osimModel.updMatterSubsystem().setShowDefaultGeometry(true);
//		Visualizer& viz = osimModel.updVisualizer().updSimbodyVisualizer();

		// Integrate from initial time to final time
//		manager.setInitialTime(initialTime);
//		manager.setFinalTime(finalTime);
//		std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
//		manager.integrate(s);


		Vec3 A(0.42, 0.21, 0.0);
		Vec3 B(0.32, 0.21, 0.0);
		Vec3 C(0.2, 0.21, 0.0);

		Vector contForce(6, Real(0));
		for (int j = 0; j < 3; j++)
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
					controller->setdesPosition(A);
					break;
				case 2:
					controller->setdesPosition(B);
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

				//			std::cout << contForce << std::endl;

			}
		}

		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("./Results/twolink-states_SCNew2.sto");
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		osimModel.printControlStorage("./Results/twolink-controls_SCNew2.sto");

		aForceReporter->printResults("ForceReport_SCNew2", "./Results/ForceData", 0.005, ".sto");
		aBodyKinematics->printResults("BodyKinematics_SCNew2", "./Results/KinematicsData", 0.005, ".sto");
//		aPointKinematics->printResults("PointKinematics_SC100", "./Results/KinematicsData", 0.005, ".sto");

//		const Storage& forcestorage = aForceReporter->getForceStorage();
//		int size = forcestorage.getSize();
//		Array<std::string> labels = forcestorage.getColumnLabels();
//		Vector data(32);
//		forcestorage.getDataAtTime(0.01, 32, data);
//		std::cout << data << std::endl;

		
		
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



