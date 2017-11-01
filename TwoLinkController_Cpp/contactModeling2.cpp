/* --------------------------------------------------------------------------*
*                  Two Link Controller Using getForceSet Method		        *
* -------------------------------------------------------------------------- *
*/

#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include "Simbody.h"
#include "C:\OpenSim 3.3\sdk\include\OpenSim\Simulation\Control\Controller.h"


using namespace OpenSim;
using namespace SimTK;

class PDController2 : public Controller
{
	OpenSim_DECLARE_CONCRETE_OBJECT(PDController2, Controller);

private:
	double kp;
	double kv;

public:

	PDController2::PDController2() : Controller() {
		setKp(100);
		setKv(20);
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

	}

	Matrix PDController2::Mass22(const State& s) const  {
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

	Vector PDController2::cForce(const State& s) const {
		
		_model->getMultibodySystem().realize(s, SimTK::Stage::Dynamics);
		Array<double> F = _model->getForceSet().get(3).getRecordValues(s);

		Vector cf(6, Real(0));

		for (unsigned int i = 0; i < 6; i++) {
			cf[i] = F[i];
		};

		return cf;
	} 

	Vec3 PDController2::locationOfStationinG(const State& s) const {
		Vec3 location;
		Vec3 station(1.0, 0, 0);
		const MobilizedBody& endlink = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(2));
		return location = endlink.findStationLocationInGround(s, station);
	}

	Vec3 PDController2::velocityOfStationinG(const State& s) const {
		Vec3 velocity;
		Vec3 station(1.0, 0, 0);
		const MobilizedBody& endlink = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(2));
		return velocity = endlink.findStationVelocityInGround(s, station);
	}

	Matrix PDController2::JT(const State& s) const {
		Matrix J(3, 2);
		Vec3 station(1.0, 0, 0);
		_model->getMatterSubsystem().calcStationJacobian(s, MobilizedBodyIndex(2), station, J);
		Matrix JT = J.transpose();
//		return JT;
		Matrix JT23(2, 3);

		for (unsigned int iT = 0; iT < 2; iT++) {

			for (unsigned int iJ = 0; iJ < 2; iJ++) {
				JT23[iT][iJ] = JT[iT][iJ];
			};
		};
		return JT23;
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
		Vec3 position = locationOfStationinG(s);
		Vec3 velocity = velocityOfStationinG(s);
		Mat33 KP(kp, 0, 0, 0, kp, 0, 0, 0, kp), KV(kv, 0, 0, 0, kv, 0, 0, 0, kv);
		Vec3 ax = -KP*(position - desPosition) - KV*(velocity);
		return ax;
	}

	Vec2 PDController2::controlLaw(const State& s, Vector desQ) const {
		const Coordinate& joint1 = _model->getCoordinateSet().get("joint_1");
		const Coordinate& joint2 = _model->getCoordinateSet().get("joint_2");

		double q1 = joint1.getValue(s);
		double dq1 = joint1.getSpeedValue(s);
		double q2 = joint2.getValue(s);
		double dq2 = joint2.getSpeedValue(s);

		Vec2 controlLaw;
		Vec2 q(q1, q2);
		Vec2 dq(dq1, dq2);
		Vec2 q_des(desQ[0], desQ[1]);
		Vec2 dq_des(desQ[2], desQ[3]);

		Mat22 KP(kp, 0, 0, kp), KV(kv, 0, 0, kv);

		return controlLaw = -KP*(q - q_des) - KV*(dq - dq_des);

	}

	Vector PDController2::stiffnessControl(const State& s, Vec3 desPosition) const {
		//	Vec3 desPosition(-1, 1, 0);
		Vec3 position = locationOfStationinG(s);
		Vec3 velocity = velocityOfStationinG(s);
		Mat33 KP(kp, 0, 0, 0, kp, 0, 0, 0, kp);
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

		Vector tau = JT(s) * control - KV * dq + G21(s);

		return tau;
	}

	Matrix PDController2::JT26(const State& s) const {
		Matrix J63;
		Vec3 pos(0, 0, 0);
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

	//_____________________________________________________________________________
	void PDController2::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
	{
		double t = s.getTime();

//		Vector C = C21(s);
//		Vec3 desPosition(1.2, 0.75, 0);
//		Vec3 desPosition(-0.75, 1.2, 0);

		
//		Array<double> F = _model->getForceSet().get(3).getRecordValues(s);
//		Array<std::string> labels = _model->getForceSet().get(3).getRecordLabels();

//		std::cout << "Force Values are" << F << std::endl;
//		std::cout << "Force Values are" << cForce(s) << std::endl;

//		Matrix J = JT26(s);

//		std::cout << "stiffness control: " << contr << std::endl;

//		ForceSet forces = _model->getForceSet();
		//	std::cout << "forceset: " << forces << std::endl;
//		OpenSim::Force& contact = forces.get(3);
//		std::cout << contact.getConcreteClassName() << std::endl;

		//	std::cout << "contactforce: " << contact << std::endl;
//		Array<double> F(3);
//		_model->getMultibodySystem().realize(s, SimTK::Stage::Acceleration);
		//	Array<std::string> labels = contact.getRecordLabels();
//		F = contact.getRecordValues(s);
//		BodySet body = _model->getBodySet();
//		std::cout << "body: " << body << std::endl;
/*
		const Coordinate& joint1 = _model->getCoordinateSet().get("joint_1");
		const Coordinate& joint2 = _model->getCoordinateSet().get("joint_2");

		double q1 = joint1.getValue(s);
		double dq1 = joint1.getSpeedValue(s);
		double q2 = joint2.getValue(s);
		double dq2 = joint2.getSpeedValue(s);
*/		
		Vector desQ(4, Real(0));

		desQ[0] = 70 * Pi / 180; //q1
		desQ[1] = 0.0;	//dq1
		desQ[2] = -75 * Pi / 2;	//q2
		desQ[3] = 0.0;	//dq2

		//		std::cout << "controlLaw = " << controlLaw << std::endl;

		Vec2 ax = controlLaw(s, desQ);

		Vector g;
		Vector C_Vector, G_Vector, controlTorque;
		Matrix Mass;

		C_Vector = VelocityVector(s);
		G_Vector = GravityVector(s);
		Mass = MassMatrix(s);

		controlTorque = Mass22(s) * Vector_<Real>(ax) + C21(s) - G21(s) + JT26(s)*cForce(s);

//		controlTorque = Mass*Vector_<Real>(controlLaw) + C_Vector - G_Vector;

		std::cout << "The code is running. The current runtime is: " << s.getTime() << std::endl;

		const Actuator& j1_act = _model->getActuators().get("joint_1_actuator");
		const Actuator& j2_act = _model->getActuators().get("joint_2_actuator");


		double FOpt_1 = j1_act.getOptimalForce();
		double FOpt_2 = j2_act.getOptimalForce();


		double control_1 = controlTorque.get(0) / FOpt_1;
		double control_2 = controlTorque.get(1) / FOpt_2;

		Vec2 control(control_1, control_2);

		Vector control1(1, control[0]);
		Vector control2(1, control[1]);

		j1_act.addInControls(control1, controls);
		j2_act.addInControls(control2, controls);

	}
};

int main()
{
	try {
		// Create an OpenSim model and set its name
//		Model osimModel("Two_Link_Arm.osim");
		Model osimModel("TwoLinkArmModel.osim");
//		osimModel.setUseVisualizer(true);

		double kp = 100.0;
		double kv = 20.0;

		PDController2 *controller = new PDController2();
		controller->setName("my_controller");
		controller->setActuators(osimModel.updActuators());
		osimModel.addController(controller);

		State& s = osimModel.initSystem();

		CoordinateSet& modelSet = osimModel.updCoordinateSet();

		const Coordinate& joint1 = osimModel.getCoordinateSet().get("joint_1");
		const Coordinate& joint2 = osimModel.getCoordinateSet().get("joint_2");

		joint1.setValue(s, 119*Pi/180);
		joint1.setSpeedValue(s, 0);
		joint2.setValue(s, -110*Pi/180);
		joint2.setSpeedValue(s, 0);

		std::cout << "System Initialized" << std::endl;

		std::cout << "getMultibodySystem Done" << std::endl;

		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-3);
		Manager manager(osimModel, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 1.0;

//		osimModel.updMatterSubsystem().setShowDefaultGeometry(true);
//		Visualizer& viz = osimModel.updVisualizer().updSimbodyVisualizer();

		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
		manager.integrate(s);

		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("./Results/twolink-states2.sto");
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		osimModel.printControlStorage("./Results/twolink-controls2.sto");

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



