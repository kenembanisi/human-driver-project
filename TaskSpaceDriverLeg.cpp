/* --------------------------------------------------------------------------*
* Title: Indirect Force Control of the Driver leg Model
*
* This includes the implementation of an Impedance control and an Active
* Compliance control
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

Vector a(double pC, double pD, double t0, double tf) {
	Vector a(6, Real(0));
	double den = (t0 - tf)*(pow(t0, 3) - 3 * pow(t0, 2)*tf + 3 * pow(tf, 2)*t0 - pow(tf, 3));

	a[0] = (pD*pow(t0, 5) - 5 * pD*pow(t0, 4)*tf + 10 * pD*pow(t0, 3)*pow(tf, 2) - 10 * pC*pow(t0, 2)*pow(tf, 3) + 5 * pC*t0*pow(tf, 4) - pC*pow(tf, 5))/den;
	a[1] = (30 * (pC*pow(t0, 2)*pow(tf, 2) - pD*pow(t0, 2)*pow(tf, 2)))/den;
	a[2] = (-30 * (pC*t0*pow(tf, 2) - pC*pow(t0, 2)*tf - pD*t0*pow(tf, 2) - pD*pow(t0, 2)*tf))/den;
	a[3] = (10 * (pC*pow(t0, 2) - pD*pow(t0, 2) + pC*pow(tf, 2) - pD*pow(tf, 2) + 4 * pC*t0*tf - 4 * pD*t0*tf))/den;
	a[4] = (-15 * (t0 - tf)*(pC - pD))/den;
	a[5] = (6 * (pC - pD))/den;

	return a;
}

Vector desPositionTraj(Vector pC, Vector pD, double t0, double tf, double t) {
	Vector pos(6, Real(0));
	
	/*Matrix A(6, 6);
	A[0][0] = 1;		A[0][1] = t0;		A[0][2] = pow(t0, 2);		A[0][3] = pow(t0, 3);		A[0][4] = pow(t0, 4);		A[0][5] = pow(t0, 5);
	A[1][0] = 0;		A[1][1] = 1;			A[1][2] = 2*t0;				A[1][3] = 3*pow(t0, 2);		A[1][4] = 4 * pow(t0, 3);	A[1][5] = 5 * pow(t0, 4);
	A[2][0] = 0;		A[2][1] = 0;			A[2][2] = 2;					A[2][3] = 6*t0;				A[2][4] = 12*pow(t0, 2);		A[2][5] = 20*pow(t0, 3);
	A[3][0] = 1;		A[3][1] = tf;		A[3][2] = pow(tf, 2);		A[3][3] = pow(tf, 3);		A[3][4] = pow(tf, 4);		A[3][5] = pow(tf, 5);
	A[4][0] = 0;		A[4][1] = 1;			A[4][2] = 2 * tf;			A[4][3] = 3 * pow(tf, 2);	A[4][4] = 4 * pow(tf, 3);	A[4][5] = 5 * pow(tf, 4);
	A[5][0] = 0;		A[5][1] = 0;			A[5][2] = 2;					A[5][3] = 6 * tf;			A[5][4] = 12 * pow(tf, 2);	A[5][5] = 20 * pow(tf, 3);

	Vector b(6, Real(0));
	b[0] = */

	for (int i = 0; i < 6; i++) {
		Vector A = a(pC[i], pD[i], t0, tf);

		pos[i] = A[0] + A[1]*t + A(2)*pow(t, 2) + A(3)*pow(t, 3) + A(4)*pow(t, 4) + A(5)*pow(t, 5);
	}
	
	/*pos[0] = pC[0] + (1.25)*(pD[0] - pC[0])*pow(t,3) + (0.9375)*(pC[0] - pD[0])*pow(t, 4) + (0.1875)*(pD[0] - pC[0])*pow(t, 5);
	pos[1] = pC[1] + (1.25)*(pD[1] - pC[1])*pow(t, 3) + (0.9375)*(pC[1] - pD[1])*pow(t, 4) + (0.1875)*(pD[1] - pC[1])*pow(t, 5);
	pos[2] = pC[2] + (1.25)*(pD[2] - pC[2])*pow(t, 3) + (0.9375)*(pC[2] - pD[2])*pow(t, 4) + (0.1875)*(pD[2] - pC[2])*pow(t, 5);
	pos[3] = pC[3] + (1.25)*(pD[3] - pC[3])*pow(t, 3) + (0.9375)*(pC[3] - pD[3])*pow(t, 4) + (0.1875)*(pD[3] - pC[3])*pow(t, 5);
	pos[4] = pC[4] + (1.25)*(pD[4] - pC[4])*pow(t, 3) + (0.9375)*(pC[4] - pD[4])*pow(t, 4) + (0.1875)*(pD[4] - pC[4])*pow(t, 5);
	pos[5] = pC[5] + (1.25)*(pD[5] - pC[5])*pow(t, 3) + (0.9375)*(pC[5] - pD[5])*pow(t, 4) + (0.1875)*(pD[5] - pC[5])*pow(t, 5);*/
	
	return pos;
}

Vector desVelocityTraj(Vector pC, Vector pD, double t0, double tf, double t) {
	Vector vel(6, Real(0));

	for (int i = 0; i < 6; i++) {
		Vector A = a(pC[i], pD[i], t0, tf);

		vel[i] = A[1] + 2*A(2)*t + 3*A(3)*pow(t, 2) + 4*A(4)*pow(t, 3) + 5*A(5)*pow(t, 4);
	}

	/*vel[0] = 3 * (1.25)*(pD[0] - pC[0])*pow(t, 2) + 4 * (0.9375)*(pC[0] - pD[0])*pow(t, 3) + 5 * (0.1875)*(pD[0] - pC[0])*pow(t, 4);
	vel[1] = 3 * (1.25)*(pD[1] - pC[1])*pow(t, 2) + 4 * (0.9375)*(pC[1] - pD[1])*pow(t, 3) + 5 * (0.1875)*(pD[1] - pC[1])*pow(t, 4);
	vel[2] = 3 * (1.25)*(pD[2] - pC[2])*pow(t, 2) + 4 * (0.9375)*(pC[2] - pD[2])*pow(t, 3) + 5 * (0.1875)*(pD[2] - pC[2])*pow(t, 4);
	vel[3] = 3 * (1.25)*(pD[3] - pC[3])*pow(t, 2) + 4 * (0.9375)*(pC[3] - pD[3])*pow(t, 3) + 5 * (0.1875)*(pD[3] - pC[3])*pow(t, 4);
	vel[4] = 3 * (1.25)*(pD[4] - pC[4])*pow(t, 2) + 4 * (0.9375)*(pC[4] - pD[4])*pow(t, 3) + 5 * (0.1875)*(pD[4] - pC[4])*pow(t, 4);
	vel[5] = 3 * (1.25)*(pD[5] - pC[5])*pow(t, 2) + 4 * (0.9375)*(pC[5] - pD[5])*pow(t, 3) + 5 * (0.1875)*(pD[5] - pC[5])*pow(t, 4);*/

	return vel;
}

Vector desAccTraj(Vector pC, Vector pD, double t0, double tf, double t) {
	Vector acc(6, Real(0));

	for (int i = 0; i < 6; i++) {
		Vector A = a(pC[i], pD[i], t0, tf);

		acc[i] = 2 * A(2) + 6 * A(3)*t + 12 * A(4)*pow(t, 2) + 20 * A(5)*pow(t, 3);
	}

	/*acc[0] = 6 * (1.25)*(pD[0] - pC[0])*pow(t, 1) + 12 * (0.9375)*(pC[0] - pD[0])*pow(t, 2) + 20 * (0.1875)*(pD[0] - pC[0])*pow(t, 3);
	acc[1] = 6 * (1.25)*(pD[1] - pC[1])*pow(t, 1) + 12 * (0.9375)*(pC[1] - pD[1])*pow(t, 2) + 20 * (0.1875)*(pD[1] - pC[1])*pow(t, 3);
	acc[2] = 6 * (1.25)*(pD[2] - pC[2])*pow(t, 1) + 12 * (0.9375)*(pC[2] - pD[2])*pow(t, 2) + 20 * (0.1875)*(pD[2] - pC[2])*pow(t, 3);
	acc[3] = 6 * (1.25)*(pD[3] - pC[3])*pow(t, 1) + 12 * (0.9375)*(pC[3] - pD[3])*pow(t, 2) + 20 * (0.1875)*(pD[3] - pC[3])*pow(t, 3);
	acc[4] = 6 * (1.25)*(pD[4] - pC[4])*pow(t, 1) + 12 * (0.9375)*(pC[4] - pD[4])*pow(t, 2) + 20 * (0.1875)*(pD[4] - pC[4])*pow(t, 3);
	acc[5] = 6 * (1.25)*(pD[5] - pC[5])*pow(t, 1) + 12 * (0.9375)*(pC[5] - pD[5])*pow(t, 2) + 20 * (0.1875)*(pD[5] - pC[5])*pow(t, 3);*/

	return acc;
}

Matrix T(Vector pose) {
	// Transformation matrix, T, corresponding to the XYZ convention
	// angVel = T(phi)

	Matrix T(3, 3);
	double a = pose[0], b = pose[1], c = pose[2];
	T[0][0] = 1;		T[0][1] = 0;			T[0][2] = sin(b);
	T[1][0] = 0;		T[1][1] = cos(a);	T[1][2] = -sin(a)*cos(b);
	T[2][0] = 0;		T[2][1] = sin(a);	T[2][2] = cos(a)*cos(b);

	return T;
}

Matrix Tdot(Vector pose, Vector angVel) {
	// Derivative of Transformation matrix, Tdot, corresponding to the XYZ convention
	
	Matrix T(3, 3);
	double a = pose[0], b = pose[1], c = pose[2];
	double adot = angVel[0], bdot = angVel[1], cdot = angVel[2];

	T[0][0] = 1;		T[0][1] = 0;					T[0][2] = -bdot * cos(b);
	T[1][0] = 0;		T[1][1] = -adot * sin(a);	T[1][2] = -adot * cos(a)*cos(b) + bdot * sin(a)*sin(b);
	T[2][0] = 0;		T[2][1] = -adot * cos(a);	T[2][2] = -adot * sin(a)*cos(b) - bdot * cos(a)*sin(b);

	return T;
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
	double initTime, finalTime;


public:

	// Constructor
	PDController2::PDController2() : Controller() {
		setKp(400);
		setKv(50);
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

	Vector PDController2::C61(const State& s) const {
		Vector c61(6, Real(0));
		Vector c81 = VelocityVector(s);

		for (unsigned int iJ = 0; iJ < 6; iJ++) {
			c61[iJ] = c81[iJ+2];
		};

		return c61;
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

	Matrix PDController2::Mass66(const State& s) const {
		Matrix Mass88 = MassMatrix(s);
		//	Vector f(7, Real(0));
		Matrix Mass66(6, 6);
		//	Vector Mass97col;

		for (unsigned int iT = 0; iT < 6; iT++) {

			for (unsigned int iJ = 0; iJ < 6; iJ++) {
				Mass66[iT][iJ] = Mass88[iT+2][iJ+2];
			};
		};
		return Mass66;
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

	Vector PDController2::framePoseinG(const State& s) const {
		Transform frame;
		Transform station(Vec3(0, 0, 0));
		const MobilizedBody& toepointerPos = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(14)); // 14 for Pointer_Toe body
		frame = toepointerPos.findFrameTransformInGround(s, station);
		Vec3 pos = frame.p();
		Vec3 rot = Rot2RPY(frame.R());

		Vector pose(pos.size() + rot.size());

		for (unsigned int iJ = 0; iJ < 3; iJ++) {
			pose[iJ + 3] = pos[iJ];
			pose[iJ] = rot[iJ];
		};

		return pose;
	}

	Vec3 PDController2::velocityOfStationinG(const State& s) const {
		Vec3 velocity;
		Vec3 station(0, 0, 0); // The center of mass of the Pointer_link2
		const MobilizedBody& endlink = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(2));
		return velocity = endlink.findStationVelocityInGround(s, station);
	}

	Vector PDController2::frameVelocityinG(const State& s) const {
		SpatialVec vel;
		Vec3 station(0, 0, 0);
		const MobilizedBody& toepointerVel = _model->getMatterSubsystem().getMobilizedBody(MobilizedBodyIndex(14)); // 14 for Pointer_Toe body
		vel = toepointerVel.findFrameVelocityInGround(s, station);

	//	std::cout << "vel: " << vel << std::endl;

		Vector velocity(6);

		for (unsigned int iJ = 0; iJ < 3; iJ++) {
			velocity[iJ] = vel[0][iJ];
			velocity[iJ + 3] = vel[1][iJ];
		};
	//	std::cout << "velocity: " << velocity << std::endl;
		return velocity;
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

	Matrix PDController2::J66(const State& s) const {
		Matrix J68;
		Vec3 pos(0, 0, 0);
		Array_<Vec3> station(1, pos);
		Array_<MobilizedBodyIndex> mobodInd(1);

		mobodInd[0] = MobilizedBodyIndex(14);

		_model->getMatterSubsystem().calcFrameJacobian(s, mobodInd, station, J68);

		Matrix J66(6, 6);

		for (unsigned int iT = 0; iT < 6; iT++) {

			for (unsigned int iJ = 0; iJ < 6; iJ++) {
				J66[iT][iJ] = J68[iT][iJ + 2];
			};
		};
		return J66;
	}

	Matrix PDController2::J66Inv(const State& s) const {
		Matrix J66Inv;

		SimTK::FactorSVD svd;
		svd.factor(J66(s));
		svd.inverse(J66Inv);
	
		return J66Inv;
	}

	Vector PDController2::JDotU6(const State& s) const {
		Vec3 pos(0, 0, 0);
		Vector JDotU6;
		Array_<Vec3> station(1, pos);
		Array_<MobilizedBodyIndex> mobodInd(1);
		mobodInd[0] = MobilizedBodyIndex(14);
		_model->getMatterSubsystem().calcBiasForFrameJacobian(s, mobodInd, station, JDotU6);
		return JDotU6;
	}

	Vec3 PDController2::Rot2RPY(const Rotation R) const {

		// Euler Angles using the XYZ convention

	/*	double p, x, y, z;
		Vec3 rpy;
		p = sqrt(pow(R[2][1], 2) + pow(R[2][2], 2));
		x = atan2(R[2][1], R[2][2]);
		y = atan2(-R[2][0], p);
		z = atan2(R[1][0], R[0][0]);
		rpy.set(0, x); rpy.set(1, y); rpy.set(2, z); */

		double p, a, b, c;
		Vec3 rpy;
		p = sqrt(pow(R[0][0], 2) + pow(R[0][1], 2));
		a = atan2(-R[1][2], R[2][2]);
		b = atan2(R[0][2], p);
		c = atan2(-R[0][1], R[0][0]);
		rpy.set(0, a); rpy.set(1, b); rpy.set(2, c);
		return rpy;
	}

	Vector PDController2::controlLaw(const State& s, Vector desPosition) const {

		double t = s.getTime();
		double t0 = getinitTime();
		double tf = getfinalTime();

		// computes the actual position and orientation for the leg in ground frame
		Vector toepointerPose = framePoseinG(s);

		// computes the actual linear and angular velocity of the leg in ground frame
		Vector toepointerVel = frameVelocityinG(s);
		Matrix KPp(3, 3), KVp(3, 3), KPo(3, 3), KVo(3, 3);

		KPp.setToZero(); KPo.setToZero();
		KVp.setToZero(); KVo.setToZero();
		KPp.updDiag() = kp; KPo.updDiag() = kp;
		KVp.updDiag() = kv; KVo.updDiag() = kv;

		Vector actPp(3), actPo(3);
		actPp[0] = toepointerPose[3]; actPp[1] = toepointerPose[4]; actPp[2] = toepointerPose[5];
		actPo[0] = toepointerPose[0]; actPo[1] = toepointerPose[1]; actPo[2] = toepointerPose[2];

		Vector actVp(3), actVo(3);
		actVp[0] = toepointerVel[3]; actVp[1] = toepointerVel[4]; actVp[2] = toepointerVel[5];
		actVo[0] = toepointerVel[0]; actVo[1] = toepointerVel[1]; actVo[2] = toepointerVel[2];


		// computes the desired trajectories -- pos, vel, acc
		Vector desPp(3), desPo(3);
		Vector desP = desPositionTraj(toepointerPose, desPosition, t0, tf, t);
		desPp[0] = desP[3]; desPp[1] = desP[4]; desPp[2] = desP[5];
		desPo[0] = desP[0]; desPo[1] = desP[1]; desPo[2] = desP[2];

		Vector desVp(3), desVo(3);
		Vector desV = desVelocityTraj(toepointerPose, desPosition, t0, tf, t);
		desVp[0] = desV[3]; desVp[1] = desV[4]; desVp[2] = desV[5];
		desVo[0] = desV[0]; desVo[1] = desV[1]; desVo[2] = desV[2];

		Vector desAp(3), desAo(3);
		Vector desA = desAccTraj(toepointerPose, desPosition, t0, tf, t);
		desAp[0] = desA[3]; desAp[1] = desA[4]; desAp[2] = desA[5];
		desAo[0] = desA[0]; desAo[1] = desA[1]; desAo[2] = desA[2];


		// computes the position control law
		Vector ap = desAp + KPp * (desPp - actPp) + KVp * (desVp - actVp);

		// computes the orientation control law
		Vector ao = T(actPo) * (desAo + KVo * (desVo - actVo) + KPo * (desPo - actPo)) + Tdot(actPo, actVo) * actVo;

	//	Vector ax = -KP*(toepointerPose - desPosition) - KV*(toepointerVel);

		// merges both control laws together into a 6x1 vector:
		Vector a(6);
		a[0] = ao[0]; a[1] = ao[1]; a[2] = ao[2];
		a[3] = ap[0]; a[4] = ap[1]; a[5] = ap[2];

		return a;
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

	Vector PDController2::impedanceControl(const State& s, Vector desPosition) const {
		
		Vector alpha = J66Inv(s)*(controlLaw(s, desPosition) - JDotU6(s));

		Vector tau = Mass66(s)*alpha + C61(s) - G61(s);

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

	void PDController2::setinitTime(double t0) {
		initTime = t0;
	}

	void PDController2::setfinalTime(double tf) {
		finalTime = tf;
	}

	double PDController2::getinitTime() const {
		return initTime;
	}

	double PDController2::getfinalTime() const {
		return finalTime;
	}

	Vec3 PDController2::getdesPosition() const {
		return desPos;
	}

	//_____________________________________________________________________________
	void PDController2::computeControls(const SimTK::State& s, SimTK::Vector &controls) const
	{
		double t = s.getTime();

		//-----------Testing zone----------------------------------------------------------------------------

		//		std::cout << "BodySet is = " << _model->getBodySet() << std::endl;
		//		std::cout << "CoordinateSet is = " << _model->getCoordinateSet() << std::endl;
		//		std::cout << J66(s) << std::endl;
		//		std::cout << J66Inv(s) << std::endl;

		//		Vector pc(6, Real(0)), pd(6, Real(0));
		//		pd[0] = 2; pd[1] = -2; pd[2] = 3; pd[3] = 1; pd[4] = 10; pd[5] = 0;

		//		Vector val = desPositionTraj(pc, pd, 0.5);
		//		std::cout << val << std::endl;
		//		std::cout <<"controlLaw is = " << controlLaw(s, pd) << std::endl;
		//		Vector val2 = desVelocityTraj(pc, pd, 0.5);
		//		std::cout << val2 << std::endl;
		//		Vector val3 = desAccTraj(pc, pd, 0.5);
		//		std::cout << val3 << std::endl;

				
		//		std::cout << frameVelocityinG(s) << std::endl;

		//		std::cout << "Time is: " << t << "  " << locationOfStationinG(s, 4) << std::endl;
		//		std::cout << locationOfStationinG(s, 8) << std::endl;
		//		std::cout << locationOfStationinG(s, 14) << std::endl;
		//		std::cout << JT(s, 8) << std::endl;


		//		const Coordinate& GP = _model->updCoordinateSet().get("G_Pedal_tilt");
		//		const Coordinate& joint2 = _model->getCoordinateSet().get("joint_2");

		//		double j1 = GP.setValue(s, 10 * Pi/180);
		//		double j1_u = joint1.getSpeedValue(s);
		//		double j2 = joint2.getValue(s);
		//		double j2_u = joint2.getSpeedValue(s);

		//---------------------------------------------------------------------------------------------------

		Vector controlTorque;

		Vector contForce = getContactForces();
		// Vec3 desPosition = getdesPosition();
		Vector desPosition(6, Real(0));

		/*// desired pose for arbitrary location
		desPosition[0] = -0.3561;
		desPosition[1] = -0.5327;
		desPosition[2] = 1.1653;
		desPosition[3] = 0.59044;
		desPosition[4] = -0.088;
		desPosition[5] = 0.4037;		*/

		// desired pose for brake pedal location
		desPosition[0] = -0.3561;
		desPosition[1] = -0.5327;
		desPosition[2] = 1.1653;
		desPosition[3] = 0.8903;
		desPosition[4] = -0.17127;
		desPosition[5] = 0.01528;

		std::cout << "Pose in ground = " << framePoseinG(s) << std::endl;
		std::cout << "Error in position = " << framePoseinG(s) - desPosition << std::endl;

		double t0 = getinitTime();
		double tf = getfinalTime();


		controlTorque = impedanceControl(s, desPosition);

		/*if (vecNorm(contForce) > 0)
			controlTorque = stiffnessControl(s, desPosition, 25000, 25000, 1000, 500);
		else
			controlTorque = stiffnessControl(s, desPosition, 5000, 5000, 1000, 300);*/

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
		Model osimModel("Driver_Leg-20171019_6DOF_PedalModified.osim");

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

		/*
		const Coordinate& hipflex = osimModel.getCoordinateSet().get("hip_flexion_r");
		const Coordinate& hipadd = osimModel.getCoordinateSet().get("hip_adduction_r");
		const Coordinate& hiprot = osimModel.getCoordinateSet().get("hip_rotation_r");
		const Coordinate& kneeflex = osimModel.getCoordinateSet().get("knee_angle_r");
		const Coordinate& ankleangle = osimModel.getCoordinateSet().get("ankle_angle_r");
		const Coordinate& subtalar = osimModel.getCoordinateSet().get("subtalar_angle_r");

		hipflex.setValue(s, 115 * Pi / 180);
		hipadd.setValue(s, -30 * Pi / 180);
		hiprot.setValue(s, 5 * Pi / 180);
		kneeflex.setValue(s, -100 * Pi / 180);
		ankleangle.setValue(s, 30 * Pi / 180);
		subtalar.setValue(s, 0 * Pi / 180);

		*/

	//	std::cout << pedalPositioninG(s, "B_Pedal", 0, osimModel) << std::endl;

		std::cout << "System Initialized" << std::endl;

		std::cout << "getMultibodySystem Done" << std::endl;


		SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
		integrator.setAccuracy(1.0e-3);
		integrator.setMinimumStepSize(5e-7);
		Manager manager(osimModel, integrator);

		// Define the initial and final simulation times
		double initialTime = 0.0;
		double finalTime = 2;


		controller->setinitTime(initialTime);
		controller->setfinalTime(finalTime);

		//Vec3 A(0.845, -0.1968, 0.1788);
		//Vec3 B(0.872, -0.2188, 0.1788);
		//Vec3 C(0.887, -0.228, 0.1788);
		//Vec3 D(0.8903, -0.1712, 0.01528);
		//Vec3 E(0.9226, -0.1661, 0.01528);
		//Vec3 F(0.6857, -0.3832, 0.1332);

		Vector contForce(6, Real(0));

		//for (int j = 0; j < 4; j++)
		//{
		//	double System_frequency = 1000.0;
		//	double System_timestep = 1.0 / System_frequency;
		//	int System_StartTim = j;
		//	double System_End_Time = 1.00;
		//	int System_Num_step = 2000;

		//	for (int i = 0; i < System_Num_step; i++)
		//	{
		//		// Define the initial and final simulation times
		//		initialTime = System_StartTim * 2 + System_timestep * i;
		//		if (i == System_Num_step - 1)
		//			finalTime = System_End_Time;
		//		else
		//			finalTime = initialTime + System_timestep;

		//		controller->setContactForces(contForce);

		//		switch (j)
		//		{
		//		case 0:
		//			controller->setdesPosition(A);
		//			break;
		//		case 1:
		//			controller->setdesPosition(B);
		//			break;
		//		case 2:
		//			controller->setdesPosition(D);
		//			break;
		//		case 3:
		//			controller->setdesPosition(F);
		//			break;
		//		}

		//		// Integrate from initial time to final time
		//		manager.setInitialTime(initialTime);
		//		manager.setFinalTime(finalTime);
		//		std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
		//		manager.integrate(s);
		//		std::cout << " Cycle [ " << i + 1 << " / " << System_Num_step << " ] has done! \n" << std::endl;

		//		const Storage& forcestorage = aForceReporter->getForceStorage();
		//		contForce = force(forcestorage, finalTime);

		//		std::cout << contForce << std::endl;

		//	}
		//}
		
		// Integrate from initial time to final time
		manager.setInitialTime(initialTime);
		manager.setFinalTime(finalTime);
		std::cout << "\n\nIntegrating from " << initialTime << " to " << finalTime << std::endl;
		manager.integrate(s);
		// std::cout << " Cycle [ " << i + 1 << " / " << System_Num_step << " ] has done! \n" << std::endl;

		const Storage& forcestorage = aForceReporter->getForceStorage();
		contForce = force(forcestorage, finalTime);

		std::cout << contForce << std::endl;

		// Save the simulation results
		Storage statesDegrees(manager.getStateStorage());
		statesDegrees.print("./Results/driverleg-states_toBrake2.sto");
		osimModel.updSimbodyEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		osimModel.printControlStorage("./Results/driverleg-controls_toBrake2.sto");

		aForceReporter->printResults("ForceReport_toBrake2", "./Results/ForceData", 0.005, ".sto");
		aBodyKinematics->printResults("BodyKinematics_toBrake2", "./Results/KinematicsData", 0.005, ".sto");


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