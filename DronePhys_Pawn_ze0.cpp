// Fill out your copyright notice in the Description page of Project Settings.


#include "DronePhys_Pawn_ze0.h"

// Sets default values
ADronePhys_Pawn_ze0::ADronePhys_Pawn_ze0()
{
 	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	m_RespawnLoc.X = 0.f;
	m_RespawnLoc.Y = 0.f;
	m_RespawnLoc.Z = 0.f;

	// Set this pawn to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	/*fixed physics tick*/
	OnCalculateCustomPhysics.BindUObject(this, &ADronePhys_Pawn_ze0::CustomPhysics);

	m_DroneMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("RootComponent"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> staticMeshRootAsset(TEXT("/Game/Inertia_model"));
	if (staticMeshRootAsset.Succeeded()) {
		m_DroneMesh->SetStaticMesh(staticMeshRootAsset.Object);
		m_DroneMesh->SetWorldScale3D(FVector(0.1));/*prev 0.05, prev 0.06*/
	}
	SetupObject();
	AutoPossessPlayer = EAutoReceiveInput::Player0;
}
void ADronePhys_Pawn_ze0::PostInitProperties()
{
	Super::PostInitProperties();
	mI_RstickP = 0.f;
	mI_RstickR = 0.f;
	mI_LstickT = 0.f;
	mI_LstickY = 0.f;
}

void ADronePhys_Pawn_ze0::calculate_ForceMotorConstants()
{
	m_PI = 3.14159265359f;
	float propDiameterMeters = m_prop_diameter_cm / 100.f;
	float AreaProp = propDiameterMeters * propDiameterMeters / 4.f;
	AreaProp *= m_PI;
	float rhoA = 0.5 * m_air_density_Kg_per_cubicmeter * AreaProp;
	float Ve2max = m_max_motor_force_N / rhoA;
	m_C_F = rhoA;
	m_C_F /= (100.f*100.f); //take into account that Ve will be given in cm per second instead of meters per second
	m_C_F *= (100.f); //convert  m -> cm force is in Kg*cm/s^2 instead of newtons
	m_C_F *= 4.f; //set this up for total motor force instead of individual / simplifed model
	m_Ve_max = sqrt(Ve2max); //this should be around 25 meters per second or 55mph
	m_Ve_max *= 100.f; //convert from meters per second to cm per second
	m_Ve_max2 = m_Ve_max * m_Ve_max;
#ifdef DO_DEBUG
	UE_LOG(LogTemp, Log, TEXT("C_F: %f,  Ve_max: %f,  Ve_max^2:%f"), m_C_F, m_Ve_max, m_Ve_max2);
#endif
}
void ADronePhys_Pawn_ze0::SetConstants()
{
	if (m_gravAccel < 0.f)
		m_gravAccel = 981.f;
}
void ADronePhys_Pawn_ze0::SetMass() {
	m_mass = m_DroneMesh->GetMass();/*mass is in kg*/
	m_I = m_DroneMesh->GetInertiaTensor();/*inertia is in kg*cm^2*/
	m_F_grav = -1.f*m_mass * m_gravAccel;/*force of gravity on drone, will be negative*/
#ifdef DO_DEBUG
	UE_LOG(LogTemp, Log, TEXT("mass: %f"), m_mass);
	UE_LOG(LogTemp, Log, TEXT("g, F_grav:%f, %f "), m_gravAccel, m_F_grav);
#endif
}
void ADronePhys_Pawn_ze0::SetupObject()
{
	RootComponent = (USceneComponent*)m_DroneMesh;
	m_DroneMesh->SetCollisionProfileName(TEXT("BlockAll"));
}
// Called when the game starts or when spawned
void ADronePhys_Pawn_ze0::BeginPlay()
{
	Super::BeginPlay();

	m_DroneMesh->SetSimulatePhysics(true);
	m_DroneMesh->SetEnableGravity(false);
	m_DroneMesh->SetVisibility(false);

	SetConstants();
	calculate_ForceMotorConstants();
	SetMass();
	InitControl();
#ifdef DO_DEBUG
	initDEBUG();
#endif
}
void ADronePhys_Pawn_ze0::UpdateAllRotDependent(FBodyInstance* BodyInstance) {
	setBodyCoordUvecs(BodyInstance);
	m_dRot_world = BodyInstance->GetUnrealWorldAngularVelocityInRadians();
	m_Velocity = BodyInstance->GetUnrealWorldVelocity();
}
void ADronePhys_Pawn_ze0::setBodyCoordUvecs(FBodyInstance* BodyInstance)
{
	FTransform body_transform = BodyInstance->GetUnrealWorldTransform();
	FRotator body_rotator = body_transform.GetRotation().Rotator();
	FVector x_vec(1.f, 0.f, 0.f);
	FVector y_vec(0.f, 1.f, 0.f);
	FVector z_vec(0.f, 0.f, 1.f);
	m_body_Ux = body_rotator.RotateVector(x_vec);
	m_body_Uy = body_rotator.RotateVector(y_vec);
	m_body_Uz = body_rotator.RotateVector(z_vec);
}
void ADronePhys_Pawn_ze0::UpdatePhysicsTick(float DeltaTime, FBodyInstance* BodyInstance)
{
	UpdateMotorForce();
	UpdateAlpha();
	ApplyForces(BodyInstance, DeltaTime);
}


void ADronePhys_Pawn_ze0::UpdateMotorForce()
{
	float F_max = LimitForceMotor(m_Velocity);
	m_F_motors = FMath::Clamp(m_target_F_throttle, 0.f, F_max);
#ifdef DO_DEBUG
	m_DEBUG_limF[m_DEBUG_array_i] = F_max;
#endif
}
float ADronePhys_Pawn_ze0::LimitForceMotor(FVector& motorVelocity)
{
	/*right now only take into account velocities perpendicular to the motor plane*/
/*this means that only the componet of velocity perp to the drone's z axis will count*/
	float motorAirspeed = FVector::DotProduct(m_body_Uz, motorVelocity);
	float force;
	/*get the maximum allowed force for this motor airspeed*/
	if (motorAirspeed >= 0.f) {
		force = m_C_F * (m_Ve_max2 - motorAirspeed * motorAirspeed);
	}
	else {
		force = (m_C_F)*(m_Ve_max2 + motorAirspeed * motorAirspeed);/*this may or may not be correct*/
	}
	return force;/*since m_C_F has been multiplied by 4 this is actually the force for all 4 motors*/
}
void ADronePhys_Pawn_ze0::UpdateAlpha()
{
	m_Alpha.X = FMath::Clamp(m_targetAlpha.X, -m_max_angular_accel.X, m_max_angular_accel.X);
	m_Alpha.Y = FMath::Clamp(m_targetAlpha.Y, -m_max_angular_accel.Y, m_max_angular_accel.Y);
	m_Alpha.Z = FMath::Clamp(m_targetAlpha.Z, -m_max_angular_accel.Z, m_max_angular_accel.Z);
}
void ADronePhys_Pawn_ze0::ApplyForces(FBodyInstance* BodyInstance, float DeltaTime)
{
	/*apply force from gravity*/
	FVector F_grav(0.0f, 0.0f, m_F_grav);
	BodyInstance->AddForce(F_grav, false, false);
	/*apply force from motors*/
	FVector CM_Pos(0.f, 0.f, 0.f);
	FVector F_motors(0.f, 0.f, m_F_motors);
	BodyInstance->AddForceAtPosition(F_motors, CM_Pos, false, true);/*substepping is false, is local force true*/
	/*apply torque as accel from motors*/
	//BodyInstance->ClearTorques(false);
	FVector aveVelocity_drone = m_Alpha / 2.f;
	aveVelocity_drone *= DeltaTime;
	aveVelocity_drone += m_dRot_drone;
	FVector aveVelocity = rotToWorldFrame(aveVelocity_drone);
	BodyInstance->SetAngularVelocityInRadians(aveVelocity,false);
	//BodyInstance->AddTorqueInRadians(m_Alpha, false, true);/*substepping false, accel change true(means inertial mass WILL NOT be taken into account)*/
}
FVector ADronePhys_Pawn_ze0::rotToBodyFrame(FVector& vec)
{
	/*must call setBodyCoordUvecs first*/
	FVector retVec(0.f, 0.f, 0.f);
	retVec.X = FVector::DotProduct(m_body_Ux, vec);
	retVec.Y = FVector::DotProduct(m_body_Uy, vec);
	retVec.Z = FVector::DotProduct(m_body_Uz, vec);
	return retVec;
}
FVector ADronePhys_Pawn_ze0::rotToWorldFrame(FVector& vec)
{
	/*must call setBodyCoordUvecs first*/
	FVector retVec = vec.X*m_body_Ux + vec.Y*m_body_Uy + vec.Z*m_body_Uz;
	return retVec;
}
FVector ADronePhys_Pawn_ze0::bodyXYToWorldXYAligned(FVector& vec)
{
	/*must call setBodyCoorddUvecs first*/

	float len_body_x = m_body_Ux.X*m_body_Ux.X + m_body_Ux.Y*m_body_Ux.Y;
	len_body_x = sqrtf(len_body_x);
	float len_body_y = m_body_Uy.X*m_body_Uy.X + m_body_Uy.Y*m_body_Uy.Y;
	len_body_y = sqrtf(len_body_y);
	float len_vec = vec.Size();
	FVector retVec(0.f, 0.f, 1.f);
	float min_len = 0.001f;
	float costheta = 0.0f;
	float sintheta = 1.0f;
	if (len_body_x < min_len) {
		if (len_body_y < min_len)
			return retVec;/*this shouldn't ever happen*/
		costheta = m_body_Uy.Y / len_body_y;
		sintheta = m_body_Uy.X / len_body_y;
	}
	else {
		costheta = m_body_Ux.X / len_body_x;
		sintheta = -m_body_Ux.Y / len_body_x;
	}
	retVec.X = costheta * vec.X + sintheta * vec.Y;
	retVec.Y = -sintheta * vec.X + costheta * vec.Y;
	retVec.Z = vec.Z;
	return retVec;
}
FVector ADronePhys_Pawn_ze0::bodyXYToWorldXYZ(float bodyX, float bodyY)
{
	FVector Xcomp = m_body_Ux * bodyX;
	FVector Ycomp = m_body_Uy * bodyY;
	Xcomp += Ycomp;
	return Xcomp;
}
FVector ADronePhys_Pawn_ze0::worldXYZToUnitHorizXY(FVector& worldXYZ)
{
	float len = sqrt(worldXYZ.X*worldXYZ.X + worldXYZ.Y*worldXYZ.Y);
	FVector Uhoriz(1.f, 0.f, 0.f);
	if (len < m_minHorizProjSize) {
		return Uhoriz;
	}
	Uhoriz.X = worldXYZ.X / len;
	Uhoriz.Y = worldXYZ.Y / len;
	return Uhoriz;
}

void ADronePhys_Pawn_ze0::InitControl()
{
	SetInitControlConstants();
	m_target_Uz_world.X = 0.f;
	m_target_Uz_world.Y = 0.f;
	m_target_Uz_world.Z = 1.f;
	m_target_Ux_horiz_world.X = 1.f;
	m_target_Ux_horiz_world.Y = 0.f;
	m_target_Ux_horiz_world.Z = 0.f;
	/*zero values for starters*/
	m_control_pitch_angle = 0.f;
	m_control_roll_angle = 0.f;
	m_control_yaw_velocity = 0.0f;
	m_target_fixed_Ux = false;/*try to get the program to find the current drone direction and set it into the Ux_horiz_world*/
	m_targetAlpha *= 0.f;
#ifdef DO_DEBUG
	UE_LOG(LogTemp, Log, TEXT("target_F_throttle: %f"), m_target_F_throttle);
#endif
}
void ADronePhys_Pawn_ze0::SetInitialThrust()
{
	m_target_F_throttle = -m_F_grav;
}
void ADronePhys_Pawn_ze0::SetInitControlConstants(
	float min_control_tick,
	float minHorizProjSize,
	float control_max_angle,
	float control_yaw_min_max_secondary_rot,
	float control_max_yaw_deadzone,
	float control_max_throttle_range
)
{
	m_minHorizProjSize = minHorizProjSize;
	if (m_control_tick < min_control_tick)
		m_control_tick = min_control_tick;
	if (m_control_num_clicksTo_target < 1)
		m_control_num_clicksTo_target = 1;
	m_control_timeTo_target = m_control_num_clicksTo_target * m_control_tick;
	if (m_control_max_angle > control_max_angle)
		m_control_max_angle = control_max_angle;
	if (m_control_yaw_max_secondary_rot < control_yaw_min_max_secondary_rot)
		m_control_yaw_max_secondary_rot = control_yaw_min_max_secondary_rot;
	if (m_control_yaw_deadzone < 0.f)
		m_control_yaw_deadzone = 0.f;
	if (m_control_yaw_deadzone > control_max_yaw_deadzone)
		m_control_yaw_deadzone = control_max_yaw_deadzone;
	if (m_control_throttle_max_range < 0.f)
		m_control_throttle_max_range = 0.f;
	if (m_control_throttle_max_range > control_max_throttle_range)
		m_control_throttle_max_range = control_max_throttle_range;
	m_invdt = 1 / m_control_timeTo_target;
	m_invdt2 = m_invdt * m_invdt;
	/*
		theta - 3/2*dt*omega_i = alpha_1*dt^2
		(alpha_1 + alpha_2)*dt = -omega_i
		this is the equation for the routine where max accel towards target is applied for dt then
		opposite accel is applied for dt with the final result being omega=0, where intial omega is omega_i
		alpha = angular acceleration, omega= angular velocity, theta = angle to be transversed 
	  */
	m_omegaFactor = -1.5f * m_control_timeTo_target; /*  - 3/2*dt   */


	SetInitialThrust();/*sets initial m_target_F_throttle*/
	m_control_throttle_center = (m_target_F_throttle > 0.f) ? sqrt(m_target_F_throttle) : 0.f;/*target_F_throttle should be already calculated relative to gravity*/
	float max_control_throttle = (m_Ve_max*sqrt(m_C_F) - m_control_throttle_center) * m_control_throttle_max_range + m_control_throttle_center;
	float min_control_throttle = m_control_throttle_center - (m_control_throttle_center*m_control_throttle_max_range);
	if (max_control_throttle < 0)
		max_control_throttle = 0;
	if (min_control_throttle < 0)
		min_control_throttle = 0;
	m_control_throttle_max = max_control_throttle;
	m_control_throttle_min = min_control_throttle;
	float max_control_throttle_diff = m_control_throttle_max - m_control_throttle_center;
	m_control_throttle_scale = max_control_throttle_diff;//only using the scale up to set throttle down may under or overshoot

	if (m_control_max_yaw_velocity > m_max_angular_accel.Z)
		m_control_max_yaw_velocity = m_max_angular_accel.Z;
#ifdef DO_DEBUG
	UE_LOG(LogTemp, Log, TEXT("max yaw velocity: %f, max secondary rot: %f"), m_control_max_yaw_velocity, m_control_yaw_max_secondary_rot);
#endif

}
void ADronePhys_Pawn_ze0::UpdateControlTick()
{
	SetTargetsFromControlInput(); /* translates input values to target pitch/roll, yaw values
								 sets target_Uz, target_Ux or yaw_velocity */
	UpdateTargetAlpha();
}
void ADronePhys_Pawn_ze0::SetTargetsFromControlInput()
{
	SetTargetThrottleFromControlInput();
	SetTargetTiltFromControlInput();
	SetTargetYawFromControlInput();
}
void ADronePhys_Pawn_ze0::SetTargetThrottleFromControlInput()
{
	float scaled_target_Ve = m_control_throttle_scale * mI_LstickT + m_control_throttle_center;
	float clamped_value = FMath::Clamp(scaled_target_Ve, m_control_throttle_min, m_control_throttle_max);
	m_target_F_throttle = clamped_value * clamped_value;
}
void ADronePhys_Pawn_ze0::SetTargetTiltFromControlInput()
{
	m_control_pitch_angle = m_control_max_angle * mI_RstickP;
	m_control_roll_angle = m_control_max_angle * mI_RstickR;

	FVector RotY;
	RotY.X = sin(m_control_pitch_angle);
	RotY.Y = 0.f;
	RotY.Z = cos(m_control_pitch_angle);

	/*assume right is negative*/
	FVector RotX;
	RotX.X = 0.f;
	RotX.Y = sin(m_control_roll_angle);
	RotX.Z = cos(m_control_roll_angle);


	FVector u_top = FVector::CrossProduct(RotY, RotX);
	float mag_top = u_top.Size();
	FVector u_tilt(0.f, 0.f, 1.f);

	if (mag_top > 0.001f) {
		u_top.X /= mag_top;
		u_top.Y /= mag_top;
		u_top.Z /= mag_top;
		/*find ratios of Z of tilt to side of tilt
		  using u_top (dot) u_tilt = 0  */
		float X = RotY.X;
		float Y = RotX.Y;
		if (abs(RotY.X) > 0.001f && abs(RotX.Y) > 0.001f && abs(u_top.Z) > 0.001f) {
			u_tilt.X = X;
			u_tilt.Y = Y;
			u_tilt.Z = -(u_top.X*X + u_top.Y*Y) / (u_top.Z);
		}
		else {
			if (abs(X) <= abs(Y)) {
				u_tilt.X = RotX.X;
				u_tilt.Y = RotX.Y;
				u_tilt.Z = RotX.Z;
			}
			else {
				u_tilt.X = RotY.X;
				u_tilt.Y = RotY.Y;
				u_tilt.Z = RotY.Z;
			}
		}
	}

	/*normalize since the above is probably not normalized*/
	float mag_u_tilt = u_tilt.Size();
	if (mag_u_tilt <= 0.0001f)/*shouldn't happen*/
	{
		u_tilt.X = 0.f;
		u_tilt.Y = 0.f;
		u_tilt.Z = 1.f;
	}
	else {
		u_tilt.X /= mag_u_tilt;
		u_tilt.Y /= mag_u_tilt;
		u_tilt.Z /= mag_u_tilt;
	}
	/*u_tilt is currently in a frame oriented along the drone's x/y axis direction the real control target must
	  be rotated around the z axis*/
	m_target_Uz_world = bodyXYToWorldXYAligned(u_tilt);
}
void ADronePhys_Pawn_ze0::SetTargetYawFromControlInput()
{
	float abs_input = fabs(mI_LstickY);
	if (abs_input < m_control_yaw_deadzone) {
		m_target_fixed_Ux = true;
		m_target_Ux_horiz_world = worldXYZToUnitHorizXY(m_body_Ux);
	}
	else {
		m_target_fixed_Ux = false;
		m_control_yaw_velocity = (mI_LstickY*m_control_max_yaw_velocity);
	}
}
void ADronePhys_Pawn_ze0::UpdateTargetAlpha()
{
	UpdateTargetTiltAlpha();
	UpdateTargetYawAlpha();
}
void ADronePhys_Pawn_ze0::UpdateTargetTiltAlpha()
{
	FVector u_rot = FVector::CrossProduct(m_body_Uz, m_target_Uz_world);/*m_body_Uz is given in world coordinates*/
	float mag_u_rot = u_rot.Size();
	if (mag_u_rot <= 0.0000001) {
		FVector dummyZero(0.f, 0.f, 0.f);
		m_target_dAng_drone = dummyZero;
		m_target_ddRot_drone = dummyZero;
		m_targetAlpha = dummyZero;
		return;
	}
	u_rot /= mag_u_rot;
	FVector u_rot_drone = rotToBodyFrame(u_rot);/* conversion of the rotation vector to the drone body frame */
	/*assumes physic tick has already been run */
	m_dRot_drone = rotToBodyFrame(m_dRot_world);

	float dot_body_target = FVector::DotProduct(m_body_Uz, m_target_Uz_world);
	float AngVstarget = FMath::Acos(dot_body_target);

	m_target_dAng_drone.X = u_rot_drone.X*AngVstarget;
	m_target_dAng_drone.Y = u_rot_drone.Y*AngVstarget;
	m_target_dAng_drone.Z = 0.f;
	m_target_ddRot_drone = m_invdt2 * m_target_dAng_drone + m_omegaFactor * m_dRot_drone;
	FVector Alpha2 = -1.f*(m_target_ddRot_drone + m_invdt * m_dRot_drone);
	m_targetAlpha.X = scaleToOppositeMaxAccel(m_target_ddRot_drone.X, Alpha2.X, m_max_angular_accel.X, m_dRot_drone.X);
	m_targetAlpha.Y = scaleToOppositeMaxAccel(m_target_ddRot_drone.Y, Alpha2.Y, m_max_angular_accel.Y, m_dRot_drone.Y);

}
void ADronePhys_Pawn_ze0::UpdateTargetYawAlpha()
{
	/*find the yaw velocity*/
	FVector projHoriz(m_body_Ux.X, m_body_Ux.Y, 0.f);
	float projHorizSize = projHoriz.Size();
	if (projHorizSize > m_minHorizProjSize) {
		FVector yawVec = FVector::CrossProduct(projHoriz, m_target_Ux_horiz_world);
		float yawarcsin = yawVec.Size() / projHorizSize;/*m_target_Ux_horiz_world should be normalized to 1*/
		float AngVstarget = FMath::Asin(yawarcsin);
		m_target_dAng_drone.Z = (yawVec.Z >= 0.f) ? AngVstarget : -AngVstarget;
	}
	else {
		m_target_dAng_drone.Z = 0.f;
	}
	if (m_target_fixed_Ux) {
		m_target_ddRot_drone.Z = m_invdt2 * m_target_dAng_drone.Z + m_omegaFactor * m_dRot_drone.Z;
		float Alpha2 = -1.f*(m_target_ddRot_drone.Z + m_invdt * m_dRot_drone.Z);
		m_targetAlpha.Z = scaleToOppositeMaxAccel(m_target_ddRot_drone.Z, Alpha2, m_max_angular_accel.Z, m_dRot_drone.Z);
	}
	else
	{
		float secondaryRot = sqrtf(m_dRot_drone.X*m_dRot_drone.X + m_dRot_drone.Y*m_dRot_drone.Y);
		if (secondaryRot < m_control_yaw_max_secondary_rot)
			m_targetAlpha.Z = (m_control_yaw_velocity - m_dRot_drone.Z)*m_invdt;
		else
			m_targetAlpha.Z = 0.f;
	}
}
float ADronePhys_Pawn_ze0::scaleToOppositeMaxAccel(float alpha, float alpha2, float alpha_max, float omega)
{
	if ((fabs(alpha2) > 2.f*fabs(alpha)) || fabs(alpha) > alpha_max)
		return alpha2;
	return alpha;
}
void ADronePhys_Pawn_ze0::CustomPhysics(float DeltaTime, FBodyInstance* BodyInstance)
{
	FBodyInstance* cur_body_instance = m_DroneMesh->GetBodyInstance();/*this may be just replicating BodyInstance*/
	UpdateAllRotDependent(cur_body_instance);
	UpdateControlTick();
	UpdatePhysicsTick(DeltaTime, cur_body_instance);
#ifdef DO_DEBUG
	recordTickDEBUG(DeltaTime);
#endif
}
// Called every frame
void ADronePhys_Pawn_ze0::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	/*fixed physics tick*/
	if (m_DroneMesh->GetBodyInstance() != NULL) {
		m_DroneMesh->GetBodyInstance()->AddCustomPhysics(OnCalculateCustomPhysics);
	}

#ifdef DO_DEBUG
	tickDEBUG(DeltaTime);
#endif
}

// Called to bind functionality to input
void ADronePhys_Pawn_ze0::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	PlayerInputComponent->BindAction("RespawnGP", IE_Pressed, this, &ADronePhys_Pawn_ze0::Handle_Respawn);
	PlayerInputComponent->BindAxis("PitchGP", this, &ADronePhys_Pawn_ze0::Handle_Pitch_R);
	PlayerInputComponent->BindAxis("RollGP", this, &ADronePhys_Pawn_ze0::Handle_Roll_R);
	PlayerInputComponent->BindAxis("YawGP", this, &ADronePhys_Pawn_ze0::Handle_Yaw_L);
	PlayerInputComponent->BindAxis("ThrottleGP", this, &ADronePhys_Pawn_ze0::Handle_Thrust_L);
}
void ADronePhys_Pawn_ze0::Handle_Respawn()
{
	FRotator orient(0.f, 0.f, 0.f);
	FVector velocity(0.f, 0.f, 0.f);
	m_DroneMesh->SetAllPhysicsAngularVelocityInRadians(velocity);
	m_DroneMesh->SetAllPhysicsLinearVelocity(velocity);
	m_DroneMesh->SetWorldLocationAndRotation(m_RespawnLoc, orient);
}
#ifdef DO_DEBUG
void ADronePhys_Pawn_ze0::initDEBUG() {
	m_DEBUG_tick = 0.0001f;
	m_DEBUG_do_tick = true;
	m_DEBUG_array_i = 0;
	double time_now = GetWorld()->GetRealTimeSeconds();
	m_DEBUG_last_tick_time = time_now;
	m_DEBUG_start_time = time_now;
	m_DEBUG_max_i = DEBUG_ARRAY_LEN - 1;
	m_DEBUG_logged = false;
}
void ADronePhys_Pawn_ze0::tickDEBUG(float deltaTime)
{
	double time_now = GetWorld()->GetRealTimeSeconds();
	double dTime = time_now - m_DEBUG_last_tick_time;
	if (m_DEBUG_start_time == 0)
		m_DEBUG_start_time = time_now;
	double dctrlTime = time_now - m_DEBUG_start_time;
	/*
	mI_LstickY = 0.;
	if (dctrlTime < 0.2) {
		mI_LstickY = 0.f;
	}
	else if (dctrlTime < 1.2) {
		mI_LstickY = 0.1;
	}
	else
		mI_LstickY = 0.1;
		*/
}
void ADronePhys_Pawn_ze0::recordTickDEBUG(float deltaTime)
{
	double time_now = GetWorld()->GetRealTimeSeconds();
	double dTime = time_now - m_DEBUG_last_tick_time;
	if (dTime > m_DEBUG_tick && mI_LstickY>0.f) {
		m_DEBUG_last_tick_time = time_now;
		if (m_DEBUG_array_i < m_DEBUG_max_i)
			m_DEBUG_array_i++;
		else
			endDEBUG();

		m_DEBUG_curtime[m_DEBUG_array_i] = time_now;
		m_DEBUG_deltatime[m_DEBUG_array_i] = deltaTime;

		m_DEBUG_body_Uz[m_DEBUG_array_i][0] = m_body_Uz.X;
		m_DEBUG_body_Uz[m_DEBUG_array_i][1] = m_body_Uz.Y;
		m_DEBUG_body_Uz[m_DEBUG_array_i][2] = m_body_Uz.Z;
		m_DEBUG_body_Ux[m_DEBUG_array_i][0] = m_body_Ux.X;
		m_DEBUG_body_Ux[m_DEBUG_array_i][1] = m_body_Ux.Y;
		m_DEBUG_body_Ux[m_DEBUG_array_i][2] = m_body_Ux.Z;
		m_DEBUG_target_Uz[m_DEBUG_array_i][0] = m_target_Uz_world.X;
		m_DEBUG_target_Uz[m_DEBUG_array_i][1] = m_target_Uz_world.Y;
		m_DEBUG_target_Uz[m_DEBUG_array_i][2] = m_target_Uz_world.Z;
		m_DEBUG_target_Ux[m_DEBUG_array_i][0] = m_target_Ux_horiz_world.X;
		m_DEBUG_target_Ux[m_DEBUG_array_i][1] = m_target_Ux_horiz_world.Y;
		m_DEBUG_target_Ux[m_DEBUG_array_i][2] = m_target_Ux_horiz_world.Z;

		m_DEBUG_target_dAng_drone[m_DEBUG_array_i][0] = m_target_dAng_drone.X;
		m_DEBUG_target_dAng_drone[m_DEBUG_array_i][1] = m_target_dAng_drone.Y;
		m_DEBUG_target_dAng_drone[m_DEBUG_array_i][2] = m_target_dAng_drone.Z;
		m_DEBUG_dRot_drone[m_DEBUG_array_i][0] = m_dRot_drone.X;
		m_DEBUG_dRot_drone[m_DEBUG_array_i][1] = m_dRot_drone.Y;
		m_DEBUG_dRot_drone[m_DEBUG_array_i][2] = m_dRot_drone.Z;
		m_DEBUG_targetAlpha[m_DEBUG_array_i][0] = m_targetAlpha.X;
		m_DEBUG_targetAlpha[m_DEBUG_array_i][1] = m_targetAlpha.Y;
		m_DEBUG_targetAlpha[m_DEBUG_array_i][2] = m_targetAlpha.Z;
		m_DEBUG_Alpha[m_DEBUG_array_i][0] = m_Alpha.X;
		m_DEBUG_Alpha[m_DEBUG_array_i][1] = m_Alpha.Y;
		m_DEBUG_Alpha[m_DEBUG_array_i][2] = m_Alpha.Z;
		m_DEBUG_targetF[m_DEBUG_array_i] = m_target_F_throttle;
		m_DEBUG_F[m_DEBUG_array_i] = m_F_motors;

		m_DEBUG_control_yaw_velocity[m_DEBUG_array_i] = m_control_yaw_velocity;
		m_DEBUG_yaw_pause[m_DEBUG_array_i] = m_target_fixed_Ux;
	}
}
void ADronePhys_Pawn_ze0::endDEBUG()
{
	if (m_DEBUG_logged)
		return;
	m_DEBUG_logged = true;
	TArray<FString> arrOut;
	FString headerStr = FString::Printf(TEXT("time, dtime, Uz x, Uz y, Uz z, Ux x, Ux y, Ux z, targUz x, targUz y, targUz z, targUx x, targUx y, targUx z, dAng x, dAng y, dAng z, dRot x, dRot y, dRot z, targAlph x, targAlph y, targAlph z, alph x, alph y, alph z, targ F, Lim F, motorF, control yaw, fixed Ux"));
	arrOut.Add(headerStr);
	for (int i = 0; i < DEBUG_ARRAY_LEN; i++)
	{
		FString lineStr = FString::Printf(TEXT("%f, %f,   %f,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f,  %f,%f,%f, %f,%f,%f, %f,%f,%f, %f,%f,%f,  %f, %f, %f, %f, %f"),
			m_DEBUG_curtime[i],
			m_DEBUG_deltatime[i],
			m_DEBUG_body_Uz[i][0], m_DEBUG_body_Uz[i][1], m_DEBUG_body_Uz[i][2],
			m_DEBUG_body_Ux[i][0], m_DEBUG_body_Ux[i][1], m_DEBUG_body_Ux[i][2],
			m_DEBUG_target_Uz[i][0], m_DEBUG_target_Uz[i][1], m_DEBUG_target_Uz[i][2],
			m_DEBUG_target_Ux[i][0], m_DEBUG_target_Ux[i][1], m_DEBUG_target_Ux[i][2],
			m_DEBUG_target_dAng_drone[i][0], m_DEBUG_target_dAng_drone[i][1], m_DEBUG_target_dAng_drone[i][2],
			m_DEBUG_dRot_drone[i][0], m_DEBUG_dRot_drone[i][1], m_DEBUG_dRot_drone[i][2],
			m_DEBUG_targetAlpha[i][0], m_DEBUG_targetAlpha[i][1], m_DEBUG_targetAlpha[i][2],
			m_DEBUG_Alpha[i][0], m_DEBUG_Alpha[i][1], m_DEBUG_Alpha[i][2],
			m_DEBUG_targetF[i],
			m_DEBUG_limF[i],
			m_DEBUG_F[i],
			m_DEBUG_control_yaw_velocity[i],
			m_DEBUG_yaw_pause[i]);
		arrOut.Add(lineStr);
	}
	FString filePath = FPaths::ConvertRelativePathToFull(FPaths::GameSavedDir()) + TEXT("/KinOutZE0Log.txt");
	UE_LOG(LogTemp, Log, TEXT("logging kin to file: %s"), *filePath);
	FFileHelper::SaveStringArrayToFile(arrOut, *filePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), EFileWrite::FILEWRITE_Append);
}
#endif