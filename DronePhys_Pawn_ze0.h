// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <EngineGlobals.h>
#include <Runtime/Engine/Classes/Engine/Engine.h>

#include "CoreMinimal.h"
#include "Components/StaticMeshComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"

/*DEBUG*/
#include "Misc/FileHelper.h"
/*******/

#include "DronePhys_Pawn_ze0.generated.h"

//#define DO_DEBUG
#define DEBUG_ARRAY_LEN 300

UCLASS()
class DRONELAKE001_API ADronePhys_Pawn_ze0 : public APawn
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere)
		UStaticMeshComponent* m_DroneMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RCInput")
		float mI_RstickP;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RCInput")
		float mI_RstickR;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RCInput")
		float mI_LstickT;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "RCInput")
		float mI_LstickY;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InitConfig")
		FVector m_RespawnLoc;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InitConfig")
		float m_gravAccel;/*in cm/s^2 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InitConfig")
		float m_air_density_Kg_per_cubicmeter;/*Note: This is in m^3 NOT cm^3*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InitConfig")
		float m_prop_diameter_cm;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InitConfig")
		float m_max_motor_force_N;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "InitConfig")
		FVector m_max_angular_accel;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_tick;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_num_clicksTo_target;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_max_angle;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_max_yaw_velocity;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_yaw_max_secondary_rot;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_yaw_deadzone;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "InitControl")
		float m_control_throttle_max_range;

	// Sets default values for this pawn's properties
	ADronePhys_Pawn_ze0();

	virtual void PostInitProperties();
protected:
	float m_PI;

	float m_mass;
	FVector m_I;
	float m_F_grav;/*force gravity; will be negative*/

	/*constant in front of ForceMotor equation that transitions from exit velocity to F for a given airspeed*/
	float m_C_F;/*multiplied by 4 so that it coverts to total motor force*/
	/* maximum exit velocity motors can generate at max thrust*/
	float m_Ve_max;
	float m_Ve_max2; //m_Ve_max^2 


	/*set per tick*/
	float m_F_motors;/*net CM force from all four motors*/
	/* unit vectors of the drone body coordinate system in
	   the world coordinate system.
	   The handedness of this system is left alone
	*/
	FVector m_body_Ux;
	FVector m_body_Uy;
	FVector m_body_Uz;

	/*current anglular velocity of the drone in the world frame*/
	FVector m_dRot_world;
	/*current anglular velocity of the drone in its frame*/
	FVector m_dRot_drone;
	/*current linear veloicty of the drone in the world/drone frame*/
	FVector m_Velocity;

	FVector m_Alpha;/*acceleration in radians per sec*/
	FVector m_torque;/*alpha times inertia*/

	/********************************************************************/
	/* Initialization */

	void calculate_ForceMotorConstants();
	void SetConstants();
	void SetMass();
	void SetupObject();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	/********************************************************************/
	/* State (Orient/Velocity) functions that are called prior to the 
		physics and controll ticks*/
	void UpdateAllRotDependent(FBodyInstance* BodyInstance);
	/** helpers to UpdateAllRotDependent**/
	void setBodyCoordUvecs(FBodyInstance* BodyInstance);


	/********************************************************************/
	/* Physics Functions typically called every physics tick */
	/*run the actual physics during the tick*/
	void UpdatePhysicsTick(float DeltaTime, FBodyInstance* BodyInstance);

	/** Helpers to UpdatePhysicsTick **/
	/*functions that update frame dependent coordinat system*/
	void UpdateMotorForce();
	/**helpers to Update motor force**/
	float LimitForceMotor(FVector& motorVelocity);
	/**                             **/
	void UpdateAlpha();

	void ApplyForces(FBodyInstance* BodyInstance, float DeltaTime);

	/*utility functions*/
	/** utility function for orientation & angular velocity **/
	FVector rotToBodyFrame(FVector& vec);/*converts vec in world coordinates to vector in body coordinates*/
	FVector rotToWorldFrame(FVector& vec);/*converts a vector in body coordinates to a vector in world coordinates*/
	FVector bodyXYToWorldXYAligned(FVector& vec);/*rotates vec which is a vector in a frame with z axis along world but x,y axis along drone frame
												   from this frame to the world frame,
												   in the event that the alignment is ambiguous then x is taken to be in the z drone axis direction*/
	FVector bodyXYToWorldXYZ(float bodyX, float bodyY);
	FVector worldXYZToUnitHorizXY(FVector& worldXYZ);/*takes a vector that is in world coordinates and find the unit projection along the horizontal plane, return unit X=1 if proj not possible*/


	/********************************************************************/
	/* Control Section */
	float m_minHorizProjSize;
	float m_control_timeTo_target;/*how long the target accel is setup for*/

	float m_invdt;
	float m_invdt2;
	float m_omegaFactor;

	/*control input variables*/
	/*set at begining*/
	float m_control_throttle_center;/*typically set to balance gravity*/
	float m_control_throttle_scale;/*scale factor that translate control input into throttle response*/
	float m_control_throttle_min;
	float m_control_throttle_max;

	/****** Updated each tick *****/
	/*current control configuration set dynamically each control tick*/
	float m_control_pitch_angle;
	float m_control_roll_angle;
	float m_control_yaw_velocity;

	/*flag that temporay stops fixing rotation to a given x horizontal vector*/
	bool m_target_fixed_Ux;

	/*target Uz position for drone in world frame*/
	FVector m_target_Uz_world;
	/*target for front  of drone in world frame*/
	FVector m_target_Ux_horiz_world;
	/*diffs in angle between the current orientation and the target orientation*/
	FVector m_target_dAng_drone;
	/*target anglular velocity of the drone in its frame*/
	FVector m_target_ddRot_drone;

	/*target angular acceleration*/
	FVector m_targetAlpha;

	/*current throttle target force   */
	float m_target_F_throttle;

	/******                   *****/
	/****** Init of Control   *****/
	/*SetConstants() & SetMassInertia() must be called first*/
	/*sets the inital constants and zeros dynamic variables*/
	void InitControl();
	/*helpers to InitControl*/
	void SetInitialThrust();
	void SetInitControlConstants(
		float min_control_tick = 0.002f,
		float minHorizProjSize = 0.05f,
		float control_max_angle = 3.f,
		float control_yaw_min_max_secondary_rot = 2.f,
		float control_max_yaw_deadzone = 0.5f,
		float control_max_throttle_range = 0.99f
	);
	/*****                    *****/

	void UpdateControlTick();
	/** Helpers to UpdateControlTick **/

	/*sets the m_target vectors from the mI_ values passed by the level blueprint*/
	void SetTargetsFromControlInput();
	/*helpers to SetTargets...*/
	void SetTargetThrottleFromControlInput();
	void SetTargetTiltFromControlInput();
	void SetTargetYawFromControlInput();

	/* Update the target torques for tilt*/
	/* uses equation for deltat accel deltat decell to zero velocity
	   alpha1 = (theta/dt^2 - 3/2*omega/dt)
	   alpha1 = - (alpha2 + omega/2)
	   this is based on comver theta distance by accelerating at alpha1 for dt then decelerating at alpha2 for dt where the inital velocity was omega
	   Since this equation is reapplied not swaped to change to alpha2 the "swap" is limit on overage of alpha1 or alpha2 at which point
	   target alpha is reset to just stop
	   */
	void UpdateTargetAlpha();
	/**helper to UpdateTargetAlpha**/
	void UpdateTargetTiltAlpha();
	void UpdateTargetYawAlpha();
	/***helper to Updates ****/
	float scaleToOppositeMaxAccel(float alpha, float alpha2, float alpha_max, float omega);

#ifdef DO_DEBUG
	double m_DEBUG_last_tick_time;
	double m_DEBUG_start_time;
	double m_DEBUG_tick;
	bool  m_DEBUG_do_tick;
	int   m_DEBUG_array_i;
	int   m_DEBUG_max_i;
	bool  m_DEBUG_logged;

	void initDEBUG();
	void tickDEBUG(float deltaTime);
	void recordTickDEBUG(float deltaTime);
	void endDEBUG();

	float m_DEBUG_curtime[DEBUG_ARRAY_LEN]; /*curtime*/
	float m_DEBUG_deltatime[DEBUG_ARRAY_LEN]; /*delta time*/
	float m_DEBUG_body_Uz[DEBUG_ARRAY_LEN][3]; /*Uz x, Uz y, Uz z*/
	float m_DEBUG_body_Ux[DEBUG_ARRAY_LEN][3]; /*Ux x, Ux y, Ux z*/
	float m_DEBUG_target_Uz[DEBUG_ARRAY_LEN][3];/* targUz x, targUz y, targUz z*/
	float m_DEBUG_target_Ux[DEBUG_ARRAY_LEN][3];/* targUx x, targUx y, targUx z*/

	float m_DEBUG_target_dAng_drone[DEBUG_ARRAY_LEN][3]; /*dAng x, dAng y, dAng z*/
	float m_DEBUG_dRot_drone[DEBUG_ARRAY_LEN][3]; /* dRot x, dRot y, dRot z*/
	float m_DEBUG_targetAlpha[DEBUG_ARRAY_LEN][3]; /* targAlph x, targAlph y, targAlph z*/
	float m_DEBUG_Alpha[DEBUG_ARRAY_LEN][3];/*alph x, alph y, alph z*/

	float m_DEBUG_targetF[DEBUG_ARRAY_LEN];/* targ F*/
	float m_DEBUG_limF[DEBUG_ARRAY_LEN];/* lim F */
	float m_DEBUG_F[DEBUG_ARRAY_LEN];/* motorF */

	float m_DEBUG_control_yaw_velocity[DEBUG_ARRAY_LEN];
	float m_DEBUG_yaw_pause[DEBUG_ARRAY_LEN];
#endif
public:
	/*fixed physics tick*/
	FCalculateCustomPhysics OnCalculateCustomPhysics; //Custom physics delegate
	void CustomPhysics(float DeltaTime, FBodyInstance* BodyInstance);

	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	void Handle_Respawn();
	void Handle_Pitch_R(float AxisValue) { mI_RstickP = AxisValue; }
	void Handle_Roll_R(float AxisValue) { mI_RstickR = AxisValue; }
	void Handle_Yaw_L(float AxisValue) { mI_LstickY = AxisValue;}
	void Handle_Thrust_L(float AxisValue) { mI_LstickT = AxisValue; }
};
