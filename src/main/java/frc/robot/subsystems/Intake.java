// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoint;
import frc.robot.Constants.ElevatorConstants.GamePiece;
import frc.robot.Constants.IntakeConstants.State;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX mMotor;
  private State mCurrentState;
 
  private ElevatorSetpoint mElevatorSetpoint = ElevatorConstants.ZERO;
  private GamePiece mGamePiece = ElevatorConstants.CONE;

  public Intake() {
    mMotor = new WPI_TalonFX(Constants.IntakeConstants.kIntakeCanID);
    configureMotor();
    mCurrentState = State.STOP;
  }

  private void configureMotor(){
    mMotor.configFactoryDefault();
    mMotor.setNeutralMode(NeutralMode.Brake);
    mMotor.configVoltageCompSaturation(10);
    mMotor.enableVoltageCompensation(true);
    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 10, 0));
  };

  public void changeState(State state, GamePiece piece, ElevatorSetpoint setpoint){
    
    if(state == IntakeConstants.State.RELEASE) {

      if(piece == ElevatorConstants.CUBE) {

        if(setpoint == ElevatorConstants.LEVEL1) {
          mCurrentState = Constants.IntakeConstants.State.RELEASE_CUBE_L1;
        } else if(setpoint == ElevatorConstants.LEVEL2) {
          mCurrentState = Constants.IntakeConstants.State.RELEASE_CUBE_L2;
        } else {
          mCurrentState = Constants.IntakeConstants.State.RELEASE_CUBE_L3;
        }

      } else {

        if(setpoint == ElevatorConstants.LEVEL1) {
          mCurrentState = Constants.IntakeConstants.State.RELEASE_CONE_L1;
        } else if(setpoint == ElevatorConstants.LEVEL2) {
          mCurrentState = Constants.IntakeConstants.State.RELEASE_CONE_L2;
        } else {
          mCurrentState = Constants.IntakeConstants.State.RELEASE_CONE_L3;
        }
      }

    } else if(state == Constants.IntakeConstants.State.GRAB) {
      mCurrentState = Constants.IntakeConstants.State.GRAB;
    } else {
      mCurrentState = Constants.IntakeConstants.State.IDLE;
    }

  }

  public void run(){

    mMotor.set(mCurrentState.speed);
  }

  public void log(){
    SmartDashboard.putString("Intake/State", mCurrentState.name());
    SmartDashboard.putNumber("Intake/Motor Percent", mCurrentState.speed);
  }

}
