// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
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
    mCurrentState = state;
  }

  public void run(){
    mMotor.set(mCurrentState.speed);
  }

  public void log(){
    SmartDashboard.putString("Intake/State", mCurrentState.name());
    SmartDashboard.putNumber("Intake/Motor Percent", mCurrentState.speed);
  }

}
