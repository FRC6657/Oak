// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.State;

public class Intake extends SubsystemBase {
  private final WPI_TalonFX mMotor;
  private State mCurrentState;

  public Intake() {
    mMotor = new WPI_TalonFX(Constants.IntakeConstants.kIntakeCanID);
    configureMotor();
    mCurrentState = State.STARTING;
  }

  private void configureMotor(){
    mMotor.configFactoryDefault();
    mMotor.setNeutralMode(NeutralMode.Brake);
    mMotor.configVoltageCompSaturation(10);
    mMotor.enableVoltageCompensation(true);
    mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
};

public Command changeState(State state){
  return new InstantCommand(() -> mCurrentState = state);
}

public void runIntake(){
  mMotor.set(mCurrentState.speed);
}

}
