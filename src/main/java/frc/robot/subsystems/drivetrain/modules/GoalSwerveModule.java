package frc.robot.subsystems.drivetrain.modules;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Ideal swerve module, useful for debugging */
public class GoalSwerveModule implements SwerveModule {

  private SwerveModuleState state = new SwerveModuleState();
  private double distance;

  @Override
  public SwerveModuleState getState() {
    return state;
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(distance, state.angle);
  }

  @Override
  public void setDesiredState(SwerveModuleState _desiredState) {
    state = SwerveModuleState.optimize(_desiredState, state.angle);
    distance += state.speedMetersPerSecond * 0.02;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return state;
  }

  @Override
  public void resetEncoders() {}

}