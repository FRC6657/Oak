package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class CommandFactory {
    
    private final Drivetrain drivetrain;

    /*
     * Constructor for CommandFactory
     * 
     * The command factory is used to create commands that combine multiple subsystems.
     * 
     * @param _drivetrain The drivetrain subsystem
     */
    public CommandFactory(Drivetrain _drivetrain){
        drivetrain = _drivetrain;
    }

    /**
     * Creates a command to drive the robot using the swerve drive during teleop.
`    * All inputs are (-1,1) and scaled to the max speed.
     * Output is field relative
     */
    public Command TeleopSwerve(DoubleSupplier _xInput, DoubleSupplier _yInput, DoubleSupplier _rInput){
        return new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(_xInput.getAsDouble(), DriverConstants.kDriveDeadband),
                -MathUtil.applyDeadband(_yInput.getAsDouble(), DriverConstants.kDriveDeadband),
                -MathUtil.applyDeadband(_rInput.getAsDouble(), DriverConstants.kDriveDeadband),
                true, 
                RobotBase.isReal()
            ),
            drivetrain
        );
    }

}
