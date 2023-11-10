package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoint;
import frc.robot.Constants.ElevatorConstants.GamePiece;


public class Elevator extends SubsystemBase{

    //Delares the elevator motor
    private final WPI_TalonFX mMotor;
    private final TalonFXSimCollection mMotorSim;
    private final ElevatorSim mElevatorSim;
    //Declares the PID controller
    private final PIDController mPID = ElevatorConstants.kElevatorPID;
    
    private ElevatorSetpoint mSetpoint = ElevatorConstants.ZERO;
    private GamePiece mGamePiece = ElevatorConstants.CONE;

    /***
     * Elevator Subsystem
     */
    public Elevator(){
        mMotor = new WPI_TalonFX(ElevatorConstants.kElevatorCanID); //Initializes the motor
        mMotorSim = mMotor.getSimCollection(); //Initializes the motor simulator

        //Initializes the motor simulator
        mElevatorSim = new ElevatorSim(
            DCMotor.getFalcon500(1),
            12d,
            4,
            Units.inchesToMeters(1.751*Math.sin(Units.degreesToRadians(50))),
            Units.inchesToMeters(0),
            Units.inchesToMeters(54),
            false
        );

        configureMotor(); //Configures the motor
    };

    /***
     * Configures the motor
     */
    private void configureMotor(){
        mMotor.configFactoryDefault(); //Resets the motor to factory defaults
        mMotor.setInverted(true);
        mMotor.setNeutralMode(NeutralMode.Brake); //Sets the motor to brake mode
        mMotor.configVoltageCompSaturation(10); //Sets the voltage compensation to 10 volts to compensate for battery voltage drop
        mMotor.enableVoltageCompensation(true); //Enables the voltage compensation
        //Sets the current limit to 25 amps
        mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
    };

    /***
     * Checks if the elevator is at the setpoint
     * 
     * @return true if the elevator is at its setpoint
     */
    public boolean atSetpoint() {
        double heightTolerance = 0.5; 
        return (Math.abs(getHeight() - getPieceSetpoint()) < heightTolerance);
    }

    public double getPieceSetpoint(){
        return (mGamePiece.isCone) ? mSetpoint.cone : mSetpoint.cube;
    }

    // public void setSpeed(double speed){
    //     mMotor.set(speed);
    // }

    /***
     * Runs the elevator control loop
     */
    public void run(){
        double clampedSetpoint = MathUtil.clamp(getPieceSetpoint(), ElevatorConstants.kStartingHeight, ElevatorConstants.LEVEL3.cone);
        mMotor.set(MathUtil.clamp(mPID.calculate(getHeight(), clampedSetpoint), -0.75/2, 0.75/2));
    }

    /**
     * @return the height of the elevator in inches from the ground
     */
    private double getHeight(){
        return mMotor.getSelectedSensorPosition()*ElevatorConstants.kFalconToHeight + ElevatorConstants.kStartingHeight;
    };

    /**
     * Changes the setpoint of the elevator
     * @param height
     */
    public void changeSetpoint(ElevatorSetpoint setpoint){
        mSetpoint = setpoint;
    }

    public void changeGamePiece(GamePiece selected) {
        mGamePiece = selected;
    }

    public GamePiece getGamePiece(){
        return mGamePiece;
    }

    public ElevatorSetpoint getElevatorSetpoint(){
        return mSetpoint;
    }
    private ShuffleboardTab tab = Shuffleboard.getTab("Driver UI");

    private GenericEntry heightLog = tab.add("Elevator Height", 0)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();

    private GenericEntry setpointLog = tab.add("Elevator Setpoint Value", 0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();

    private GenericEntry setpointName = tab.add("Setpoint", "n/a")
        .withPosition(0,1)
        .withSize(2, 1)
        .getEntry();

    private GenericEntry atSetpointLog = tab.add("Elevator At Setpoint", false)
        .withPosition(0,2)
        .withSize(2, 1)
        .getEntry();
        
    private GenericEntry gamePieceLog = tab.add("Game Piece", false)
        .withPosition(2, 0)
        .withSize(3, 3)
        .getEntry();

    public void log(){
        heightLog.setDouble(getHeight());
        setpointLog.setDouble(getPieceSetpoint());
        setpointName.setString(mSetpoint.name);
        atSetpointLog.setBoolean(atSetpoint());
        gamePieceLog.setBoolean(mGamePiece.isCone);
    }

    public void simulate(){
        mElevatorSim.setInput(mMotor.get() * 12);
        mElevatorSim.update(0.02);
        mMotorSim.setIntegratedSensorRawPosition((int)(Units.metersToInches(mElevatorSim.getPositionMeters()) * 1/ElevatorConstants.kFalconToHeight));
    
        double height = Units.inchesToMeters((getHeight()-9.6));
        double motionRatio = Math.sin(Units.degreesToRadians(40))/ Math.sin(Units.degreesToRadians(50));

        // x,y,z w_rot, x_rot, y_rot, z_rot
        // No rotation, no y translation
        double elevatorStageLocations[] = {
            0, 0, 0, 0, 0, 0, 0,
            (height*motionRatio)/4, 0, height/4, 0, 0, 0, 0,
            (height*motionRatio)/2, 0, height/2, 0, 0, 0, 0,
        };

        SmartDashboard.putNumberArray("ElevatorPoses", elevatorStageLocations);
    
    }

}