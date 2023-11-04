package frc.robot.subsystems.lift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;


public class Elevator extends SubsystemBase{
    private final WPI_TalonFX mMotor;
    private final PIDController mPID;
    private double targetHight = Constants.ElevatorConstants.ZERO.cone; 

    public Elevator(){
        mMotor = new WPI_TalonFX(Constants.ElevatorConstants.kElevatorCanID);
        mPID = new PIDController(1d/40d,0,0);
        

        configureMotor();
    };

    private void configureMotor(){
        mMotor.configFactoryDefault();
        mMotor.setNeutralMode(NeutralMode.Brake);
        mMotor.configVoltageCompSaturation(10);
        mMotor.enableVoltageCompensation(true);
        mMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 25, 0));
    };

    public boolean atSetpoint() {

        double tolerance = 0.5; 

        return (Math.abs(getHight() - targetHight) < tolerance);
    }


    public void runPivot(){
        mMotor.set(MathUtil.clamp(mPID.calculate(getHight(), targetHight), -0.5, 0.5));
    }

    private double getHight(){
        return mMotor.getSelectedSensorPosition()*Constants.ElevatorConstants.kFalconToHight + Constants.ElevatorConstants.kStartingHight;
    };

    public void changeSetPoint(double hight){
        double clampedHight = MathUtil.clamp(hight, Constants.ElevatorConstants.kStartingHight, Constants.ElevatorConstants.LEVEL3.cone);
        targetHight = clampedHight;
    }
}