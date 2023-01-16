package frc.robot;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;


public class SwerveModule {
private CANSparkMax drivemotor;
private CANSparkMax turnmotor;
private RelativeEncoder driveencoder;
private RelativeEncoder turnencoder;
private final PIDController drivePidController = new PIDController(1, 0, 0);
private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

private final ProfiledPIDController turnPidController = 
    new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Constants.maxanglevelocity, Constants.maxangleacceleration));

public SwerveModule(int driveid, int turnid){
    drivemotor = new CANSparkMax(driveid, CANSparkMaxLowLevel.MotorType.kBrushless);
    turnmotor = new CANSparkMax(turnid, CANSparkMaxLowLevel.MotorType.kBrushless);   
    driveencoder = drivemotor.getEncoder();
    turnencoder = turnmotor.getEncoder();
    turnPidController.enableContinuousInput(-Math.PI, Math.PI);
}

public SwerveModuleState getstate(){
    return new SwerveModuleState(
        driveencoder.getVelocity(), new Rotation2d(turnencoder.getPosition())
    );
}
public SwerveModuleState getposition(){
    return new SwerveModuleState(
        driveencoder.getPosition(), new Rotation2d(turnencoder.getPosition())
    );
}
public void setdesiredstate(SwerveModuleState desired){
    SwerveModuleState state = SwerveModuleState.optimize(desired, new Rotation2d(turnencoder.getPosition()));
    final double driveOutput = drivePidController.calculate(driveencoder.getVelocity(), state.speedMetersPerSecond);
    final double turnOutput = turnPidController.calculate(turnencoder.getPosition(), state.angle.getRadians()); 
    
    final double driveFeed = driveFeedforward.calculate(state.speedMetersPerSecond);
    final double turnFeed = turnFeedforward.calculate(turnPidController.getSetpoint().velocity);

    drivemotor.setVoltage(driveOutput + driveFeed);
    turnmotor.setVoltage(turnOutput + turnFeed);
}


}