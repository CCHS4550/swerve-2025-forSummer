package frc.robot.subsystems.drive;

import java.util.concurrent.locks.ReentrantLock;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.module.*;
import frc.robot.subsystems.drive.module.Module;

import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysId;
    private final Alert gyroDCAlert = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.driveConstants.moduleTranslations);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePos = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePos, new Pose2d());

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO){
            this.gyroIO = gyroIO;
            modules[0] = new Module(flModuleIO, 0);
            modules[1] = new Module(frModuleIO, 1);
            modules[2] = new Module(flModuleIO, 2);
            modules[3] = new Module(frModuleIO, 3);

            //Usage reporting
            HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

            OdometryThread.getInstance().start();

            //AutoBuilder.configure(null, null, null, null, null, null, null, null);
            sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
      }
      @Override
      public void periodic(){
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules){
            module.periodic();
        }
        odometryLock.unlock();

        if(DriverStation.isDisabled()){
            for(var module: modules){
                module.stop();
            }
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;
        for(int i = 0; i< sampleCount; i++){
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for(int moduleIndex = 0; moduleIndex < 4; moduleIndex++){
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePos[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
                lastModulePos[moduleIndex] = modulePositions[moduleIndex];
            }    
            if(gyroInputs.connected){
                rawGyroRotation = gyroInputs.yawOdometryPos[i];
            } 
            else {
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        gyroDCAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
      }

      public void runVelocity(ChassisSpeeds speeds){
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setPointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setPointStates, Constants.driveConstants.maxSpeed);

        Logger.recordOutput("SwerveStates/Setpoints", setPointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        for(int i = 0; i < 4; i++){
            modules[i].runSetPoint(setPointStates[i]);
        }

        Logger.recordOutput("SwerveStates/SetpointsOptimized", setPointStates);
      }

      public void runCharacterization(Double output){
        for(int i = 0; i < 4; i++){
            modules[i].runCharacterization(output);
        }
      }
      public void stop(){
        runVelocity(new ChassisSpeeds());
      }

      public void StopWithX(){
        Rotation2d [] headings = new Rotation2d[4];
        for(int i = 0; i < 4; i++){
            headings[i] = Constants.driveConstants.moduleTranslations[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
      }

      public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
      }
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
      }

      @AutoLogOutput(key = "SwerveStates/measured")
      private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = modules[i].getState();
        }
        return states;
      }
      private SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            states[i] = modules[i].getPosition();
        }
        return states;
      }
      @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
        private ChassisSpeeds getChassisSpeeds(){
            return kinematics.toChassisSpeeds(getModuleStates());
        }

      public double[] getWheelRadiusCharacterizationPosition(){
        double[] values = new double [4];
        for(int i = 0; i < 4; i++){
           values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
      }
      
      public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++){
            output += modules [i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
      }

      @AutoLogOutput(key = "Odometry/Robot")
      public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
      }

      public Rotation2d getRotation(){
        return getPose().getRotation();
      }
      public void setPose(Pose2d pose){
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
      }
      // leave this blank for now because I dont want to deal with vision rn
      public void addVisionMeasurement(){}

      public double getMaxLinearSpeed(){ //meters per sec
        return Constants.driveConstants.maxSpeed;
      }

      public double getMaxAngularSpeed(){ //radians per sec
        return Constants.driveConstants.maxSpeed / Constants.driveConstants.driveBaseRadius;
      }
    }
