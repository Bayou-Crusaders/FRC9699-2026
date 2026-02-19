// Copyright (c) 2017-2018 FIRST. All Rights Reserved.
// Open Source Software - may be modified and shared by FRC teams. The code
// must be accompanied by the FIRST BSD license file in the root directory of
// the project. 

//@TODO: Create a Function that takes in current Velocity and Acceleration to return variable FeedForward Gains

package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.ClosedLoopSlot;

/**Class to interface a single jointed arm.*/
public class ArmSubsystem extends SubsystemBase {
    //Initilaze the variables
    private SparkMax m_armMotor;
    private SparkMaxConfig m_armConfig;
    private SparkClosedLoopController m_closedLoopController;
    private ArmFeedforward m_feedforward;
    private RelativeEncoder m_encoder;

    /**Creates a new Arm Subsystem */
    public ArmSubsystem() {
        //Init the motor
        m_armMotor = new SparkMax(OperatorConstants.karmId, MotorType.kBrushless);

        //Get the ClosedLoopController
        m_closedLoopController = m_armMotor.getClosedLoopController();

        // Create a feedforward object
        m_feedforward = new ArmFeedforward(
            OperatorConstants.kS,
            OperatorConstants.kG,
            OperatorConstants.kV,
            OperatorConstants.kA
        );
        
        //Get the Connected Encoder from the Neo
        m_encoder = m_armMotor.getEncoder();

        //Create a config object
        m_armConfig = new SparkMaxConfig();

        /**Use to change the conversion factor */
        m_armConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        /**Applies closed loop configs to the arm config */
        m_armConfig.closedLoop
            //This is the sensor (encoder) that is providing feedback (current position) to the loop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)

            //The 'p' term contributes to the control signal proportionally to the current position error. 
            //Increasing 'p' will make the arm move faster to the setpoint. If the arm oscillates around the setpoint, decrease 'p'
            .p(0.5) 

            //The 'i' term contributes to the control signal by driving the total accumulated error to zero.
            //Esentilly, keep it VERY SMALL (please)
            .i(0)

            //The 'd' term contributes to the control signal by driving the derivitive of the error to zero.
            //Increasing 'd' will make the arm smoothly track a moving setpoint, but could also make the arm unstable, so be careful.
            .d(0.001)

            //For more detail in tuning P, I, D, and FF view: 
            // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html

            //This is the minimum and maximum output the loop can give to the motor
            .outputRange(-1, 1);

        /**Applies MAXMotion configs to the arm config */
        m_armConfig.closedLoop.maxMotion
            .maxVelocity(500)
            .maxAcceleration(100)
            .allowedClosedLoopError(1);

        /**Apply the arm config to the motor */
        m_armMotor.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

        /**Use to go to an angle */
        public Command goToAngle(float targetDegrees) {
            return Commands.runOnce( //Runs this command once per call of function
                () -> {
                    m_closedLoopController.setReference(
                        Units.degreesToRotations(targetDegrees), //Convert given degrees into rotations
                        ControlType.kPosition, //Use position control
                        ClosedLoopSlot.kSlot0, //Use Slot0 configs
                        m_feedforward.calculate(Units.degreesToRadians(targetDegrees), m_encoder.getVelocity()) //This will add an Arm feed forward calculation to the reference, which is highly encoraged for an arm
                    );
                }, 
                this // Command requires this subsystem
            );
        }
        
        /**Periodic method provided by {@link SubsystemBase}, which is scheduled every scheduler loop */
        @Override
        public void periodic() {
            // Display encoder position and velocity
            SmartDashboard.putNumber("Position", m_encoder.getPosition());
            SmartDashboard.putNumber("Velocity", m_encoder.getVelocity());

            if (SmartDashboard.getBoolean("Reset Encoder", false)) {
                // Reset the encoder position to 0
                m_encoder.setPosition(0);

                SmartDashboard.putBoolean("Reset Encoder", false);
                
            }
        }
    }