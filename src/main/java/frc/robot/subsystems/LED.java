// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;
import frc.robot.trobot5013lib.led.IntensityPattern;
import frc.robot.trobot5013lib.led.ChaosPattern;
import frc.robot.trobot5013lib.led.ScannerPattern;
import frc.robot.trobot5013lib.led.AlternatingColorPattern;

public class LED extends SubsystemBase {
	// private TrobotAddressableLED m_led1 = new TrobotAddressableLED(Constants.LEDConstants.LED1_PWM_PORT, Constants.LEDConstants.LED1_LENGTH);
	//private TrobotAddressableLED m_led2 = new TrobotAddressableLED(Constants.LEDConstants.LED2_PWM_PORT, Constants.LEDConstants.LED2_LENGTH);
	private TrobotAddressableLED m_led3 = new TrobotAddressableLED(Constants.LEDConstants.LED3_PWM_PORT, Constants.LEDConstants.LED3_LENGTH);

	private RobotContainer m_RobotContainer;
	private int intensityDegrees = 10;
    private Color[] redWhiteArray = {Color.kRed, Color.kWhite};
    private Color[] blueWhiteArray = {Color.kBlue, Color.kWhite};
	private Color[] blackOrangeArray = {Color.kBlack, Color.kOrange};
	private Color[] redWhiteBlueArray = {Color.kRed, Color.kWhite, Color.kBlue};

	private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
	private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);

	private TrobotAddressableLEDPattern m_alternatingColorPattern = new AlternatingColorPattern(redWhiteBlueArray);
	private TrobotAddressableLEDPattern m_blinkingRed = new BlinkingPattern(Color.kRed, 0.25);
	private TrobotAddressableLEDPattern m_blinkingGreen = new BlinkingPattern(Color.kGreen, 0.25);
	private TrobotAddressableLEDPattern m_chaosPattern = new ChaosPattern();
    private TrobotAddressableLEDPattern m_redChasePattern = new ChasePattern(redWhiteArray, 3);
    private TrobotAddressableLEDPattern m_blueChasePattern = new ChasePattern(blueWhiteArray, 3);
	private TrobotAddressableLEDPattern m_orangeChasePattern = new ChasePattern(blackOrangeArray, 3);
	private TrobotAddressableLEDPattern m_redWhiteBlueChasePattern = new ChasePattern(redWhiteBlueArray, 2);
	private IntensityPattern m_blueIntensityPattern = new IntensityPattern(Color.kBlue, intensityDegrees);
	private IntensityPattern m_redIntensityPattern = new IntensityPattern(Color.kRed, intensityDegrees);
	private TrobotAddressableLEDPattern m_rainbowPattern = new RainbowPattern();
	private TrobotAddressableLEDPattern m_scannerPattern = new ScannerPattern(Color.kChartreuse,Color.kDeepPink,2);
	private TrobotAddressableLEDPattern m_greenPattern = new SolidColorPattern(Color.kGreen);
	private TrobotAddressableLEDPattern m_yellowPattern = new SolidColorPattern(Color.kLightYellow);
	private TrobotAddressableLEDPattern m_purplePattern = new SolidColorPattern(Color.kPurple);
	
	private TrobotAddressableLEDPattern m_disabledPattern = m_scannerPattern;
	
	/** Creates a new StatusLED. */
	public LED(RobotContainer robotContainer) {
		super();
		m_RobotContainer = robotContainer;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		
		// set patterns to run based on various statuses of the robot via the m_RobotContainer

		if (m_RobotContainer.isDisabled()) {
			// m_led1.setPattern(m_disabledPattern);
			//m_led2.setPattern(m_disabledPattern);
			m_led3.setPattern(m_disabledPattern);
		} else if (m_RobotContainer.isRedAlliance()) {
			// m_led1.setPattern(m_redPattern);
			//m_led2.setPattern(m_redPattern);
			m_led3.setPattern(m_redPattern);
		} else {
			// m_led1.setPattern(m_bluePattern);
			//m_led2.setPattern(m_redPattern);
			m_led3.setPattern(m_redPattern);
		}

	}
}
