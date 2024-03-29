{
    # Ports for motors
    # If doing drive test, treat this as the left side of the drivetrain
    "motorPorts": [0],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightMotorPorts": [1],
    # Class names of motor controllers used.
    # Options:
    # 'Spark'
    # 'Victor'
    # 'VictorSP'
    # 'PWMTalonSRX'
    # 'PWMVictorSPX'
    # If doing drive test, treat this as the left side of the drivetrain
    "controllerTypes": [Spark],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightControllerTypes": [Spark],
    # Set motors to inverted or not
    # If doing drive test, treat this as the left side of the drivetrain
    "motorsInverted": [False],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightMotorsInverted": [False],
    # Encoder edges-per-revolution (*NOT* cycles per revolution!)
    # For the AMT 103-V, use 8192 (2048 * 4)
    "encoderEPR": 5760,
    # Gearing accounts for the gearing between the encoder and the output shaft
    "gearing": 120,
    # Encoder ports
    # If doing drive test, treat this as the left side of the drivetrain
    "encoderPorts": [4, 5],
    # Only if you are doing drive (leave empty "[]" if not)
    "rightEncoderPorts": [6, 7],
    # Set to True if encoders need to be inverted
    # If doing drive test, treat this as the left side of the drivetrain
    "encoderInverted": False,
    # Only if you are doing drive (set to False if not needed)
    "rightEncoderInverted": False,
    # ** The following is only if you are using a gyro for the DriveTrain test**
    # Gyro type (one of "NavX", "Pigeon", "ADXRS450", "AnalogGyro", or "None")
    "gyroType": "None",
    # Whatever you put into the constructor of your gyro
    # Could be:
    # "SPI.Port.kMXP" (MXP SPI port for NavX or ADXRS450),
    # "SerialPort.Port.kMXP" (MXP Serial port for NavX),
    # "I2C.Port.kOnboard" (Onboard I2C port for NavX),
    # "0" (Pigeon CAN ID or AnalogGyro channel),
    # "new WPI_TalonSRX(3)" (Pigeon on a Talon SRX),
    # "" (NavX using default SPI, ADXRS450 using onboard CS0, or no gyro)
    "gyroPort": "",
}

