# RaidZero FRC 2025

4253 Raid Zero robot code for 2025 Reefscape season

## Subsystem details

### Climb

| Device Name | Device Type          | CAN ID | CAN Bus |
|-------------|----------------------|--------|---------|
| Climb Joint | TalonFX - Kraken     | 17     | RoboRIO |
| Climb Winch | TalonFX - Kraken     | 16     | RoboRIO |

### Limelight

##### Offsets table

| Limelight | IP           | X       | Y      | Z       | Roll | Pitch | Yaw    |
|-----------|--------------|---------|--------|---------|------|-------|--------|
| FL        | 10.42.53.11  | -0.2155 | 0.2216 | 0.2689  | 0.0  | 20.0  | -45.0  |
| FR        | 10.42.53.12  | 0.2155  | 0.2216 | 0.2689  | 0.0  | 20.0  | 45.0   |
| BR        | 10.42.53.13  | 0.2445  | 0.2216 | -0.2521 | 0.0  | 35.0  | -135.0 |
| BL        | 10.42.53.14  | -0.254  | 0.2286 | 0.1778  | 0.0  | 45.0  | 180.0  |

<details>
    <summary>Directions</summary>
    <ul>
        <li>X is left-right direction
            <ul>
            <li>Left is negative, right is positive</li>
            </ul>
        </li>
        <li>Y is up-down direction
            <ul>
                <li>Down is negative, up is positive</li>
            </ul>
        </li>
        <li>Z is forward-backward direction
            <ul>
                <li>Backward is negative, forward is positive</li>
            </ul>
        </li>
        <li>Counterclockwise is positive degrees</li>
    </ul>
</details>

<details>
    <summary>Notes</summary>
    <ul>
        <li>Forward of the bot is the scoring side</li>
        <li>Back of the bot contains the RoboRIO</li>
        <li>All units are in meters and degrees</li>
        <li>XYZ measurements are relative to the center of the bot in CAD</li>
    </ul>
</details>

### Swerve

| Device Name | Device Type         | CAN ID | CAN Bus                      |
|-------------|---------------------|--------|------------------------------|
| RotorFL     | TalonFX - Kraken    | 6      | CANivore - "CANdoAttitude"   |
| ThrottleFL  | TalonFX - Kraken    | 5      | CANivore - "CANdoAttitude"   |
| CANCoderFL  | CANCoder            | 3      | CANivore - "CANdoAttitude"   |
| RotorFR     | TalonFX - Kraken    | 4      | CANivore - "CANdoAttitude"   |
| ThrottleFR  | TalonFX - Kraken    | 3      | CANivore - "CANdoAttitude"   |
| CANCoderFR  | CANCoder            | 2      | CANivore - "CANdoAttitude"   |
| RotorBL     | TalonFX - Kraken    | 8      | CANivore - "CANdoAttitude"   |
| ThrottleBL  | TalonFX - Kraken    | 7      | CANivore - "CANdoAttitude"   |
| CANCoderBL  | CANCoder            | 4      | CANivore - "CANdoAttitude"   |
| RotorBR     | TalonFX - Kraken    | 2      | CANivore - "CANdoAttitude"   |
| ThrottleBR  | TalonFX - Kraken    | 1      | CANivore - "CANdoAttitude"   |
| CANCoderBR  | CANCoder            | 1      | CANivore - "CANdoAttitude"   |
| Pigeon      | Pigeon2 IMU         | 0      | CANivore - "CANdoAttitude"   |

### Telescoping arm

#### Arm

| Device Name                | Device Type       | CAN ID | CAN Bus |
|----------------------------|-------------------|--------|---------|
| Telescopingarm Joint       | TalonFX - Kraken  | 10     | RoboRIO |
| Telescopingarm Telescope   | TalonFX - Kraken  | 11     | RoboRIO |
| Joint CANCoder             | CANCoder          | 11     | RoboRIO |

#### Coral intake

| Device Name        | Device Type                    | CAN ID | CAN Bus |
|--------------------|--------------------------------|--------|---------|
| CoralIntake        | TalonFXS - Minion              | 12     | RoboRIO |
| CoralIntake follow | TalonFXS - Minion              | 13     | RoboRIO |
| LaserCAN top       | Grapple Robotics - LaserCAN    | 1      | RoboRIO |
| LaserCAN bottom    | Grapple Robotics - LaserCAN    | 0      | RoboRIO |

## Controls

### Driver

| Input Type      | Function                       | Tap/Hold |
|-----------------|--------------------------------|----------|
| Left Stick X    | Field relative Y speed         | N/A      |
| Left Stick Y    | Field relative X speed         | N/A      |
| Right Stick X   | Robot rotation speed           | N/A      |
| Left Bumper     | Extake coral                   | Hold     |
| Right Bumper    | Intake coral                   | Tap      |
| A               | Robot centric slowed driving   | Hold     |
| B               | OTF to closest station         | Hold     |
| X               | OTF to closest left reef       | Hold     |
| Y               | OTF to closest right reef      | Hold     |
| POV Right       | X swerve brake                 | Hold     |

### Operator

| Button ID  | Function                      | Tap/Hold |
|------------|-------------------------------|----------|
| 1          | Retract climb winch (Up)      | Hold     |
| 2          | Deploy climb                  | Tap      |
| 3          | Extend climb winch (Down)     | Hold     |
| 4          | Check L4 slam                 | Hold     |
| 5          | L4 Slam                       | Tap      |
| 6          | Unbound                       | N/A      |
| 7          | Arm L2                        | Hold     |
| 8          | Arm L3                        | Hold     |
| 9          | Arm L4                        | Hold     |
| 10         | Extake coral                  | Hold     |
| 11         | Intake coral                  | Tap      |
| 12         | Scooch coral upwards          | Tap      |
| 13         | Reset intake Y offset         | Tap      |
| 14         | Unbound                       | N/A      |
| 15         | Decrease intake Y offset      | Tap      |
| 16         | Increase intake Y offset      | Tap      |

## Libraries/APIs used

- [Phoenix 5](https://docs.ctre-phoenix.com/en/stable/)
- [Phoenix 6](https://pro.docs.ctr-electronics.com/en/latest/)
- [PathPlannerLib](https://pathplanner.dev/home.html)
- [LimelightLib](https://github.com/LimelightVision/limelightlib-wpijava)
- [ElasticLib](https://github.com/Gold872/elastic-dashboard)
- [libgrapplefrc](https://github.com/GrappleRobotics/libgrapplefrc)
