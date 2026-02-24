# Bullseye Robotics 2026 – REBUILT℠ Presented by Haas

Welcome to FRC Team 4984’s home for the 2026 competition robot. This repo contains the full Java codebase used to drive, score, simulate, and deploy the *REBUILT℠* robot, including autonomous paths generated in PathPlanner, Limelight-assisted targeting logic, and subsystem code for the climber, launcher, and intake.

## 🧭 Project Snapshot

- **Frameworks:** WPILib 2026, GradleRIO 2026.2.1, Java 17.
- **Drivetrain:** Custom swerve using NavX (Studica) gyro, WPILib odometry, and PathPlanner holonomic controllers.
- **Vision:** Limelight pipelines for auto-aim and tag following.
- **Subsystems:** Intake, Launcher, Climber, and auxiliary camera thread wired through `RobotContainer`.
- **Autos:** Generated with PathPlanner GUI and deployed from `src/main/deploy/pathplanner`.

## 📁 Repository Layout

```
FRC-2026-Code-Imported/
├─ build.gradle              # GradleRIO configuration (Java 17)
├─ src
│  └─ main
│     ├─ java/frc/robot      # RobotContainer, subsystems, commands
│     ├─ deploy              # Files copied to /home/lvuser/deploy (PathPlanner, configs)
│     └─ deploy/pathplanner  # Autos/paths shared with the GUI
├─ vendordeps                # REV, CTRE Phoenix 5/6, PathPlanner, AdvantageKit, etc.
└─ gradlew(.bat)             # Wrapper to build/test/deploy without extra installs
```

## ⚙️ Prerequisites

- WPILib 2026 (VS Code extension pack or the full WPILib installer).
- Java 17 (bundled with WPILib 2026 distribution).
- GradleRIO tooling (already packaged via the `gradlew` wrapper).
- Vendor libraries listed in `vendordeps/` are auto-resolved when you open the project with WPILib VS Code.

## 🚀 Quick Start

1. **Clone the repo** and open the `FRC-2026-Code-Imported` folder in the WPILib-enabled VS Code.
2. **Set the team number** via `WPILib: Set Team Number` or edit `.wpilib/wpilib_preferences.json` so deploy targets the correct RoboRIO.
3. **Build locally:**
	```powershell
	.\gradlew build
	```
4. **Run unit tests:**
	```powershell
	.\gradlew test
	```
5. **Deploy to the robot (tethered/Wi‑Fi):**
	```powershell
	.\gradlew deploy
	```
6. **Start the Driver Station** and enable teleop/auto modes as usual.

## 🧪 Simulation & Debugging

- Launch the desktop simulator (with Driver Station + GUI widgets) using:
  ```powershell
  .\gradlew simulateJava
  ```
- `RobotContainer` publishes SendableChoosers for autos and field visualization (`Field2d`) so you can validate odometry in Shuffleboard or Glass.
- SmartDashboard keys include `Robot Heading`, `odometer X/Y`, and Limelight status, simplifying pit-side debugging.

## 🤖 Key Features

- **SwerveSubsystem:** Handles module state control, NavX heading, odometry, and PathPlanner auto configuration with mirrored red/blue alliance support.
- **Command-based architecture:** `RobotContainer` wires joystick bindings, default commands (`SwerveJoystickCmd`), and Limelight routines (auto aim & AprilTag follow).
- **PathPlanner integration:** Drop `.path`/`.auto` files into `src/main/deploy/pathplanner` and select them from the SmartDashboard chooser without code changes.
- **Extensible subsystems:** Intake, Launcher, and Climber stubs are in place with clear binding points for future tuning.
- **Backup artifacts:** The Gradle `jar` task embeds source and vendordep backups to aid at-event recovery.

## 🗺️ Working with PathPlanner

1. Open the PathPlanner GUI and point it at `src/main/deploy/pathplanner/`.
2. Edit or create autos/paths; the saved files are version-controlled.
3. Select the generated auto via the SmartDashboard “Auto Chooser” before a match.
4. If an auto has a defined starting pose, `AutoBuilder` will call `resetPose` automatically.

## 🤝 Contributing

- Create feature branches per subsystem/feature.
- Prefer adding or updating tests in `src/test/java` (JUnit 5) when you change behavior.
- Run `.\gradlew build` before opening a PR to ensure lint/tests pass.
- Document new commands or operator bindings directly in `RobotContainer.java` comments plus this README when relevant.

## 📜 License & Credits

- WPILib BSD license applies (see `WPILib-License.md`).
- Vendor libraries retain their own licenses via `vendordeps`.
- Shout-out to sponsors—especially Haas—and the Bullseye Robotics mentors & students maintaining this code.

Have questions or spot something we can improve? File an issue or reach out to the software lead on Slack/Discord. Let’s make 2026 our sharpest season yet.
