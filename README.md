![valkyrie](https://maven-badges.herokuapp.com/maven-central/us.ihmc/valkyrie/badge.svg?style=plastic)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-fast.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-controller-api.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-controller-api-2.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-humanoid-flat-ground.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-humanoid-obstacle.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-humanoid-push-recovery.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-humanoid-rough-terrain.yml/badge.svg)
![buildstatus](https://github.com/ihmcrobotics/valkyrie/actions/workflows/main-GradleCI-humanoid-toolbox.yml/badge.svg)

# Install standalone application on Ubuntu 22.04 (Recommended)

1. Install Java 17: `sudo apt install openjdk-17-jdk`
2. Download the latest release (`valkyrie-simulation-[version].deb`) from the [releases page](https://github.com/ihmcrobotics/valkyrie/releases).
3. Install the package:
    - Open your favorite terminal application (Ctrl+Alt+T)
    - `sudo dpkg -i valkyrie-simulation-[version].deb`
4. Shortly after the installation completed, the desktop application should be available in the application menu as `Valkyrie Obstacle Course` (`Super` key, then
   start looking up for Valkyrie).
5. If you want to run the simulation from the command line, you can add the following line to your `~/.bashrc`
   file: `export PATH=$PATH:/opt/valkyrie-simulation-[version]/bin`.
    - You can then run from the command line with: `ValkyrieObstacleCourseSCS2`

To uninstall, run: `sudo dpkg -r valkyrie-simulation`

# Configuring IHMC communication with ROS 2
The IHMC controller stack uses ROS 2 to communicate, i.e. to receive sensor data and send commands to the robot.
To configure the ROS 2 communication, you need to create the file `~/.ihmc/IHMCNetworkParameters.ini` with the following content:
```
RTPSDomainID=0
RTPSSubnet=192.168.1.0/24
```
- The `RTPSDomainID` is the ROS 2 domain ID (set to 0 in the example above).
Make it match the domain ID of your ROS 2 network.
If not specified, it will be set to a random value.
- The `RTPSSubnet` is the subnet of the network interface used to communicate with the robot.
This can be useful if you have multiple network interfaces on your computer and you want to force the IHMC controller stack to use a specific one.
In the above example, IHMC software will only be able to communicate with computers on the `192.168.1.x` subnet.
If you don't know what this is, you can remove this field from the configuration file.

# Further information and documentation

This is a work in progress, we'll be adding more documentation and tutorials soon.