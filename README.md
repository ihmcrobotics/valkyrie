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
4. Shortly after the install completed, the desktop application should be available in the application menu as `Valkyrie Obstacle Course` (`Super` key, then
   start looking up for Valkyrie).
5. If you want to run the simulation from the command line, you can add the following line to your `~/.bashrc`
   file: `export PATH=$PATH:/opt/valkyrie-simulation-[version]/bin`.
    - You can then run from the command line with: `ValkyrieObstacleCourseSCS2`

To uninstall, run: `sudo dpkg -r valkyrie-simulation`

# Further information and documentation

This is a work in progress, we'll be adding more documentation and tutorials soon.