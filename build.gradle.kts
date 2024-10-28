import org.apache.tools.ant.taskdefs.condition.Os
import us.ihmc.cd.LogTools

plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
   id("us.ihmc.scs") version "0.4"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

ihmc {
   group = "us.ihmc"
   version = "0.14.0-240126"
   vcsUrl = "https://github.com/ihmcrobotics/valkyrie"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

val ihmcOpenRoboticsSoftwareVersion = "source"

mainDependencies {
   api("com.martiansoftware:jsap:2.1")
   api("org.yaml:snakeyaml:1.17") //1.11
   api("org.ejml:ejml-core:0.39")
   api("org.ejml:ejml-simple:0.39")
   api("org.ejml:ejml-ddense:0.39")
   api("us.ihmc:jinput:2.0.6-ihmc2")
   api("us.ihmc:ihmc-lord-microstrain-drivers:17-0.0.7")

   api("us.ihmc:euclid:0.22.2")
   api("us.ihmc:euclid-geometry:0.22.2")
   api("us.ihmc:euclid-frame:0.22.2")
   api("us.ihmc:euclid-shape:0.22.2")
   api("us.ihmc:euclid-frame-shape:0.22.2")
   api("us.ihmc:ihmc-realtime:1.6.0")
   api("us.ihmc:ihmc-ros-control:0.7.1")

   api("us.ihmc:ihmc-system-identification:$ihmcOpenRoboticsSoftwareVersion")
   api("us.ihmc:ihmc-avatar-interfaces:$ihmcOpenRoboticsSoftwareVersion") {
      exclude(group = "us.ihmc", module = "javacpp")
   }
   api("us.ihmc:ihmc-footstep-planning-visualizers:$ihmcOpenRoboticsSoftwareVersion") {
      exclude(group = "us.ihmc", module = "javacpp")
   }
   api("us.ihmc:ihmc-parameter-tuner:0.15.0")

   // This is required to get the binaries for Linux.
   var javaFXVersion = "17.0.2"
   api(ihmc.javaFXModule("graphics", javaFXVersion))
}

testDependencies {
   api("us.ihmc:euclid:0.22.2")
   api("us.ihmc:euclid-geometry:0.22.2")
   api("us.ihmc:euclid-frame:0.22.2")
   api("us.ihmc:euclid-shape:0.22.2")
   api("us.ihmc:euclid-frame-shape:0.22.2")

   api("us.ihmc:ihmc-avatar-interfaces-test:$ihmcOpenRoboticsSoftwareVersion") {
      exclude(group = "us.ihmc", module = "javacpp")
   }
}

ihmc.jarWithLibFolder()
tasks.getByPath("installDist").dependsOn("compositeJar")
val installDistOutputFolder = "${project.projectDir}/build/install/valkyrie"

app.entrypoint("IHMCValkyrieJoystickApplication", "us.ihmc.valkyrie.joystick.ValkyrieJoystickBasedSteppingApplication")
app.entrypoint("valkyrie-network-processor", "us.ihmc.valkyrie.ValkyrieNetworkProcessor")
app.entrypoint("ValkyrieObstacleCourseNoUI", "us.ihmc.valkyrie.ValkyrieObstacleCourseNoUI")

tasks.create("deployOCUApplications") {
   dependsOn("installDist")

   doLast {
      val appFolder = File(System.getProperty("user.home"), "ihmc_apps/valkyrie")
      appFolder.delete()
      appFolder.mkdirs()
      copy {
         from(installDistOutputFolder)
         into(appFolder)
      }
      println("-------------------------------------------------------------------------")
      println("------- Deployed files to: " + appFolder.path + " -------")
      println("-------------------------------------------------------------------------")
   }
}

tasks.create("deployLocal") {
   dependsOn("installDist")

   doLast {
      val libFolder = File(System.getProperty("user.home"), "valkyrie/lib")
      libFolder.delete()
      libFolder.mkdirs()

      copy {
         from("$installDistOutputFolder/lib")
         into(libFolder)
      }

      val binFolder = File(System.getProperty("user.home"), "valkyrie/bin")
      binFolder.delete()
      binFolder.mkdirs()

      copy {
         from("$installDistOutputFolder/bin")
         into(binFolder)
      }

      copy {
         from("build/libs/valkyrie-$version.jar")
         into(File(System.getProperty("user.home"), "valkyrie"))
      }

      File(System.getProperty("user.home"), "valkyrie/valkyrie-$version.jar").renameTo(File(System.getProperty("user.home"), "valkyrie/ValkyrieController.jar"))

      val configurationDir = File(System.getProperty("user.home"), ".ihmc/Configurations")
      configurationDir.delete()
      configurationDir.mkdirs()

      copy {
         from("saved-configurations/defaultREAModuleConfiguration.txt")
         into(configurationDir)
      }
   }
}

val directory = "/home/val/valkyrie"

tasks.create("deploy") {
   dependsOn("installDist")

   doLast {
      val valkyrie_link_ip: String by project
      val valkyrie_realtime_username: String by project
      val valkyrie_realtime_password: String by project

      remote.session(valkyrie_link_ip, valkyrie_realtime_username, valkyrie_realtime_password) // control
      {
         exec("mkdir -p $directory")

         exec("rm -rf $directory/lib")
         put(file("$installDistOutputFolder/lib").toString(), "$directory/lib")
         exec("ls -halp $directory/lib")

         put(file("build/libs/valkyrie-$version.jar").toString(), "$directory/ValkyrieController.jar")
         put(file("launchScripts").toString(), directory)
         exec("chmod +x $directory/runNetworkProcessor.sh")
         exec("ls -halp $directory")
      }

      deployNetworkProcessor()
   }
}

tasks.create("deployNetworkProcessor") {
   dependsOn("installDist")

   doLast {
      deployNetworkProcessor()
   }
}

fun deployNetworkProcessor()
{
   val valkyrie_zelda_ip: String by project
   val valkyrie_bronn_ip: String? by project
   val valkyrie_realtime_username: String by project
   val valkyrie_realtime_password: String by project
   val local_bronn_ip = valkyrie_bronn_ip

   remote.session(valkyrie_zelda_ip, valkyrie_realtime_username, valkyrie_realtime_password) // perception
   {
      exec("mkdir -p $directory")

      exec("rm -rf $directory/bin")
      exec("rm -rf $directory/lib")

      put(file("$installDistOutputFolder/bin").toString(), "$directory/bin")
      exec("chmod +x $directory/bin/valkyrie-network-processor")
      put(file("$installDistOutputFolder/lib").toString(), "$directory/lib")
      exec("ls -halp $directory/lib")

      put(file("build/libs/valkyrie-$version.jar").toString(), "$directory/ValkyrieController.jar")
      put(file("launchScripts").toString(), directory)
      exec("chmod +x $directory/runNetworkProcessor.sh")
      exec("ls -halp $directory")

      exec("rm -rf /home/val/.ihmc/Configurations")
      exec("mkdir -p /home/val/.ihmc/Configurations")
      put(file("saved-configurations/defaultREAModuleConfiguration.txt").toString(), ".ihmc/Configurations")
      exec("ls -halp /home/val/.ihmc/Configurations")
   }

   if (local_bronn_ip != null)
   {
      remote.session(local_bronn_ip, valkyrie_realtime_username, valkyrie_realtime_password) // perception
      {
         exec("mkdir -p $directory")

         exec("rm -rf $directory/bin")
         exec("rm -rf $directory/lib")

         put(file("$installDistOutputFolder/bin").toString(), "$directory/bin")
         exec("chmod +x $directory/bin/valkyrie-network-processor")
         put(file("$installDistOutputFolder/lib").toString(), "$directory/lib")
         exec("ls -halp $directory/lib")

         put(file("build/libs/valkyrie-$version.jar").toString(), "$directory/ValkyrieController.jar")
         put(file("launchScripts").toString(), directory)
         exec("chmod +x $directory/runNetworkProcessor.sh")
         exec("ls -halp $directory")

         exec("rm -rf /home/val/.ihmc/Configurations")
         exec("mkdir -p /home/val/.ihmc/Configurations")
         put(file("saved-configurations/defaultREAModuleConfiguration.txt").toString(), ".ihmc/Configurations")
         exec("ls -halp /home/val/.ihmc/Configurations")
      }
   }
}

val debianName = "valkyrie-simulation-${ihmc.version}"
val simulationApplicationName = "ValkyrieObstacleCourseSCS2"
app.entrypoint(simulationApplicationName, "us.ihmc.valkyrie.ValkyrieObstacleCourseNoUISCS2", listOf("-Djdk.gtk.version=2", "-Dprism.vsync=false", "-Dprism.forceGPU=true"))

tasks.create("buildDebianSimulationPackage") {
   dependsOn("installDist")

   doLast {
      val deploymentFolder = "${project.projectDir}/deployment"

      val debianFolder = "$deploymentFolder/debian"
      File(debianFolder).deleteRecursively()

      val baseFolder = "$deploymentFolder/debian/$debianName/"
      val sourceFolder = "$baseFolder/opt/$debianName/"

      copy {
         from("${project.projectDir}/logo/scs-icon.png")
         into("$sourceFolder/icon/")
      }

      copy {
         from(installDistOutputFolder)
         into(sourceFolder)
      }

      fileTree("$sourceFolder/bin").matching {
         exclude(simulationApplicationName)
      }.forEach(File::delete)

      addJavaFXVsyncHack(File("$sourceFolder/bin/$simulationApplicationName"))

      File("$baseFolder/DEBIAN").mkdirs()
      LogTools.info("Created directory $baseFolder/DEBIAN/: ${File("${baseFolder}/DEBIAN").exists()}")

      File("$baseFolder/DEBIAN/control").writeText(
            """
         Package: valkyrie-simulation
         Version: ${ihmc.version}
         Section: base
         Architecture: all
         Depends: default-jre (>= 2:1.17) | java17-runtime
         Maintainer: IHMC Robotics
         Description: Simulation environments for Valkyrie
         Homepage: ${ihmc.vcsUrl}
         
         """.trimIndent()
      )

      File("$baseFolder/DEBIAN/postinst").writeText(
            """
         #!/bin/bash
         # Without this, the desktop file does not appear in the system menu.
         sudo desktop-file-install /usr/share/applications/$simulationApplicationName.desktop
         echo "-----------------------------------------------------------------------------------------"
         echo "---------------------------- Installation Notes: ----------------------------------------"
         echo "Add the following to your .bashrc to run simulations from the command line:"
         echo "   export PATH=\${'$'}PATH:/opt/$debianName/bin/"
         echo "Then try to run the command '$simulationApplicationName'"
         echo "-----------------------------------------------------------------------------------------"
         echo "-----------------------------------------------------------------------------------------"
         """.trimIndent()
      )

      createDesktopApplicationFile(
            "$baseFolder/usr/share/applications/",
            debianName,
            simulationApplicationName,
            "Valkyrie Obstacle Course",
            "Launch simulation of Valkyrie Obstacle Course using SCS2"
      )

      if (Os.isFamily(Os.FAMILY_UNIX))
      {
         exec {
            commandLine("chmod", "+x", "$baseFolder/DEBIAN/postinst")
         }
         exec {
            commandLine("chmod", "+x", "$sourceFolder/bin/$simulationApplicationName")
         }
         exec {
            workingDir(File(debianFolder))
            commandLine("dpkg", "--build", debianName)
         }
      }
   }
}

/**
 * This is a workaround for a bug in JavaFX 17.0.1, disabling vsync to improve framerate with multiple windows.
 * @param launchScriptFile the launch script to modify.
 */
fun addJavaFXVsyncHack(launchScriptFile: File)
{
   var originalScript = launchScriptFile.readText()
   originalScript = originalScript.replaceFirst(
         "#!/bin/sh", """
         #!/bin/bash
         # This is a workaround for a bug in JavaFX 17.0.1, disabling vsync to improve framerate with multiple windows.
         export __GL_SYNC_TO_VBLANK=0
         
      """.trimIndent()
   )

   launchScriptFile.delete()
   launchScriptFile.writeText(originalScript)
}

fun createDesktopApplicationFile(destination: String, debianName: String, applicationName: String, title: String, description: String)
{
   File("$destination/").mkdirs()
   File("$destination/$applicationName.desktop").writeText(
         """
         [Desktop Entry]
         Name=$title
         Comment=$description
         Exec=/opt/$debianName/bin/$applicationName
         Icon=/opt/$debianName/icon/scs-icon.png
         Version=1.0
         Terminal=true
         Type=Application
         Categories=Utility;Application;
         """.trimIndent()
   )
}