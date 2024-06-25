package us.ihmc.valkyrie;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.simulationStarter.AvatarSimulationToolsSCS2;
import us.ihmc.avatar.simulationStarter.AvatarSimulationToolsSCS2.AvatarSimulationEnvironment;
import us.ihmc.avatar.simulationStarter.DRCSimulationTools.Modules;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

public class ValkyrieObstacleCourseCLI
{
   // Increase to 10 when you want the sims to run a little faster and don't need the data.
   private final int recordFrequencySpeedup = 10;

   public ValkyrieObstacleCourseCLI(List<Modules> modulesToStart, DRCStartingLocation startingLocation)
   {
      ValkyrieRobotVersion version = ValkyrieRosControlController.VERSION;
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, version);
      DefaultCommonAvatarEnvironment environment = new DefaultCommonAvatarEnvironment();

      if (startingLocation instanceof CustomStartingLocation customStartingLocation)
      {
         if (Double.isNaN(customStartingLocation.groundHeight))
         {
            HeightMapWithNormals heightMap = environment.getTerrainObject3D().getHeightMapIfAvailable();
            if (heightMap != null)
            {
               double groundHeight = heightMap.heightAt(customStartingLocation.x, customStartingLocation.y, 0.0);
               customStartingLocation.setGroundHeight(groundHeight);
            }
            else
            {
               customStartingLocation.setGroundHeight(0.0);
            }
         }
      }

      AvatarSimulationEnvironment simEnvironment = AvatarSimulationToolsSCS2.setupSimulationEnvironment(robotModel,
                                                                                                        environment,
                                                                                                        null,
                                                                                                        null,
                                                                                                        startingLocation,
                                                                                                        modulesToStart);
      if (simEnvironment != null)
      {
         simEnvironment.getAvatarSimulationFactory().setSimulationDataRecordTimePeriod(recordFrequencySpeedup * robotModel.getControllerDT());
         simEnvironment.build();
         simEnvironment.start();
      }
   }

   public static void main(String[] args)
   {
      String propertiesPathLongOpt = "properties";
      String rosLongOpt = "ros";
      String sensorLongOpt = "sensor";
      String reaLongOpt = "rea";
      String kinematicsLongOpt = "kinematics";
      String kinematicsPlanningLongOpt = "kinematicsplanning";
      String wholeBodyTrajectoryLongOpt = "wholebodytraj";
      String directionalLongOpt = "directional";
      String footstepPlanningLongOpt = "footstepplan";
      String startingLocationLongOpt = "startinglocation";

      Options options = new Options();
      options.addOption("p", propertiesPathLongOpt, true, "[Optional] Path to a properties file to configure the simulation.");
      options.addOption("r", rosLongOpt, false, "Start the simulation with ROS module enabled.");
      options.addOption("s", sensorLongOpt, false, "Start the simulation with the sensor suite (Lidar, camera, etc.).");
      options.addOption("e", reaLongOpt, false, "Starts the Robot Environment Awareness module.");
      options.addOption("k", kinematicsLongOpt, false, "Start the kinematics module.");
      options.addOption("l", kinematicsPlanningLongOpt, false, "Start the kinematics planning module.");
      options.addOption("m", wholeBodyTrajectoryLongOpt, false, "Start the whole body trajectory module.");
      options.addOption("j", directionalLongOpt, false, "Start the directional control module.");
      options.addOption("f", footstepPlanningLongOpt, false, "Start the footstep planning module.");
      Option startingLocationOption = new Option("o",
                                                 startingLocationLongOpt,
                                                 true,
                                                 "[Optional] Starting location for the robot can be <x y yaw>, or <x y yaw groundHeight> or one of the following presets:\n\t%s.".formatted(
                                                       Arrays.toString(DRCObstacleCourseStartingLocation.values())));
      startingLocationOption.setArgs(Option.UNLIMITED_VALUES);
      options.addOption(startingLocationOption);
      options.addOption("h", "help", false, "Print this message.");

      String propertiesPath = null;
      boolean ros = false;
      boolean sensor = false;
      boolean rea = false;
      boolean kinematics = false;
      boolean kinematicsPlanning = false;
      boolean wholeBodyTrajectory = false;
      boolean directional = false;
      boolean footstepPlanner = false;
      DRCStartingLocation startingLocation = null;

      List<Modules> modulesToStart = new ArrayList<>();
      modulesToStart.add(Modules.SIMULATION);
      modulesToStart.add(Modules.NETWORK_PROCESSOR);

      // Create a parser
      CommandLineParser parser = new DefaultParser();
      try
      {
         // Parse the command line arguments
         CommandLine line = parser.parse(options, args);

         if (line.hasOption("help"))
         {
            HelpFormatter formatter = new HelpFormatter();
            String header = "Valkyrie Obstacle Course Demo Simulation: This application runs the Valkyrie obstacle course simulation.";
            String footer = "Please report issues at https://github.com/ihmcrobotics/valkyrie/issues.";
            formatter.printHelp("ValkyrieObstacleCourseCLI", header, options, footer, true);
            System.exit(0);
            return;
         }

         // Access parsed arguments
         propertiesPath = line.getOptionValue(propertiesPathLongOpt);
         ros = line.hasOption(rosLongOpt);
         sensor = line.hasOption(sensorLongOpt);
         rea = line.hasOption(reaLongOpt);
         kinematics = line.hasOption(kinematicsLongOpt);
         kinematicsPlanning = line.hasOption(kinematicsPlanningLongOpt);
         wholeBodyTrajectory = line.hasOption(wholeBodyTrajectoryLongOpt);
         directional = line.hasOption(directionalLongOpt);
         footstepPlanner = line.hasOption(footstepPlanningLongOpt);
         if (line.hasOption(startingLocationLongOpt))
         {
            try
            {
               startingLocation = DRCObstacleCourseStartingLocation.valueOf(line.getOptionValue(startingLocationLongOpt));
            }
            catch (IllegalArgumentException e)
            {
               // See if the user passed [x, y, yaw]
               startingLocation = parseXYYaw(line.getOptionValues(startingLocationLongOpt));
            }
         }

         if (propertiesPath != null)
         {
            System.out.println("Loading properties file: " + propertiesPath);
            Properties properties = new Properties();
            properties.load(new FileInputStream(propertiesPath));
            if (properties.containsKey(rosLongOpt))
               ros = Boolean.parseBoolean(properties.getProperty(rosLongOpt));
            if (properties.containsKey(sensorLongOpt))
               sensor = Boolean.parseBoolean(properties.getProperty(sensorLongOpt));
            if (properties.containsKey(reaLongOpt))
               rea = Boolean.parseBoolean(properties.getProperty(reaLongOpt));
            if (properties.containsKey(kinematicsLongOpt))
               kinematics = Boolean.parseBoolean(properties.getProperty(kinematicsLongOpt));
            if (properties.containsKey(kinematicsPlanningLongOpt))
               kinematicsPlanning = Boolean.parseBoolean(properties.getProperty(kinematicsPlanningLongOpt));
            if (properties.containsKey(wholeBodyTrajectoryLongOpt))
               wholeBodyTrajectory = Boolean.parseBoolean(properties.getProperty(wholeBodyTrajectoryLongOpt));
            if (properties.containsKey(directionalLongOpt))
               directional = Boolean.parseBoolean(properties.getProperty(directionalLongOpt));
            if (properties.containsKey(footstepPlanningLongOpt))
               footstepPlanner = Boolean.parseBoolean(properties.getProperty(footstepPlanningLongOpt));
            if (properties.containsKey(startingLocationLongOpt))
               startingLocation = DRCObstacleCourseStartingLocation.valueOf(properties.getProperty(startingLocationLongOpt));
         }

         if (ros)
            modulesToStart.add(Modules.ROS_MODULE);
         if (sensor)
            modulesToStart.add(Modules.SENSOR_MODULE);
         if (rea)
            modulesToStart.add(Modules.REA_MODULE);
         if (kinematics)
            modulesToStart.add(Modules.KINEMATICS_TOOLBOX);
         if (kinematicsPlanning)
            modulesToStart.add(Modules.KINEMATICS_PLANNING_TOOLBOX);
         if (wholeBodyTrajectory)
            modulesToStart.add(Modules.WHOLE_BODY_TRAJECTORY_TOOLBOX);
         if (directional)
            modulesToStart.add(Modules.DIRECTIONAL_CONTROL_TOOLBOX);
         if (footstepPlanner)
            modulesToStart.add(Modules.FOOTSTEP_PLANNING_TOOLBOX);
      }
      catch (Exception e)
      {
         System.err.println("Parsing failed, use option -h to see usage. Reason: " + e.getMessage());
         System.exit(0);
         return;
      }

      if (startingLocation == null)
         startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      System.out.println("---------------------------------------------------------------------------------------------------");
      System.out.println("---------------------------------------------------------------------------------------------------");
      System.out.println("Starting Valkyrie Obstacle Course Simulation with modules: " + modulesToStart + ", starting location: " + startingLocation);
      System.out.println("---------------------------------------------------------------------------------------------------");
      System.out.println("---------------------------------------------------------------------------------------------------");

      new ValkyrieObstacleCourseCLI(modulesToStart, startingLocation);
   }

   private static DRCStartingLocation parseXYYaw(String[] args)
   {
      if (args.length != 3)
         return null;

      try
      {
         double x = Double.parseDouble(args[0]);
         double y = Double.parseDouble(args[1]);
         double yaw = Double.parseDouble(args[2]);
         return new CustomStartingLocation(x, y, yaw);
      }
      catch (NumberFormatException e)
      {
         return null;
      }
   }

   private static class CustomStartingLocation implements DRCStartingLocation
   {
      private final double x, y, yaw;
      private double groundHeight = Double.NaN;

      public CustomStartingLocation(double x, double y, double yaw)
      {
         this.x = x;
         this.y = y;
         this.yaw = yaw;
      }

      public CustomStartingLocation(double x, double y, double yaw, double groundHeight)
      {
         this(x, y, yaw);
         this.groundHeight = groundHeight;
      }

      public void setGroundHeight(double groundHeight)
      {
         this.groundHeight = groundHeight;
      }

      @Override
      public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
      {
         return new OffsetAndYawRobotInitialSetup(x, y, groundHeight, yaw);
      }

      @Override
      public String toString()
      {
         if (Double.isNaN(groundHeight))
            return "Custom starting location: [x=" + x + ", y=" + y + ", yaw=" + yaw + "]";
         else
            return "Custom starting location: [x=" + x + ", y=" + y + ", yaw=" + yaw + ", groundHeight=" + groundHeight + "]";
      }
   }
}
