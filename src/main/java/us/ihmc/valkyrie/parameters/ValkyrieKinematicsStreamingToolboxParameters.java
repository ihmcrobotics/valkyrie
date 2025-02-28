package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrie.ValkyrieNetworkProcessor.NetworkProcessorVersion;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

import java.util.HashMap;
import java.util.Map;

public class ValkyrieKinematicsStreamingToolboxParameters extends KinematicsStreamingToolboxParameters
{
   public void setDefault(boolean demoMode, ValkyrieRobotModel robotModel)
   {
      super.setDefault();
      if (NetworkProcessorVersion.fromEnvironment() == NetworkProcessorVersion.IHMC)
      {
         if (robotModel.getRobotVersion() == ValkyrieRobotVersion.ARM_MASS_SIM)
         {
            defaultConfiguration.setEnableLeftHandTaskspace(false);
            defaultConfiguration.setEnableRightHandTaskspace(false);
            defaultConfiguration.setEnableLeftArmJointspace(true);
            defaultConfiguration.setEnableRightArmJointspace(true);
         }
      }
      if (demoMode)
      {
         // Add safety parameters here
         defaultConfiguration.setLockPelvis(true);
         defaultConfiguration.setEnablePelvisTaskspace(false);

         defaultAngularRateLimit= 5.0;
         inputPoseLPFBreakFrequency = 2.0;
         minimizeAngularMomentum = true;
         minimizeLinearMomentum = true;
         angularMomentumWeight.set(0.25, 0.25, 0.25);
         linearMomentumWeight.set(0.25, 0.25, 0.25);

         // Restrict joint limits
         robotModel.setRobotDefinitionMutator(robotDefinition ->
         {
            OneDoFJointDefinition spineYaw = robotDefinition.getOneDoFJointDefinition(robotModel.getJointMap().getSpineJointName(SpineJointName.SPINE_YAW));
            spineYaw.setPositionLimits(Math.toRadians(-45.0), Math.toRadians(45.0));
            OneDoFJointDefinition spineRoll = robotDefinition.getOneDoFJointDefinition(robotModel.getJointMap().getSpineJointName(SpineJointName.SPINE_ROLL));
            spineRoll.setPositionLimits(-0.1, 0.1);
            OneDoFJointDefinition spinePitch = robotDefinition.getOneDoFJointDefinition(robotModel.getJointMap().getSpineJointName(SpineJointName.SPINE_PITCH));
            spinePitch.setPositionLimits(-0.1, 0.4);

            for (RobotSide robotSide : RobotSide.values)
            {
               OneDoFJointDefinition shoulderPitch = robotDefinition.getOneDoFJointDefinition(robotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH));
               shoulderPitch.setPositionLimits(-1.5, 0.8);

               OneDoFJointDefinition shoulderRoll = robotDefinition.getOneDoFJointDefinition(robotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL));
               if (robotSide == RobotSide.LEFT)
                  shoulderRoll.setPositionLimits(-1.3, 0.3);
               else
                  shoulderRoll.setPositionLimits(-0.3, 1.3);

               OneDoFJointDefinition shoulderYaw = robotDefinition.getOneDoFJointDefinition(robotModel.getJointMap().getArmJointName(robotSide, ArmJointName.SHOULDER_YAW));
               shoulderYaw.setPositionLimits(-1.5, 1.5);
            }
         });
      }
   }

   @Override
   public Map<String, Double> createInitialConfiguration(DRCRobotModel robotModel)
   {
      Map<String, Double> initialConfigurationMap = new HashMap<>();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      RobotInitialSetup<HumanoidFloatingRootJointRobot> defaultRobotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      HumanoidJointNameMap jointMap = robotModel.getJointMap();
      defaultRobotInitialSetup.initializeRobot(robot);

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         String jointName = joint.getName();
         double q_priv = robot.getOneDegreeOfFreedomJoint(jointName).getQ();
         initialConfigurationMap.put(jointName, q_priv);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         initialConfigurationMap.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), 0.6);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH), -0.5);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.KNEE_PITCH), 1.0);
         initialConfigurationMap.put(jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH), -0.5);
      }

      return initialConfigurationMap;
   }
}
