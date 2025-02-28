package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieKinematicsStreamingToolboxParameters;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class ValkyrieKinematicsStreamingToolboxModule extends KinematicsStreamingToolboxModule
{
   /**
    * The idea is to have a quick way to switch to a safer and more conservative operation mode meant
    * for a random random person to try out the IK streaming.
    */
   public static final boolean DEMO_MODE = false;

   public ValkyrieKinematicsStreamingToolboxModule(DRCRobotModel robotModel,
                                                   ValkyrieKinematicsStreamingToolboxParameters parameters,
                                                   boolean startYoVariableServer)
   {
      super(robotModel, parameters, startYoVariableServer);
      controller.setInitialRobotConfigurationNamedMap(parameters.createInitialConfiguration(robotModel));
   }

   @Override
   public DataServerSettings createYoVariableServerSettings()
   {
      return super.createYoVariableServerSettings(true);
   }

   public static void main(String[] args)
   {

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRosControlController.VERSION);
      ValkyrieJointMap jointMap = robotModel.getJointMap();
      if (DEMO_MODE)
      {
         ValkyrieKinematicsCollisionModel kinematicsCollisionModel = new ValkyrieKinematicsCollisionModel(jointMap);
         kinematicsCollisionModel.setEnableConservativeCollisions(true);
         robotModel.setHumanoidRobotKinematicsCollisionModel(kinematicsCollisionModel);
      }
      boolean startYoVariableServer = true;

      ValkyrieKinematicsStreamingToolboxParameters parameters = new ValkyrieKinematicsStreamingToolboxParameters();
      parameters.setDefault(DEMO_MODE, robotModel);

      ValkyrieKinematicsStreamingToolboxModule module = new ValkyrieKinematicsStreamingToolboxModule(robotModel,
                                                                                                     parameters,
                                                                                                     startYoVariableServer);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         System.out.println("Shutting down " + ValkyrieKinematicsStreamingToolboxModule.class.getSimpleName());
         module.closeAndDispose();
      }));
   }
}
