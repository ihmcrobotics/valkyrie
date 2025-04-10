package us.ihmc.valkyrie.skeletonTracking;

import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.ExternalForceEstimationOutputStatus;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

import java.util.ArrayList;
import java.util.List;

public class ValkyrieSkeletonTrackingModule extends ToolboxModule
{
   public static final int UPDATE_PERIOD_MILLIS = 30;
   private static final boolean START_YO_VARIABLE_SERVER = true;

   private final SkeletonTrackingController skeletonTrackingController;

   public ValkyrieSkeletonTrackingModule(ValkyrieRobotModel robotModel)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), START_YO_VARIABLE_SERVER, UPDATE_PERIOD_MILLIS);
      skeletonTrackingController = new SkeletonTrackingController(fullRobotModel, statusOutputManager, registry);
      setTimeWithoutInputsBeforeGoingToSleep(60.0);
   }

   @Override
   public void registerExtraPuSubs(ROS2Node ros2Node)
   {
      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);

      ros2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class), s ->
      {
         if(skeletonTrackingController != null)
            skeletonTrackingController.updateRobotConfigurationData(s.takeNextData());
      });

   }

   @Override
   public ToolboxController getToolboxController()
   {
      return skeletonTrackingController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return List.of();
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(KinematicsStreamingToolboxInputMessage.class);
      status.add(KinematicsStreamingToolboxConfigurationMessage.class);
      return status;
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return null;
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return null;
   }

   public static void main(String[] args)
   {
      new ValkyrieSkeletonTrackingModule(new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM));
   }
}
