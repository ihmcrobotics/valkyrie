package us.ihmc.valkyrie;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;

public class ValkyrieDirectRobotInterface implements RobotLowLevelMessenger
{
   private final ROS2Publisher<HighLevelStateMessage> highLevelStatePublisher;
   private final ROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2Publisher<PauseWalkingMessage> pauseWalkingPublisher;

   public ValkyrieDirectRobotInterface(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      ROS2Topic inputTopic = HumanoidControllerAPI.getInputTopic(robotModel.getSimpleRobotName());
      highLevelStatePublisher = ros2Node.createPublisher(inputTopic.withTypeName(HighLevelStateMessage.class));
      abortWalkingPublisher = ros2Node.createPublisher(inputTopic.withTypeName(AbortWalkingMessage.class));
      pauseWalkingPublisher = ros2Node.createPublisher(inputTopic.withTypeName(PauseWalkingMessage.class));
   }

   @Override
   public void sendAbortWalkingRequest()
   {
      abortWalkingPublisher.publish(new AbortWalkingMessage());
   }

   @Override
   public void sendFreezeRequest()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.EXIT_WALKING.toByte());
      highLevelStatePublisher.publish(message);
   }

   @Override
   public void sendStandRequest()
   {
      HighLevelStateMessage message = new HighLevelStateMessage();
      message.setHighLevelControllerName(HighLevelControllerName.STAND_TRANSITION_STATE.toByte());
      highLevelStatePublisher.publish(message);
   }

   @Override
   public void sendPauseWalkingRequest()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(true);
      pauseWalkingPublisher.publish(message);
   }

   @Override
   public void sendContinueWalkingRequest()
   {
      PauseWalkingMessage message = new PauseWalkingMessage();
      message.setPause(false);
      pauseWalkingPublisher.publish(message);
   }
}
