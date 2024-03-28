package us.ihmc.valkyrie;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class ValkyrieDirectRobotInterface implements RobotLowLevelMessenger
{
   private final ROS2PublisherBasics<HighLevelStateMessage> highLevelStatePublisher;
   private final ROS2PublisherBasics<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public ValkyrieDirectRobotInterface(ROS2NodeInterface ros2Node, DRCRobotModel robotModel)
   {
      ROS2Topic inputTopic = HumanoidControllerAPI.getInputTopic(robotModel.getSimpleRobotName());
      highLevelStatePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(HighLevelStateMessage.class).withTopic(inputTopic));
      abortWalkingPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(AbortWalkingMessage.class).withTopic(inputTopic));
      pauseWalkingPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(PauseWalkingMessage.class).withTopic(inputTopic));
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
