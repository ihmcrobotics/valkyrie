package us.ihmc.valkyrie.joystick;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HighLevelStateMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.avatar.joystickBasedJavaFXController.HumanoidRobotPunchMessenger;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class ValkyriePunchMessenger implements HumanoidRobotPunchMessenger, RobotLowLevelMessenger
{
   private final ROS2PublisherBasics<ArmTrajectoryMessage> armTrajectoryPublisher;
   private final ROS2PublisherBasics<HighLevelStateMessage> highLevelStatePublisher;
   private final ROS2PublisherBasics<AbortWalkingMessage> abortWalkingPublisher;
   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   public ValkyriePunchMessenger(String robotName, ROS2NodeInterface ros2Node)
   {
      ROS2Topic<?> inputTopic = HumanoidControllerAPI.getInputTopic(robotName);
      armTrajectoryPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(ArmTrajectoryMessage.class).withTopic(inputTopic));
      highLevelStatePublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(HighLevelStateMessage.class).withTopic(inputTopic));
      abortWalkingPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(AbortWalkingMessage.class).withTopic(inputTopic));
      pauseWalkingPublisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(PauseWalkingMessage.class).withTopic(inputTopic));
   }


   @Override
   public void sendArmHomeConfiguration(double trajectoryDuration, RobotSide... robotSides)
   {
      for (RobotSide robotSide : robotSides)
      {
         double[] jointAngles = new double[7];
         int index = 0;
         jointAngles[index++] = 1.0; // shoulderPitch
         jointAngles[index++] = robotSide.negateIfRightSide(-1.5); // shoulderRoll
         jointAngles[index++] = -0.2; // shoulderYaw
         jointAngles[index++] = robotSide.negateIfRightSide(-2.0); // elbowPitch
         jointAngles[index++] = robotSide.negateIfRightSide(0.0); // forearmYaw
         jointAngles[index++] = robotSide.negateIfRightSide(0.0); // wristRoll
         jointAngles[index++] = 0.0; // wristPitch
         ArmTrajectoryMessage message = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryDuration, jointAngles);
         armTrajectoryPublisher.publish(message);
      }
   }

   @Override
   public void sendArmStraightConfiguration(double trajectoryDuration, RobotSide robotSide)
   {
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
   public void sendAbortWalkingRequest()
   {
      abortWalkingPublisher.publish(new AbortWalkingMessage());
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
