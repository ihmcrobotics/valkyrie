package us.ihmc.valkyrie.fingers;

import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.handControl.HandFingerTrajectoryMessagePublisher;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.valkyrie.network.ValkyrieMessageTools;

public class ValkyrieFingerTrajectoryMessagePublisher implements HandFingerTrajectoryMessagePublisher
{
   private final ROS2PublisherBasics<ValkyrieHandFingerTrajectoryMessage> publisher;

   public ValkyrieFingerTrajectoryMessagePublisher(ROS2Node ros2Node, ROS2Topic inputTopic)
   {
      publisher = ros2Node.createPublisher(inputTopic.withTypeName(ValkyrieHandFingerTrajectoryMessage.class));
   }

   @Override
   public void sendFingerTrajectoryMessage(RobotSide robotSide, TDoubleArrayList desiredPositions, TDoubleArrayList trajectoryTimes)
   {
      if (desiredPositions.size() != trajectoryTimes.size())
         throw new RuntimeException("Inconsistent array lengths.");

      ValkyrieHandFingerTrajectoryMessage message = new ValkyrieHandFingerTrajectoryMessage();
      message.setRobotSide(robotSide.toByte());

      for (int i = 0; i < desiredPositions.size(); i++)
      {
         ValkyrieMessageTools.appendDesiredFingerConfiguration(ValkyrieFingerMotorName.values[i], trajectoryTimes.get(i), desiredPositions.get(i), message);
      }

      publisher.publish(message);
   }
}
