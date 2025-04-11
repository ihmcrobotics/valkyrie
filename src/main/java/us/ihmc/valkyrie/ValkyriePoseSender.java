package us.ihmc.valkyrie;

import com.google.common.base.CaseFormat;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class ValkyriePoseSender
{
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final ROS2Publisher<GoHomeMessage> goHomePublisher;
   private final ROS2Publisher<ChestTrajectoryMessage> chestPublisher;
   private final ROS2Publisher<NeckTrajectoryMessage> neckPublisher;
   private final ROS2Publisher<FootstepDataListMessage> footstepPublisher;
   private final ROS2Publisher<ArmTrajectoryMessage> armPublisher;
   private final GoHomeMessage goHomeMessage = new GoHomeMessage();

   private final FullHumanoidRobotModel fullRobotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
   private final ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();

   public ValkyriePoseSender(ROS2Node ros2Node, String robotName, ValkyrieRobotModel robotModel)
   {
      fullRobotModel = robotModel.createFullRobotModel();
      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      ROS2Topic<GoHomeMessage> goHomeTopic = HumanoidControllerAPI.getTopic(GoHomeMessage.class, robotName);
      goHomePublisher = ros2Node.createPublisher(goHomeTopic);

      ROS2Topic<ChestTrajectoryMessage> chestTopic = HumanoidControllerAPI.getTopic(ChestTrajectoryMessage.class, robotName);
      chestPublisher = ros2Node.createPublisher(chestTopic);

      ROS2Topic<NeckTrajectoryMessage> neckTopic = HumanoidControllerAPI.getTopic(NeckTrajectoryMessage.class, robotName);
      neckPublisher = ros2Node.createPublisher(neckTopic);

      ROS2Topic<FootstepDataListMessage> footstepTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, robotName);
      footstepPublisher = ros2Node.createPublisher(footstepTopic);

      ROS2Topic<ArmTrajectoryMessage> armTopic = HumanoidControllerAPI.getTopic(ArmTrajectoryMessage.class, robotName);
      armPublisher = ros2Node.createPublisher(armTopic);

      ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
      ros2Node.createSubscription(controllerOutputTopic.withTypeName(RobotConfigurationData.class), s -> robotConfigurationData.set(s.takeNextData()));

      this.rootJoint = fullRobotModel.getRootJoint();
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);
   }

   public void sendHomePose()
   {
      double trajectoryDuration = 3.0;
      goHomeMessage.setTrajectoryTime(trajectoryDuration);

      goHomeMessage.setHumanoidBodyPart(HumanoidBodyPart.CHEST.toByte());
      goHomePublisher.publish(goHomeMessage);

      goHomeMessage.setHumanoidBodyPart(HumanoidBodyPart.PELVIS.toByte());
      goHomePublisher.publish(goHomeMessage);

      goHomeMessage.setHumanoidBodyPart(HumanoidBodyPart.ARM.toByte());
      goHomeMessage.setRobotSide(RobotSide.LEFT.toByte());
      goHomePublisher.publish(goHomeMessage);

      goHomeMessage.setHumanoidBodyPart(HumanoidBodyPart.ARM.toByte());
      goHomeMessage.setRobotSide(RobotSide.RIGHT.toByte());
      goHomePublisher.publish(goHomeMessage);

      neckPublisher.publish(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[]{0.0, 0.0, 0.0}));
   }

   public void sendSteps()
   {
      update();

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      int numSteps = 4;
      RobotSide swingSide = RobotSide.LEFT;

      for (int i = 0; i < numSteps; i++)
      {
         FootstepDataMessage footstep = footstepDataListMessage.getFootstepDataList().add();

         FramePose3D pose = new FramePose3D(fullRobotModel.getSoleFrames().get(swingSide));
         pose.changeFrame(ReferenceFrame.getWorldFrame());

         footstep.getLocation().set(pose.getPosition());
         footstep.getOrientation().set(pose.getOrientation());
         footstep.setRobotSide(swingSide.toByte());

         swingSide = swingSide.getOppositeSide();
      }

      footstepPublisher.publish(footstepDataListMessage);
   }

   public void sendLookLeft()
   {
      goHomeMessage.setHumanoidBodyPart(HumanoidBodyPart.ARM.toByte());
      goHomeMessage.setRobotSide(RobotSide.LEFT.toByte());
      goHomePublisher.publish(goHomeMessage);

      goHomeMessage.setHumanoidBodyPart(HumanoidBodyPart.ARM.toByte());
      goHomeMessage.setRobotSide(RobotSide.RIGHT.toByte());
      goHomePublisher.publish(goHomeMessage);

      update();
      double yaw = Math.toRadians(30.0);
      FrameQuaternion orientation = new FrameQuaternion(referenceFrames.getMidFeetZUpFrame(), yaw, 0.0, 0.0);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());
      chestPublisher.publish(HumanoidMessageTools.createChestTrajectoryMessage(3.0, orientation));

      neckPublisher.publish(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[]{0.0, yaw, 0.0}));
   }

   public void sendLookRight()
   {
      update();
      double yaw = -Math.toRadians(30.0);
      FrameQuaternion orientation = new FrameQuaternion(referenceFrames.getMidFeetZUpFrame(), yaw, 0.0, 0.0);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());
      chestPublisher.publish(HumanoidMessageTools.createChestTrajectoryMessage(3.0, orientation));

      neckPublisher.publish(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[]{0.0, yaw, 0.0}));
   }

   public void sendArmsUp()
   {
      armPublisher.publish(HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 3.0, new double[]{0.0, -0.4, -1.0, -1.57}));
      armPublisher.publish(HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 3.0, new double[]{0.0, 0.4, -1.0, 1.57}));
   }

   public void sendArmsDown()
   {
      armPublisher.publish(HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.LEFT, 3.0, new double[]{-0.3, -0.9, 1.3, -1.57}));
      armPublisher.publish(HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 3.0, new double[]{-0.3, 0.9, 1.3, 1.57}));

      double pitch = Math.toRadians(20.0);
      FrameQuaternion orientation = new FrameQuaternion(referenceFrames.getMidFeetZUpFrame(), 0.0, pitch, 0.0);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());
      chestPublisher.publish(HumanoidMessageTools.createChestTrajectoryMessage(3.0, orientation));
   }

   public void update()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, rootJoint, oneDoFJoints);
         referenceFrames.updateFrames();
      }
   }

   public static void main(String[] args)
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("ihmc_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, "ValkyriePoseSender"));
      ValkyrieRobotModel valkyrieRobotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.ARM_MASS_SIM);

      ValkyriePoseSender sender = new ValkyriePoseSender(ros2Node, valkyrieRobotModel.getSimpleRobotName(), valkyrieRobotModel);

      while (true)
      {
         int pauseTime = 5000;

         sender.sendHomePose();
         ThreadTools.sleep(pauseTime);
         sender.sendArmsUp();
         ThreadTools.sleep(pauseTime);
         sender.sendArmsDown();
         ThreadTools.sleep(pauseTime);
         sender.sendLookLeft();
         ThreadTools.sleep(pauseTime);
         sender.sendLookRight();
         ThreadTools.sleep(pauseTime);
         sender.sendHomePose();
         ThreadTools.sleep(pauseTime);
         sender.sendSteps();
         ThreadTools.sleep(pauseTime);
      }
   }
}
