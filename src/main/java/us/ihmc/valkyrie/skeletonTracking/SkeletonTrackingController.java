package us.ihmc.valkyrie.skeletonTracking;

import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxHelper;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.valkyrie.perception.ZEDBodyTracking;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class SkeletonTrackingController extends ToolboxController
{
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final HumanoidReferenceFrames referenceFrames;

   private static final boolean ENABLE_CHEST = true;
   private static final boolean ENABLE_PELVIS = true;
   private static final boolean ENABLE_HANDS = true;

   private static final int HEAD = 0;
   private static final int CHEST = 1;
   private static final int R_SHOULDER = 2;
   private static final int R_ELBOW = 3;
   private static final int R_HAND = 4;
   private static final int L_SHOULDER = 5;
   private static final int L_ELBOW = 6;
   private static final int L_HAND = 7;
   private static final int R_HIP = 8;
   private static final int R_KNEE = 9;
   private static final int R_FOOT = 10;
   private static final int L_HIP = 11;
   private static final int L_KNEE = 12;
   private static final int L_FOOT = 13;
   private static final int R_FACE_INNER = 14;
   private static final int R_FACE_OUTER = 15;
   private static final int L_FACE_INNER = 16;
   private static final int L_FACE_OUTER = 17;

   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////              TRACKING DATA                ////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private final YoBoolean enableSkeletonTracking = new YoBoolean("enableSkeletonTracking", registry);
   private final YoBoolean isInitialized = new YoBoolean("isInitialized", registry);

   private final YoFramePoint3D[] keypoints = new YoFramePoint3D[18];
   private final ZEDBodyTracking zedBodyTracking = new ZEDBodyTracking();

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////              USER DATA                ///////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private final PoseReferenceFrame userFrame = new PoseReferenceFrame("UserFrame", ReferenceFrame.getWorldFrame());

   private final SideDependentList<FramePoint3D> initialUserHandPositions = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final FramePose3D initialUserChestPose = new FramePose3D();
   private final SideDependentList<FramePoint3D> currentUserHandPositions = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final FramePose3D currentUserChestPose = new FramePose3D();

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////              ROBOT DATA                //////////////////////////////////////////////////
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private final SideDependentList<FramePoint3D> initialRobotHandPositions = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final SideDependentList<FramePoint3D> handControlFramePosition = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final FramePose3D initialRobotChestPose = new FramePose3D();
   private final FramePose3D initialRobotPelvisPose = new FramePose3D();

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////              KST INPUT                //////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private final YoDouble heightAdjustment = new YoDouble("heightAdjustment", registry);
   private final YoDouble yawAdjustment = new YoDouble("yawAdjustment", registry);
   private final SideDependentList<YoFrameVector3D> handAdjustments = new SideDependentList<>();
   private final YoDouble interpolationAlpha = new YoDouble("interpolationAlpha", registry);
   private final YoFramePose3D desiredChestPose = new YoFramePose3D("desiredChestPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D currentChestPose = new YoFramePose3D("currentChestPose", ReferenceFrame.getWorldFrame(), registry);

   private final FrameVector3D tmpVector = new FrameVector3D();
   private final SideDependentList<YoFramePoint3D> desiredHandPositions = new SideDependentList<>();

   private final ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final ROS2Publisher<KinematicsToolboxInitialConfigurationMessage> kstConfigPublisher;
   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> kstInputPublisher;
   private final ROS2Publisher<GoHomeMessage> goHomePublisher;

   private final FramePoint3D desiredCoM = new FramePoint3D();
   private final KinematicsStreamingToolboxConfigurationMessage kstConfiguration = new KinematicsStreamingToolboxConfigurationMessage();
   private final KinematicsStreamingToolboxInputMessage kstInput = new KinematicsStreamingToolboxInputMessage();

   private final ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
   private final GoHomeMessage goHomeMessage = new GoHomeMessage();

   public SkeletonTrackingController(String robotName,
                                     FullHumanoidRobotModel fullRobotModel,
                                     StatusMessageOutputManager statusOutputManager,
                                     ROS2Node ros2Node,
                                     YoGraphicsListRegistry yoGraphicsListRegistry,
                                     YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      this.fullRobotModel = fullRobotModel;
      this.rootJoint = fullRobotModel.getRootJoint();
      this.oneDoFJoints = getAllJointsExcludingHands(fullRobotModel);

      kstConfiguration.setLockPelvis(false);
      kstConfiguration.setLockChest(false);

      zedBodyTracking.initialize();
      zedBodyTracking.enable();

      ROS2Topic<ToolboxStateMessage> toolboxStateTopic = KinematicsStreamingToolboxModule.getInputStateTopic(robotName);
      ROS2Topic<KinematicsToolboxInitialConfigurationMessage> kstConfigTopic = KinematicsStreamingToolboxModule.getInputStreamingInitialConfigurationTopic(robotName);
      ROS2Topic<KinematicsStreamingToolboxInputMessage> kstInputTopic = KinematicsStreamingToolboxModule.getInputCommandTopic(robotName);
      ROS2Topic<GoHomeMessage> goHomeTopic = HumanoidControllerAPI.getTopic(GoHomeMessage.class, robotName);

      toolboxStatePublisher = ros2Node.createPublisher(toolboxStateTopic);
      kstConfigPublisher = ros2Node.createPublisher(kstConfigTopic);
      kstInputPublisher = ros2Node.createPublisher(kstInputTopic);
      goHomePublisher = ros2Node.createPublisher(goHomeTopic);

      for (int i = 0; i < keypoints.length; i++)
      {
         keypoints[i] = new YoFramePoint3D("keypoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), new YoGraphicPosition("keypoint" + i + "position", keypoints[i], 0.04, YoAppearance.Red()));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handControlFramePosition.get(robotSide).setToZero(fullRobotModel.getHandControlFrame(robotSide));
         handControlFramePosition.get(robotSide).changeFrame(fullRobotModel.getHand(robotSide).getBodyFixedFrame());

         desiredHandPositions.set(robotSide, new YoFramePoint3D("desired" + robotSide + "HandPosition", ReferenceFrame.getWorldFrame(), registry));
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), new YoGraphicPosition("desired" + robotSide + "HandPositionViz", desiredHandPositions.get(robotSide), 0.04, YoAppearance.Blue()));

         handAdjustments.set(robotSide, new YoFrameVector3D("handAdjustment" + robotSide, ReferenceFrame.getWorldFrame(), registry));
      }

      //      yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), new YoGraphicCoordinateSystem("desiredChestPoseViz", desiredChestPose, 0.4));
      yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), new YoGraphicCoordinateSystem("currentChestPoseViz", currentChestPose, 0.4));

      interpolationAlpha.set(0.05);

      isInitialized.set(false);
   }

   @Override
   public boolean initialize()
   {
      return true;
   }

   @Override
   public void updateInternal()
   {
      for (int i = 0; i < keypoints.length; i++)
      {
         keypoints[i].setToNaN();
      }

      List<Point3D> bodyPartLocations = zedBodyTracking.getBodyPartLocations();
      for (int i = 0; i < bodyPartLocations.size(); i++)
      {
         Point3D keypoint = bodyPartLocations.get(i);

         // Mirror so that from the user's perspective x is forward and y is to the left
         keypoints[i].set(-keypoint.getX(), -keypoint.getY(), keypoint.getZ());
      }

      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         KinematicsToolboxHelper.setRobotStateFromRobotConfigurationData(robotConfigurationData, rootJoint, oneDoFJoints);
         referenceFrames.updateFrames();
      }

      // take in latest skeleton data and configure KST message
      if (enableSkeletonTracking.getValue())
      {
         sendGoHomeMessage = false;
         updateUserPositions(bodyPartLocations);

         currentChestPose.set(currentUserChestPose);

         if (!isInitialized.getValue())
         {
            isInitialized.set(true);

            // initialize user data
            initialUserChestPose.set(currentUserChestPose);
            for (RobotSide robotSide : RobotSide.values)
            {
               initialUserHandPositions.get(robotSide).set(currentUserHandPositions.get(robotSide));
            }

            userFrame.setPoseAndUpdate(initialUserChestPose);

            // initialize robot data
            initialRobotChestPose.setToZero(fullRobotModel.getChest().getBodyFixedFrame());
            initialRobotChestPose.changeFrame(ReferenceFrame.getWorldFrame());
            initialRobotPelvisPose.setToZero(fullRobotModel.getPelvis().getBodyFixedFrame());
            initialRobotPelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
            for (RobotSide robotSide : RobotSide.values)
            {
               initialRobotHandPositions.get(robotSide).setToZero(fullRobotModel.getHandControlFrame(robotSide));
               initialRobotHandPositions.get(robotSide).changeFrame(ReferenceFrame.getWorldFrame());
            }

            toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
            toolboxStatePublisher.publish(toolboxStateMessage);

            heightAdjustment.set(0.0);
            yawAdjustment.set(0.0);
            for (RobotSide robotSide : RobotSide.values)
            {
               handAdjustments.get(robotSide).setToZero();
            }
         }

         // Keep CoM in mid-feet zup
         desiredCoM.setToZero(referenceFrames.getMidFeetZUpFrame());
         desiredCoM.changeFrame(ReferenceFrame.getWorldFrame());
         kstInput.setUseCenterOfMassInput(true);
         kstInput.getCenterOfMassInput().getDesiredPositionInWorld().set(desiredCoM);
         kstInput.getCenterOfMassInput().setHasDesiredLinearVelocity(false);
         kstInput.getCenterOfMassInput().getSelectionMatrix().setXSelected(true);
         kstInput.getCenterOfMassInput().getSelectionMatrix().setYSelected(true);
         kstInput.getCenterOfMassInput().getSelectionMatrix().setZSelected(false);

         // Clear previous inputs
         kstInput.getInputs().clear();

         // Chest orientation
         if (ENABLE_CHEST)
         {
            KinematicsToolboxRigidBodyMessage kstChestInput = kstInput.getInputs().add();
            kstChestInput.setEndEffectorHashCode(fullRobotModel.getChest().hashCode());

            double yawAdjustment = EuclidCoreTools.angleDifferenceMinusPiToPi(currentUserChestPose.getYaw(), initialUserChestPose.getYaw());
            this.yawAdjustment.set(EuclidCoreTools.interpolate(this.yawAdjustment.getValue(), yawAdjustment, interpolationAlpha.getValue()));

            kstChestInput.getDesiredOrientationInWorld().set(initialRobotChestPose.getOrientation());
            kstChestInput.getDesiredOrientationInWorld().appendYawRotation(this.yawAdjustment.getValue());
            kstChestInput.setHasDesiredLinearVelocity(false);

            kstChestInput.getLinearSelectionMatrix().setXSelected(false);
            kstChestInput.getLinearSelectionMatrix().setYSelected(false);
            kstChestInput.getLinearSelectionMatrix().setZSelected(false);
            kstChestInput.getAngularSelectionMatrix().setXSelected(false);
            kstChestInput.getAngularSelectionMatrix().setYSelected(false);
            kstChestInput.getAngularSelectionMatrix().setZSelected(true);

            desiredChestPose.getOrientation().set(kstChestInput.getDesiredOrientationInWorld());
         }

         // Pelvis height
         if (ENABLE_PELVIS)
         {
            KinematicsToolboxRigidBodyMessage kstPelvisInput = kstInput.getInputs().add();
            kstPelvisInput.setEndEffectorHashCode(fullRobotModel.getPelvis().hashCode());

            double adjustmentZ = currentUserChestPose.getPosition().getZ() - initialUserChestPose.getPosition().getZ();
            heightAdjustment.set(EuclidCoreTools.interpolate(heightAdjustment.getValue(), adjustmentZ, interpolationAlpha.getValue()));

            kstPelvisInput.getDesiredPositionInWorld().set(initialRobotPelvisPose.getPosition());
            kstPelvisInput.getDesiredPositionInWorld().addZ(heightAdjustment.getValue());
            kstPelvisInput.setHasDesiredLinearVelocity(false);

            kstPelvisInput.getLinearSelectionMatrix().setXSelected(false);
            kstPelvisInput.getLinearSelectionMatrix().setYSelected(false);
            kstPelvisInput.getLinearSelectionMatrix().setZSelected(true);
            kstPelvisInput.getAngularSelectionMatrix().setXSelected(false);
            kstPelvisInput.getAngularSelectionMatrix().setYSelected(false);
            kstPelvisInput.getAngularSelectionMatrix().setZSelected(false);
         }

         // Hand positions
         if (ENABLE_HANDS)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               KinematicsToolboxRigidBodyMessage kstHandInput = kstInput.getInputs().add();
               kstHandInput.setEndEffectorHashCode(fullRobotModel.getHand(robotSide).hashCode());

               tmpVector.setToZero(ReferenceFrame.getWorldFrame());
               tmpVector.sub(currentUserHandPositions.get(robotSide), initialUserHandPositions.get(robotSide));
               tmpVector.changeFrame(userFrame);

               tmpVector.setReferenceFrame(referenceFrames.getMidFeetZUpFrame());
               tmpVector.changeFrame(ReferenceFrame.getWorldFrame());
               handAdjustments.get(robotSide).interpolate(tmpVector, interpolationAlpha.getValue());

               kstHandInput.getDesiredPositionInWorld().set(initialRobotHandPositions.get(robotSide));
               kstHandInput.getDesiredPositionInWorld().add(handAdjustments.get(robotSide));

               kstHandInput.setHasDesiredLinearVelocity(false);
               kstHandInput.getControlFramePositionInEndEffector().set(handControlFramePosition.get(robotSide));

               kstHandInput.getLinearSelectionMatrix().setXSelected(true);
               kstHandInput.getLinearSelectionMatrix().setYSelected(true);
               kstHandInput.getLinearSelectionMatrix().setZSelected(true);
               kstHandInput.getAngularSelectionMatrix().setXSelected(false);
               kstHandInput.getAngularSelectionMatrix().setYSelected(false);
               kstHandInput.getAngularSelectionMatrix().setZSelected(false);

               desiredHandPositions.get(robotSide).set(kstHandInput.getDesiredPositionInWorld());
            }
         }

         kstInput.setTimestamp(System.nanoTime());
         kstInput.setStreamToController(true);

         kstInputPublisher.publish(kstInput);
      }
      else
      {
         if (isInitialized.getValue())
         {
            toolboxStateMessage.setRequestedToolboxState(ToolboxStateMessage.SLEEP);
            toolboxStatePublisher.publish(toolboxStateMessage);

            sendGoHomeMessage = true;
            disableTime = System.currentTimeMillis();
         }

         isInitialized.set(false);
      }

      if (sendGoHomeMessage && System.currentTimeMillis() - disableTime > 2.0)
      {
         goHome();
         sendGoHomeMessage = false;
      }
   }

   private void goHome()
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
   }

   private boolean sendGoHomeMessage = false;
   private long disableTime;

   private void updateUserPositions(List<Point3D> bodyPartLocations)
   {
      Point3DReadOnly chest = keypoints[CHEST];
      Point3DReadOnly leftShoulder = keypoints[L_SHOULDER];
      Point3DReadOnly rightShoulder = keypoints[R_SHOULDER];

      double yaw = - Math.atan2(leftShoulder.getX() - rightShoulder.getX(), leftShoulder.getY() - rightShoulder.getY());
      currentUserChestPose.getPosition().set(chest);
      currentUserChestPose.getOrientation().setToYawOrientation(yaw);

      Point3DReadOnly leftHand = keypoints[L_HAND];
      Point3DReadOnly rightHand = keypoints[R_HAND];
      currentUserHandPositions.get(RobotSide.LEFT).set(leftHand);
      currentUserHandPositions.get(RobotSide.RIGHT).set(rightHand);
   }

   public void updateRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      this.robotConfigurationData.set(robotConfigurationData);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }
}