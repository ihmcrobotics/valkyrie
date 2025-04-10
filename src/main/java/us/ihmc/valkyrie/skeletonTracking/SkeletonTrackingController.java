package us.ihmc.valkyrie.skeletonTracking;

import controller_msgs.msg.dds.RobotConfigurationData;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.perception.ZEDBodyTracking;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotModels.FullRobotModelUtils.getAllJointsExcludingHands;

public class SkeletonTrackingController extends ToolboxController
{
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final HumanoidReferenceFrames referenceFrames;
   private final RobotConfigurationDataBasedUpdater robotConfigurationDataBasedUpdater = new RobotConfigurationDataBasedUpdater();

   private final YoBoolean enableSkeletonTracking = new YoBoolean("enableSkeletonTracking", registry);
   private final YoBoolean isInitialized = new YoBoolean("isInitialized", registry);

   private final FullHumanoidRobotModel fullRobotModel;
   private final FloatingJointBasics rootJoint;
   private final OneDoFJointBasics[] oneDoFJoints;

   private final KinematicsStreamingToolboxConfigurationMessage kstConfiguration = new KinematicsStreamingToolboxConfigurationMessage();
   private final KinematicsStreamingToolboxInputMessage kstInput = new KinematicsStreamingToolboxInputMessage();

   private final YoFramePoint3D[] keypoints = new YoFramePoint3D[10];

   private final ZEDBodyTracking zedBodyTracking = new ZEDBodyTracking();
   private final FramePoint3D desiredCoM = new FramePoint3D();

   public SkeletonTrackingController(FullHumanoidRobotModel fullRobotModel,
                                     StatusMessageOutputManager statusOutputManager,
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

      for (int i = 0; i < keypoints.length; i++)
      {
         keypoints[i] = new YoFramePoint3D("keypoint" + i, ReferenceFrame.getWorldFrame(), registry);
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), new YoGraphicPosition("keypoint" + i + "position", keypoints[i], 0.04, YoAppearance.Red()));
      }

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
      if (!enableSkeletonTracking.getValue())
      {
         isInitialized.set(false);
         return;
      }

      for (int i = 0; i < keypoints.length; i++)
      {
         keypoints[i].setToNaN();
      }

      List<Point3D> bodyPartLocations = zedBodyTracking.getBodyPartLocations();
      for (int i = 0; i < bodyPartLocations.size(); i++)
      {
         keypoints[i].set(bodyPartLocations.get(i));
      }

      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if (robotConfigurationData != null)
      {
         robotConfigurationDataBasedUpdater.updateRobotConfiguration(rootJoint, oneDoFJoints);
         referenceFrames.updateFrames();

         if (!isInitialized.getValue())
         { // initialize here
            isInitialized.set(true);

            desiredCoM.setToZero(referenceFrames.getMidFeetZUpFrame());
            desiredCoM.changeFrame(ReferenceFrame.getWorldFrame());
         }
      }

      // take in latest skeleton data and configure KST message
      if (!isInitialized.getValue())
      {
         // Keep CoM in mid-feet zup
         kstInput.setUseCenterOfMassInput(true);
         kstInput.getCenterOfMassInput().getDesiredPositionInWorld().set(desiredCoM);
         kstInput.getCenterOfMassInput().setHasDesiredLinearVelocity(false);
         kstInput.getCenterOfMassInput().getSelectionMatrix().setXSelected(true);
         kstInput.getCenterOfMassInput().getSelectionMatrix().setYSelected(true);
         kstInput.getCenterOfMassInput().getSelectionMatrix().setZSelected(false);

         // Clear previous inputs
         kstInput.getInputs().clear();

         // Chest orientation
         KinematicsToolboxRigidBodyMessage kstChestInput = kstInput.getInputs().add();
         kstChestInput.setEndEffectorHashCode(fullRobotModel.getChest().hashCode());

//         kstChestInput.getDesiredOrientationInWorld().set(); // TODO
         kstChestInput.setHasDesiredLinearVelocity(false);

         kstChestInput.getLinearSelectionMatrix().setXSelected(false);
         kstChestInput.getLinearSelectionMatrix().setYSelected(false);
         kstChestInput.getLinearSelectionMatrix().setZSelected(false);
         kstChestInput.getAngularSelectionMatrix().setXSelected(false);
         kstChestInput.getAngularSelectionMatrix().setYSelected(false);
         kstChestInput.getAngularSelectionMatrix().setZSelected(true);

         // Pelvis height
         KinematicsToolboxRigidBodyMessage kstPelvisInput = kstInput.getInputs().add();
         kstPelvisInput.setEndEffectorHashCode(fullRobotModel.getPelvis().hashCode());

//         kstPelvisInput.getDesiredPositionInWorld().set(); // TODO
         kstPelvisInput.setHasDesiredLinearVelocity(false);

         kstPelvisInput.getLinearSelectionMatrix().setXSelected(false);
         kstPelvisInput.getLinearSelectionMatrix().setYSelected(false);
         kstPelvisInput.getLinearSelectionMatrix().setZSelected(true);
         kstPelvisInput.getAngularSelectionMatrix().setXSelected(false);
         kstPelvisInput.getAngularSelectionMatrix().setYSelected(false);
         kstPelvisInput.getAngularSelectionMatrix().setZSelected(false);

         // Hand positions
         for (RobotSide robotSide : RobotSide.values)
         {
            KinematicsToolboxRigidBodyMessage kstHandInput = kstInput.getInputs().add();
            kstHandInput.setEndEffectorHashCode(fullRobotModel.getHand(robotSide).hashCode());

//            kstHandInput.getDesiredPositionInWorld().set(); // TODO
            kstHandInput.setHasDesiredLinearVelocity(false);

            kstHandInput.getLinearSelectionMatrix().setXSelected(true);
            kstHandInput.getLinearSelectionMatrix().setYSelected(true);
            kstHandInput.getLinearSelectionMatrix().setZSelected(true);
            kstHandInput.getAngularSelectionMatrix().setXSelected(false);
            kstHandInput.getAngularSelectionMatrix().setYSelected(false);
            kstHandInput.getAngularSelectionMatrix().setZSelected(false);
         }

         statusOutputManager.reportStatusMessage(kstInput);
      }
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
