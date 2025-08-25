package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.interactable.RDXInteractableRealsenseD455;
import us.ihmc.rdx.ui.interactable.RDXInteractableZED2i;

public class ValkyrieRDXROS2InteractableSensors
{
   private final RDXBaseUI baseUI;
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2SyncedRobotModel syncedRobot;
   private final HumanoidReferenceFrames referenceFramesToUseForSensors;
   private final RDXROS2RobotVisualizer robotVisualizer;

   /**
    * @param referenceFramesToUseForSensors In simulation we want to use the ground truth frames rather than state estimator frames.
    */
   public ValkyrieRDXROS2InteractableSensors(RDXBaseUI baseUI,
                                             ROS2PublishSubscribeAPI ros2,
                                             ROS2SyncedRobotModel syncedRobot,
                                             HumanoidReferenceFrames referenceFramesToUseForSensors,
                                             RDXROS2RobotVisualizer robotVisualizer)
   {
      this.baseUI = baseUI;
      this.ros2 = ros2;
      this.syncedRobot = syncedRobot;
      this.referenceFramesToUseForSensors = referenceFramesToUseForSensors;
      this.robotVisualizer = robotVisualizer;
   }

   public void setupRealsenseD455()
   {
      RDXInteractableRealsenseD455 interactableRealsenseD455 = new RDXInteractableRealsenseD455(baseUI.getPrimary3DPanel(),
                                                                                                referenceFramesToUseForSensors.getSteppingCameraFrame(),
                                                                                                syncedRobot.getRobotModel()
                                                                                                           .getSensorInformation()
                                                                                                           .getSteppingCameraTransform());
      interactableRealsenseD455.getInteractableFrameModel()
                               .addRemoteTuning(ros2,
                                                PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                syncedRobot.getRobotModel().getSensorInformation().getSteppingCameraTransform());
      robotVisualizer.attachInteractableFrameModel(interactableRealsenseD455.getInteractableFrameModel());
   }

   public void setupZED2i()
   {
      RDXInteractableZED2i interactableZED2i = new RDXInteractableZED2i(baseUI.getPrimary3DPanel(),
                                                                        referenceFramesToUseForSensors.getExperimentalCameraFrame(),
                                                                        syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraTransform());
      interactableZED2i.getInteractableFrameModel()
                       .addRemoteTuning(ros2,
                                        PerceptionAPI.EXPERIMENTAL_CAMERA_TO_PARENT_TUNING,
                                        syncedRobot.getRobotModel().getSensorInformation().getExperimentalCameraTransform());
      robotVisualizer.attachInteractableFrameModel(interactableZED2i.getInteractableFrameModel());
   }
}