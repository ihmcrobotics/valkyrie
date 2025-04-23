package us.ihmc.valkyrie.rdx;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ImageSensorPublishThread;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXZEDSVORecorderPanel;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneGraphUI;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXRobotCollidable;
import us.ihmc.rdx.ui.affordances.quickATs.RDXQuickATManager;
import us.ihmc.rdx.ui.behavior.tree.RDXROS2BehaviorTree;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlannerLogViewer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.ui.tools.RDXROS2StatsPanel;
import us.ihmc.rdx.ui.vr.RDXVRModeManager;
import us.ihmc.rdx.ui.yo.ImPlotYoGraphPanel;
import us.ihmc.rdx.ui.yo.RDXYoVariableClientPanel;
import us.ihmc.rdx.vr.RDXVRHeadset;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.valkyrie.ValkyrieCollisionBasedSelectionModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.perception.ValkyrieZEDStreamer;
import us.ihmc.zed.global.zed;

import java.util.Collections;

public class ValkyrieRDXOperatorUI
{
   private final FramePoint3D headsetFramePoint = new FramePoint3D();
   private final RDXTeleoperationManager teleoperationPanel;
   private final RDXYoVariableClientPanel yoVariableClientPanel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2PeerClockOffsetEstimator ros2PeerClockOffsetEstimator;
   private final RDXVRModeManager vrModeManager;
   private final ValkyrieRetargetingParameters retargetingParameters;
   private final ImPlotYoGraphPanel yoGraphUI;
   private final ROS2Helper ros2Helper;
   private final RDXBaseUI baseUI;

   private final ZEDImageSensor zedImageSensor;
   private final ImageSensorPublishThread zedPublishThread;

   private final ValkyrieRDXPerceptionVisualizersPanel visualizers;
   private RDXSceneGraphUI sceneGraphUI;
//   private RDXROS2BehaviorTree behaviorTreeUI;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private RDXFootstepPlannerLogViewer footstepPlannerLogViewer;
   private RDXRobotCollidable chestAvoidanceCollidable;
//   private final RDXQuickATManager quickATPanel;
   private RDXZEDSVORecorderPanel zedSVORecorderPanel;

   public ValkyrieRDXOperatorUI()
   {


      ValkyrieRobotModel robotModel = createRobotModel();

      ROS2Node ros2Node = new ROS2NodeBuilder().build("operator_ui");
      ros2Helper = new ROS2Helper(ros2Node);
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      ros2PeerClockOffsetEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

      baseUI = new RDXBaseUI("Valkyrie Operator UI");

      visualizers = new ValkyrieRDXPerceptionVisualizersPanel(baseUI, ros2Node, ros2PeerClockOffsetEstimator, syncedRobot);

      yoVariableClientPanel = new RDXYoVariableClientPanel("Controller",
                                                           NetworkParameters.getHost(NetworkParameterKeys.robotController),
                                                           DataServerSettings.DEFAULT_PORT);
      baseUI.getImGuiPanelManager().addPanel(yoVariableClientPanel);

      ValkyrieCollisionBasedSelectionModel selectionCollisionModel = new ValkyrieCollisionBasedSelectionModel(robotModel.getRobotVersion(), robotModel.getJointMap());
      selectionCollisionModel.setCollidableHelper(new CollidableHelper(), robotModel.getJointMap().getModelName(), "ground");
      RobotCollisionModel kinematicsCollisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();

      teleoperationPanel = new RDXTeleoperationManager(new CommunicationHelper(robotModel, ros2Node),
                                                       kinematicsCollisionModel,
                                                       selectionCollisionModel,
                                                       yoVariableClientPanel.getYoVariableClientHelper());
      baseUI.getImGuiPanelManager().addPanel(teleoperationPanel);

      ImGuiRemoteROS2StoredPropertySet rapidRegionsParameterPanel = new ImGuiRemoteROS2StoredPropertySet(ros2Node,
                                                                                                         new RapidRegionsExtractorParameters(),
                                                                                                         PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS);
      baseUI.getImGuiPanelManager().addPanel(rapidRegionsParameterPanel.createPanel());

//      quickATPanel = new RDXQuickATManager();
//      baseUI.getImGuiPanelManager().addPanel(quickATPanel);

      zedSVORecorderPanel = new RDXZEDSVORecorderPanel(ros2Helper);

      vrModeManager = new RDXVRModeManager();
      retargetingParameters = new ValkyrieRetargetingParameters(robotModel.getJointMap(), syncedRobot.getFullRobotModel());

      yoGraphUI = new ImPlotYoGraphPanel("Nadia Variables", 1000);
      ValkyrieYoVariableCollections.addYoVariablesToPanel(yoGraphUI);
      baseUI.getImGuiPanelManager().addPanel(yoGraphUI.getWindowName(), yoGraphUI::renderImGuiWidgetsGraphPanel);

      baseUI.getImGuiPanelManager().addPanel(new RDXROS2StatsPanel());

      // ZED
      this.zedImageSensor = new ZEDImageSensor(0, ZEDModelData.ZED_MINI, zed.SL_INPUT_TYPE_STREAM, zed.SL_DEPTH_MODE_NEURAL, "192.168.100.21", ValkyrieZEDStreamer.PORT);
      zedImageSensor.run(true);
      zedPublishThread = new ImageSensorPublishThread(ros2Node, zedImageSensor);
      zedPublishThread.addTopic(PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT), ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT), ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);
      zedPublishThread.addTopic(PerceptionAPI.ZED2_DEPTH, ZEDImageSensor.DEPTH_IMAGE_KEY);
      zedPublishThread.startRepeating();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            teleoperationPanel.create(baseUI);

            visualizers.create(baseUI);

            for (RDXRobotCollidable robotCollidable : teleoperationPanel.getAvoidanceCollisionModel().getRobotCollidables())
            {
               if (robotCollidable.getRigidBodyName().equals(robotModel.getJointMap().getChestName()))
               {
                  chestAvoidanceCollidable = robotCollidable;
               }
            }

            yoGraphUI.create();

            referenceFrameLibrary = new ReferenceFrameLibrary();
            referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
            referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());

            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI);
            referenceFrameLibrary.addDynamicCollection(sceneGraphUI.getSceneGraph().asNewDynamicReferenceFrameCollection());

            footstepPlannerLogViewer = new RDXFootstepPlannerLogViewer(baseUI, robotModel);

//            WorkspaceResourceDirectory treeFilesDirectory = new WorkspaceResourceDirectory(getClass(), "/behaviorTrees");
//            behaviorTreeUI = new RDXROS2BehaviorTree(treeFilesDirectory,
//                                                     robotModel,
//                                                     syncedRobot,
//                                                     new ROS2PeerClockOffsetEstimator(ros2Node),
//                                                     selectionCollisionModel,
//                                                     baseUI,
//                                                     baseUI.getPrimary3DPanel(),
//                                                     referenceFrameLibrary,
//                                                     ros2ControllerHelper);
//            behaviorTreeUI.createAndSetupDefault(baseUI);
//
//            quickATPanel.create(teleoperationPanel, sceneGraphUI.getSceneGraph());

            vrModeManager.create(baseUI,
                                 syncedRobot,
                                 visualizers.getRobotVisualizer(),
                                 ros2ControllerHelper,
                                 retargetingParameters,
                                 sceneGraphUI.getSceneGraph(),
                                 true);
            vrModeManager.getHandPlacedFootstepMode().setLocomotionParameters(teleoperationPanel.getLocomotionParameters());

            RDXROS2RobotVisualizer robotVisualizer = new RDXROS2RobotVisualizer(ros2Helper, syncedRobot);
            ValkyrieRDXROS2InteractableSensors interactableSensors = new ValkyrieRDXROS2InteractableSensors(baseUI,
                                                                                                            ros2Helper,
                                                                                                            syncedRobot,
                                                                                                            syncedRobot.getReferenceFrames(),
                                                                                                            robotVisualizer);
            interactableSensors.setupZED2i();
            interactableSensors.setupRealsenseD455();

            baseUI.getPrimaryScene().addRenderableProvider(vrModeManager::getRenderables);
            baseUI.getVRManager().getContext().addVRInputProcessor(vrModeManager::processVRInput);
         }

         @Override
         public void render()
         {
            zedSVORecorderPanel.update();

            syncedRobot.update();
            teleoperationPanel.update();
            vrModeManager.update();
            vrModeManager.render();
            footstepPlannerLogViewer.update();

            sceneGraphUI.update();
//            quickATPanel.update();
//            behaviorTreeUI.update();

            visualizers.update();

            // Hides the graphics that get in the way when operating in VR
            // TODO: Extract this somehow
            RDXVRHeadset headset = baseUI.getVRManager().getContext().getHeadset();
            if (chestAvoidanceCollidable != null && headset.isConnected())
            {
               headsetFramePoint.setToZero(headset.getXForwardZUpHeadsetFrame());
               headsetFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
               headsetFramePoint.subZ(0.08); // So when you are where the head is, above it, it's also gone
               headsetFramePoint.changeFrame(chestAvoidanceCollidable.getShape().getReferenceFrame());
               boolean pointInside = chestAvoidanceCollidable.getShape().isPointInside(headsetFramePoint);
               visualizers.getRobotVisualizer().getHideChest().set(pointInside);
            }

            // Pass robot's camera frames to teleporter
            baseUI.getVRManager()
                  .getTeleporter()
                  .setRobotCameraReferenceFrames(syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.LEFT),
                                                 syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT));

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedPublishThread.stopRepeating();
            try
            {
               zedPublishThread.join();
            }
            catch (InterruptedException e)
            {
               LogTools.error(e);
            }
            zedImageSensor.close();

//            behaviorTreeUI.destroy();
            yoVariableClientPanel.destroy();
            yoGraphUI.destroy();
            teleoperationPanel.destroy();
            visualizers.destroy();
            ros2PeerClockOffsetEstimator.destroy();
            ros2Node.destroy();
            baseUI.dispose();
            vrModeManager.destroy();
         }
      });
   }

   private static ValkyrieRobotModel createRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);
   }

   public static void main(String[] args)
   {
      new ValkyrieRDXOperatorUI();
   }
}
