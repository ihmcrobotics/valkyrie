package us.ihmc.valkyrie.rdx.apps;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneGraphUI;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySet;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXROS2BehaviorTree;
import us.ihmc.rdx.ui.footstepPlanner.RDXFootstepPlannerLogViewer;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.ui.tools.RDXROS2StatsPanel;
import us.ihmc.rdx.ui.vr.RDXVRModeManager;
import us.ihmc.rdx.ui.yo.CommonYoVariableCollections;
import us.ihmc.rdx.ui.yo.ImPlotYoGraphPanel;
import us.ihmc.rdx.ui.yo.RDXYoVariableClientPanel;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.valkyrie.ValkyrieKinematicsCollisionModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieKinematicsStreamingToolboxParameters;
import us.ihmc.valkyrie.parameters.ValkyrieRetargetingParameters;
import us.ihmc.valkyrie.rdx.ValkyrieRDXPerceptionVisualizersPanel;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

import java.util.Collections;

public class ValkyrieRDXOperatorUI
{
   private final RDXTeleoperationManager teleoperationPanel;
   private final RDXYoVariableClientPanel yoVariableClientPanel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper vrROS2ControllerHelper;
   private final RDXVRModeManager vrModeManager;
   private final ROS2PeerClockOffsetEstimator peerClockEstimator;
   private final ValkyrieRetargetingParameters retargetingParameters;
   private final ImPlotYoGraphPanel yoGraphUI;
   private final ROS2Helper ros2Helper;
   private final RDXBaseUI baseUI;

   private final ValkyrieRDXPerceptionVisualizersPanel visualizers;
   private RDXSceneGraphUI sceneGraphUI;
   private RDXROS2BehaviorTree behaviorTreeUI;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private RDXFootstepPlannerLogViewer footstepPlannerLogViewer;

   public ValkyrieRDXOperatorUI()
   {
      ValkyrieRobotModel robotModel = createRobotModel();

      ROS2Node ros2Node = new ROS2NodeBuilder().build("operator_ui");
      ros2Helper = new ROS2Helper(ros2Node);
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      peerClockEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);

      baseUI = new RDXBaseUI("Valkyrie Operator UI");

      visualizers = new ValkyrieRDXPerceptionVisualizersPanel(ros2Node, syncedRobot, peerClockEstimator);

      yoVariableClientPanel = new RDXYoVariableClientPanel("Controller",
                                                           NetworkParameters.getHost(NetworkParameterKeys.robotController),
                                                           DataServerSettings.DEFAULT_PORT);
      baseUI.getImGuiPanelManager().addPanel(yoVariableClientPanel);

      ValkyrieSimulationCollisionModel selectionCollisionModel = new ValkyrieSimulationCollisionModel(robotModel.getJointMap(), false);
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

      vrROS2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      vrModeManager = new RDXVRModeManager();
      retargetingParameters = new ValkyrieRetargetingParameters(robotModel.getJointMap(), syncedRobot.getFullRobotModel());

      yoGraphUI = new ImPlotYoGraphPanel("Valkyrie Variables", 1000);
      CommonYoVariableCollections.addYoVariablesToPanel(yoGraphUI);
      baseUI.getImGuiPanelManager().addPanel(yoGraphUI.getWindowName(), yoGraphUI::renderImGuiWidgetsGraphPanel);

      baseUI.getImGuiPanelManager().addPanel(new RDXROS2StatsPanel());

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(3.0, 1.0, 2.5);

            teleoperationPanel.create(baseUI);

            visualizers.create(baseUI);

            yoGraphUI.create();

            referenceFrameLibrary = new ReferenceFrameLibrary();
            referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
            referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());
            for (RobotSide side: RobotSide.values)
            {
               referenceFrameLibrary.addAll(Collections.singleton(syncedRobot.getReferenceFrames().getHandZUpFrame(side)));
            }

            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI);
            referenceFrameLibrary.addDynamicCollection(sceneGraphUI.getSceneGraph().asNewDynamicReferenceFrameCollection());

            footstepPlannerLogViewer = new RDXFootstepPlannerLogViewer(baseUI, robotModel);

            WorkspaceResourceDirectory treeFilesDirectory = new WorkspaceResourceDirectory(getClass(), "/behaviorTrees");
            behaviorTreeUI = new RDXROS2BehaviorTree(treeFilesDirectory,
                                                     robotModel,
                                                     syncedRobot,
                                                     peerClockEstimator,
                                                     selectionCollisionModel,
                                                     baseUI,
                                                     baseUI.getPrimary3DPanel(),
                                                     referenceFrameLibrary,
                                                     ros2ControllerHelper);
            behaviorTreeUI.createAndSetupDefault(baseUI);

            ValkyrieRobotModel robotIKModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRosControlController.VERSION);
            ValkyrieKinematicsCollisionModel kinematicsCollisionModel = new ValkyrieKinematicsCollisionModel(robotIKModel.getJointMap());
            kinematicsCollisionModel.setEnableConservativeCollisions(true);
            robotIKModel.setHumanoidRobotKinematicsCollisionModel(kinematicsCollisionModel);
            ValkyrieKinematicsStreamingToolboxParameters kstParameters = new ValkyrieKinematicsStreamingToolboxParameters();
            kstParameters.setDefault(false, robotIKModel);
            vrModeManager.create(baseUI,
                                 syncedRobot,
                                 visualizers,
                                 vrROS2ControllerHelper,
                                 retargetingParameters,
                                 true,
                                 kstParameters,
                                 false);

            baseUI.getVRManager().getTeleporter().setLinkBeforeCameraReferenceFrame(syncedRobot.getReferenceFrames().getChestFrame());
            baseUI.getVRManager().getTeleporter().setRobotCameraReferenceFrames(syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.LEFT), syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT));

         }

         @Override
         public void render()
         {
            syncedRobot.update();
            teleoperationPanel.update();
            vrModeManager.update();
            footstepPlannerLogViewer.update();

            sceneGraphUI.update();
            behaviorTreeUI.update();

            visualizers.update();

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
            footstepPlannerLogViewer.destroy();
            peerClockEstimator.destroy();
            behaviorTreeUI.destroy();
            yoVariableClientPanel.destroy();
            yoGraphUI.destroy();
            teleoperationPanel.destroy();
            visualizers.destroy();
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
