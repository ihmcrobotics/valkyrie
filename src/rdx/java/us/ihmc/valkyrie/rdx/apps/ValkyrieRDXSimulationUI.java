package us.ihmc.valkyrie.rdx.apps;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.rdx.simulation.environment.RDXCustomSceneLoader;
import us.ihmc.rdx.simulation.environment.RDXCustomSceneLoader.RDXDemoScene;
import us.ihmc.rdx.ui.yo.CommonYoVariableCollections;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneGraphUI;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXROS2BehaviorTree;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.ui.tools.RDXROS2StatsPanel;
import us.ihmc.rdx.ui.vr.RDXVRModeManager;
import us.ihmc.rdx.ui.yo.ImPlotYoGraphPanel;
import us.ihmc.rdx.ui.yo.RDXYoVariableClientPanel;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieKinematicsStreamingToolboxParameters;
import us.ihmc.valkyrie.parameters.ValkyrieRetargetingParameters;
import us.ihmc.valkyrie.rdx.ValkyrieRDXPerceptionVisualizersPanel;
import us.ihmc.valkyrie.rdx.ValkyrieRDXProcessManagerPanel;
import us.ihmc.valkyrie.rdx.ValkyrieRDXSimulatedAutonomyProcess;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;

import java.util.Collections;

public class ValkyrieRDXSimulationUI
{
   /**
    * If restarting the UI a lot for testing, sometimes it may help to run ValkyrieKinematicsSimulationTools
    * separately and set this to false.
    */
   private static final boolean START_KINEMATICS_SIMULATION = Boolean.parseBoolean(System.getProperty("start.kinematics.simulation", "true"));
   private static final RDXDemoScene DEMO_SCENE = RDXDemoScene.FLAT_GROUND;

   private final RDXBaseUI baseUI;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Helper ros2Helper;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2PeerClockOffsetEstimator peerClockEstimator;
   private final ValkyrieRDXPerceptionVisualizersPanel perceptionVisualizersPanel;
   private final RDXTeleoperationManager teleoperationPanel;
   private final RDXYoVariableClientPanel yoVariableClientPanel;
   private final ImPlotYoGraphPanel yoGraphUI;
   private final RDXEnvironmentBuilder environmentBuilder;
   private final ROS2ControllerHelper vrROS2ControllerHelper;
   private final RDXVRModeManager vrModeManager;
   private final ValkyrieRetargetingParameters retargetingParameters;
   private final ValkyrieRDXProcessManagerPanel processManagerPanel;
   private final Throttler sceneUpdate = new Throttler().setFrequency(1.0);

   /**
    * This UI replicates the on-robot and UI scene graph nodes, talking to each other locally,
    * in order to simulate the functionality on the real robot.
    */
   private RDXSceneGraphUI sceneGraphUI;
   private ReferenceFrameLibrary referenceFrameLibrary;

   // ValkyrieAutonomyProcess using simulated sensors
   private final ValkyrieRDXSimulatedAutonomyProcess autonomyProcess;
   private RDXROS2BehaviorTree behaviorTreeUI;
   private RDXCustomSceneLoader sceneLoader;

   private static ValkyrieRobotModel createRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }

   public ValkyrieRDXSimulationUI()
   {
      ValkyrieRobotModel robotModel = createRobotModel();
      ROS2Node ros2Node = new ROS2NodeBuilder().build("simulation_ui");
      RealtimeROS2Node realtimeRos2Node = new ROS2NodeBuilder().buildRealtime("simulation_ui_realtime");

      ros2Helper = new ROS2Helper(ros2Node);
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      realtimeRos2Node.spin();

      peerClockEstimator = new ROS2PeerClockOffsetEstimator(ros2Node);
      baseUI = new RDXBaseUI("Valkyrie Simulation UI");

      autonomyProcess = new ValkyrieRDXSimulatedAutonomyProcess(robotModel);

      perceptionVisualizersPanel = new ValkyrieRDXPerceptionVisualizersPanel(ros2Node, syncedRobot, peerClockEstimator);

      yoVariableClientPanel = new RDXYoVariableClientPanel("Controller",
                                                           NetworkParameters.getHost(NetworkParameterKeys.robotController),
                                                           DataServerSettings.DEFAULT_PORT);
      baseUI.getImGuiPanelManager().addPanel(yoVariableClientPanel);

      processManagerPanel = new ValkyrieRDXProcessManagerPanel(robotModel);
      baseUI.getImGuiPanelManager().addPanel(processManagerPanel);
      processManagerPanel.getKinematicsSimulationProcess().getKinematicsSimulationParameters().setCreateYoVariableServer(true);

      ValkyrieSimulationCollisionModel selectionCollisionModel = new ValkyrieSimulationCollisionModel(robotModel.getJointMap(), false);
      selectionCollisionModel.setCollidableHelper(new CollidableHelper(), robotModel.getJointMap().getModelName(), "ground");
      RobotCollisionModel kinematicsCollisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();

      teleoperationPanel = new RDXTeleoperationManager(new CommunicationHelper(robotModel, ros2Node),
                                                       kinematicsCollisionModel,
                                                       selectionCollisionModel,
                                                       yoVariableClientPanel.getYoVariableClientHelper());
      baseUI.getImGuiPanelManager().addPanel(teleoperationPanel);

      vrROS2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      vrModeManager = new RDXVRModeManager();
      retargetingParameters = new ValkyrieRetargetingParameters(robotModel.getJointMap(), syncedRobot.getFullRobotModel());

      environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
      baseUI.getImGuiPanelManager().addPanel(environmentBuilder);

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

            autonomyProcess.create(baseUI.getPrimaryScene());

            environmentBuilder.create();
            teleoperationPanel.create(baseUI);

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

            WorkspaceResourceDirectory treeFilesDirectory = new WorkspaceResourceDirectory(getClass(), "/behaviorTrees");
            behaviorTreeUI = new RDXROS2BehaviorTree(treeFilesDirectory,
                                                     robotModel,
                                                     syncedRobot,
                                                     new ROS2PeerClockOffsetEstimator(ros2Node),
                                                     selectionCollisionModel,
                                                     baseUI,
                                                     baseUI.getPrimary3DPanel(),
                                                     referenceFrameLibrary,
                                                     ros2ControllerHelper);
            behaviorTreeUI.createAndSetupDefault(baseUI);

            perceptionVisualizersPanel.create(baseUI);

            ValkyrieKinematicsStreamingToolboxParameters ValkyrieKSTParameters = new ValkyrieKinematicsStreamingToolboxParameters();
            ValkyrieKSTParameters.setDefault(false, robotModel);

            vrModeManager.create(baseUI,
                                 syncedRobot,
                                 perceptionVisualizersPanel,
                                 vrROS2ControllerHelper,
                                 retargetingParameters,
                                 true,
                                 ValkyrieKSTParameters,
                                 false);

            baseUI.getVRManager().getTeleporter().setLinkBeforeCameraReferenceFrame(syncedRobot.getReferenceFrames().getChestFrame());
            baseUI.getVRManager().getTeleporter().setRobotCameraReferenceFrames(syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.LEFT), syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT));

            if (START_KINEMATICS_SIMULATION)
            {
               processManagerPanel.getKinematicsSimulationProcess().start();
            }

            sceneLoader = new RDXCustomSceneLoader(sceneGraphUI, ros2Helper, syncedRobot);
            sceneLoader.loadCustomScene(DEMO_SCENE);
            environmentBuilder.loadEnvironment(sceneLoader.getEnvironmentName(DEMO_SCENE));
         }

         @Override
         public void render()
         {
            syncedRobot.update();
            perceptionVisualizersPanel.update();
            teleoperationPanel.update();
            vrModeManager.update();

            autonomyProcess.render();

            sceneGraphUI.update();
            behaviorTreeUI.update();

            if (DEMO_SCENE != RDXDemoScene.FLAT_GROUND && DEMO_SCENE != RDXDemoScene.ROUGH_TERRAIN)
            {
               if (sceneUpdate.run())
               {
                  sceneLoader.trackEnvironment(environmentBuilder.getAllObjects());
                  sceneLoader.moveManipulatedObject();
               }

            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            processManagerPanel.dispose();
            yoVariableClientPanel.destroy();
            yoGraphUI.destroy();
            autonomyProcess.close();
            behaviorTreeUI.destroy();
            teleoperationPanel.destroy();
            environmentBuilder.destroy();
            vrModeManager.destroy();
            perceptionVisualizersPanel.destroy();
            ros2Node.destroy();
            realtimeRos2Node.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new ValkyrieRDXSimulationUI();
   }
}
