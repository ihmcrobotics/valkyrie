package us.ihmc.valkyrie.rdx;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTreeExecutor;
import us.ihmc.behaviors.behaviorTree.ros2.ROS2BehaviorTree;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.detections.DetectionManager;
import us.ihmc.perception.sceneGraph.ros2.ROS2SceneGraph;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneGraphUI;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXROS2BehaviorTree;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2RobotVisualizer;
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
import us.ihmc.commons.thread.Throttler;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.RestartableThrottledThread;
import us.ihmc.valkyrie.ValkyrieCollisionBasedSelectionModel;
import us.ihmc.valkyrie.ValkyrieKinematicsCollisionModel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieKinematicsStreamingToolboxParameters;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

import java.util.Collections;

public class ValkyrieRDXSimulationUI
{
   /**
    * To toggle the following flags, go to Run Configuration, and use VM arguments with '-D' extension on the property name as: [-Dargument.name=true]
    * For example, to enable height maps, use: [-Denable.height.map=true]
    */
   private static final boolean ENABLE_ZED = Boolean.parseBoolean(System.getProperty("enable.zed", "true"));
   private static final boolean ENABLE_REALSENSE = Boolean.parseBoolean(System.getProperty("enable.realsense", "true"));
   /**
    * If restarting the UI a lot for testing, sometimes it may help to run NadiaKinematicsSimulation
    * separately and set this to false.
    */
   private static final boolean START_KINEMATICS_SIMULATION = Boolean.parseBoolean(System.getProperty("start.kinematics.simulation", "true"));

   private final RDXBaseUI baseUI;
   private ROS2SyncedRobotModel syncedRobot;
   private final ROS2Helper ros2Helper;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private RDXROS2RobotVisualizer robotGlobalVisualizer;
   private final RDXPerceptionVisualizersPanel perceptionVisualizersPanel;
   private final RDXTeleoperationManager teleoperationPanel;
   private final RDXYoVariableClientPanel yoVariableClientPanel;
   private final ImPlotYoGraphPanel yoGraphUI;
   private final RDXEnvironmentBuilder environmentBuilder;
   private final ValkyrieRDXProcessManagerPanel processManagerPanel;
   private final ROS2ControllerHelper vrROS2ControllerHelper;
   private final RDXVRModeManager vrModeManager;
   private final ValkyrieRetargetingParameters retargetingParameters;
   private RDXHighLevelDepthSensorSimulator d455Simulator;
   private RDXHighLevelDepthSensorSimulator zed2Simulator;
   /** Simulate an update rate more similar to what it would be on the robot. */
   private final Throttler perceptionThottler = new Throttler().setFrequency(ROS2BehaviorTree.SYNC_FREQUENCY);
   /**
    * This UI replicates the on-robot and UI scene graph nodes, talking to each other locally,
    * in order to simulate the functionality on the real robot.
    */
   private DetectionManager detectionManager;
   private ROS2SceneGraph onRobotSceneGraph;
   private RDXSceneGraphUI sceneGraphUI;
   private RestartableThrottledThread behaviorTreeThread;
   private ROS2BehaviorTreeExecutor onRobotBehaviorTree;
   private RDXROS2BehaviorTree behaviorTreeUI;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private final ValkyrieCollisionBasedSelectionModel selectionCollisionModel;

   public ValkyrieRDXSimulationUI()
   {
      ValkyrieRobotModel robotModel = createRobotModel();
      ROS2Node ros2Node = new ROS2NodeBuilder().build("simulation_ui");
      RealtimeROS2Node realtimeRos2Node = new ROS2NodeBuilder().buildRealtime("simulation_ui_realtime");

      ros2Helper = new ROS2Helper(ros2Node);
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      realtimeRos2Node.spin();
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      baseUI = new RDXBaseUI("Valkyrie Simulation UI");

      perceptionVisualizersPanel = new RDXPerceptionVisualizersPanel();

      yoVariableClientPanel = new RDXYoVariableClientPanel("Controller",
                                                           NetworkParameters.getHost(NetworkParameterKeys.robotController),
                                                           DataServerSettings.DEFAULT_PORT);
      baseUI.getImGuiPanelManager().addPanel(yoVariableClientPanel);

      processManagerPanel = new ValkyrieRDXProcessManagerPanel(robotModel);
      baseUI.getImGuiPanelManager().addPanel(processManagerPanel);

      selectionCollisionModel = new ValkyrieCollisionBasedSelectionModel(robotModel.getRobotVersion(), robotModel.getJointMap());
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
      baseUI.getImGuiPanelManager().addPanel(yoGraphUI.getWindowName(), yoGraphUI::renderImGuiWidgetsGraphPanel);

      baseUI.getImGuiPanelManager().addPanel(new RDXROS2StatsPanel());

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(3.0, 1.0, 2.5);

            environmentBuilder.create();
            teleoperationPanel.create(baseUI);

            yoGraphUI.create();

            onRobotSceneGraph = new ROS2SceneGraph(ros2Helper);
            detectionManager = new DetectionManager(ros2Node);

            referenceFrameLibrary = new ReferenceFrameLibrary();
            referenceFrameLibrary.addAll(Collections.singleton(ReferenceFrame.getWorldFrame()));
            referenceFrameLibrary.addAll(syncedRobot.getReferenceFrames().getCommonReferenceFrames());

            sceneGraphUI = new RDXSceneGraphUI(ros2Helper, baseUI);
            referenceFrameLibrary.addDynamicCollection(sceneGraphUI.getSceneGraph().asNewDynamicReferenceFrameCollection());

            onRobotBehaviorTree = new ROS2BehaviorTreeExecutor(ros2ControllerHelper,
                                                               robotModel,
                                                               syncedRobot,
                                                               new ROS2PeerClockOffsetEstimator(ros2Node),
                                                               referenceFrameLibrary,
                                                               sceneGraphUI.getSceneGraph(),
                                                               detectionManager);

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

            RDXROS2RobotVisualizer robotVisualizer = new RDXROS2RobotVisualizer(ros2Helper, syncedRobot);
            ValkyrieRDXROS2InteractableSensors interactableSensors = new ValkyrieRDXROS2InteractableSensors(baseUI,
                                                                                                            ros2Helper,
                                                                                                            syncedRobot,
                                                                                                            syncedRobot.getReferenceFrames(),
                                                                                                            robotVisualizer);
            interactableSensors.setupZED2i();
            interactableSensors.setupRealsenseD455();
            perceptionVisualizersPanel.addVisualizer(robotGlobalVisualizer = robotVisualizer);
            robotGlobalVisualizer.setActive(true);

            perceptionVisualizersPanel.create(baseUI);

            d455Simulator = RDXSimulatedSensorFactory.createChestD455ForMapSense(syncedRobot);
            d455Simulator.setSensorEnabled(ENABLE_REALSENSE);
            d455Simulator.setupForROS2ImageMessages(ros2Node, PerceptionAPI.D455_DEPTH_IMAGE, PerceptionAPI.D455_COLOR_IMAGE);
            d455Simulator.setupForROS2PointCloud(ros2Node, ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class));
            d455Simulator.setUseSensorColor(true);
            d455Simulator.setRenderPointCloudDirectly(true);
            d455Simulator.setRenderDepthVideoDirectly(false);
            d455Simulator.setPublishPointCloudROS2(false);
            d455Simulator.setPublishDepthImageMessageROS2(true);
            d455Simulator.setPublishColorImageMessageROS2(true);
            baseUI.getImGuiPanelManager().addPanel(d455Simulator);
            baseUI.getPrimaryScene().addRenderableProvider(d455Simulator::getRenderables);

            zed2Simulator = RDXSimulatedSensorFactory.createChestZED2ForObjectDetection(syncedRobot);
            zed2Simulator.setSensorEnabled(ENABLE_ZED);
            zed2Simulator.setupForROS2ImageMessages(ros2Node, PerceptionAPI.ZED_DEPTH, PerceptionAPI.ZED_COLOR_IMAGES.get(RobotSide.RIGHT));
            zed2Simulator.setupForROS2PointCloud(ros2Node, ROS2Tools.IHMC_ROOT.withTypeName(StereoVisionPointCloudMessage.class));
            zed2Simulator.setUseSensorColor(true);
            zed2Simulator.setRenderPointCloudDirectly(true);
            zed2Simulator.setRenderDepthVideoDirectly(false);
            zed2Simulator.setPublishPointCloudROS2(false);
            zed2Simulator.setPublishDepthImageMessageROS2(true);
            zed2Simulator.setPublishColorImageMessageROS2(true);
            baseUI.getImGuiPanelManager().addPanel(zed2Simulator);
            baseUI.getPrimaryScene().addRenderableProvider(zed2Simulator::getRenderables);

            ValkyrieRobotModel robotIKModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRosControlController.VERSION);
            ValkyrieKinematicsCollisionModel kinematicsCollisionModel = new ValkyrieKinematicsCollisionModel(robotIKModel.getJointMap());
            kinematicsCollisionModel.setEnableConservativeCollisions(true);
            robotIKModel.setHumanoidRobotKinematicsCollisionModel(kinematicsCollisionModel);
            ValkyrieKinematicsStreamingToolboxParameters kstParameters = new ValkyrieKinematicsStreamingToolboxParameters();
            kstParameters.setDefault(true, robotIKModel);

            vrModeManager.create(baseUI,
                                 syncedRobot,
                                 robotGlobalVisualizer,
                                 vrROS2ControllerHelper,
                                 retargetingParameters,
                                 sceneGraphUI.getSceneGraph(),
                                 true,
                                 kstParameters);

            if (START_KINEMATICS_SIMULATION)
            {
               processManagerPanel.getKinematicsSimulationProcess().start();
            }

            environmentBuilder.loadEnvironment("HarderTerrain.json");
         }

         @Override
         public void render()
         {
            syncedRobot.update();
            perceptionVisualizersPanel.update();
            teleoperationPanel.update();
            vrModeManager.update();

            d455Simulator.render(baseUI.getPrimaryScene());
            zed2Simulator.render(baseUI.getPrimaryScene());

            boolean runPerception = perceptionThottler.run();

            if (runPerception)
            {
               onRobotSceneGraph.updateSubscription();

               onRobotSceneGraph.updateDetections(detectionManager);
               onRobotSceneGraph.updateOnRobotOnly(syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT));
               onRobotSceneGraph.updatePublication();
            }

            if (behaviorTreeThread == null)
            {
               behaviorTreeThread = new RestartableThrottledThread(ROS2BehaviorTreeExecutor.class.getSimpleName(),
                                                                   ROS2BehaviorTree.SYNC_FREQUENCY,
                                                                   DefaultExceptionHandler.MESSAGE_AND_STACKTRACE,
                                                                   true,
                                                                   onRobotBehaviorTree::update);
               behaviorTreeThread.start();
            }

            sceneGraphUI.update();
            behaviorTreeUI.update();

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
            yoVariableClientPanel.destroy();
            yoGraphUI.destroy();
            if (d455Simulator != null)
               d455Simulator.dispose();
            if (zed2Simulator != null)
               zed2Simulator.dispose();
            teleoperationPanel.destroy();
            processManagerPanel.dispose();
            environmentBuilder.destroy();
            vrModeManager.destroy();
            perceptionVisualizersPanel.destroy();
            behaviorTreeThread.stop();
            behaviorTreeUI.destroy();
            ros2Node.destroy();
            realtimeRos2Node.destroy();
            baseUI.dispose();
         }
      });
   }

   private static ValkyrieRobotModel createRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);
   }

   public static void main(String[] args)
   {
      new ValkyrieRDXSimulationUI();
   }
}
