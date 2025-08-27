package us.ihmc.valkyrie.rdx.apps;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.communication.ros2log.ROS2LogRecord;
import us.ihmc.communication.ros2log.ROS2LogSerialization;
import us.ihmc.communication.ros2log.ROS2LogTimeSource;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXCustomSceneLoader;
import us.ihmc.rdx.simulation.environment.RDXCustomSceneLoader.RDXDemoScene;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.teleoperation.RDXTeleoperationManager;
import us.ihmc.rdx.ui.vr.RDXVRMode;
import us.ihmc.rdx.ui.vr.RDXVRModeManager;
import us.ihmc.rdx.ui.yo.RDXYoVariableClientPanel;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.simulation.collision.CollidableHelper;
import us.ihmc.valkyrie.parameters.ValkyrieKinematicsStreamingToolboxParameters;
import us.ihmc.valkyrie.parameters.ValkyrieRetargetingParameters;
import us.ihmc.valkyrie.rdx.ValkyrieRDXPerceptionVisualizersPanel;
import us.ihmc.valkyrie.rdx.ValkyrieRDXProcessManagerPanel;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieSimulationCollisionModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

import java.util.ArrayList;
import java.util.List;

public class ValkyrieRDXTeleoperationUI
{
   private static final boolean USE_SIMULATION = true; // Controlling Sim or Real Robot
   private static final boolean RECORD_VR_MOTION = true;
   private static final RDXDemoScene DEMO_SCENE = RDXDemoScene.FLAT_GROUND;

   private final RDXBaseUI baseUI;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ValkyrieRDXPerceptionVisualizersPanel perceptionVisualizersPanel;
   private final RDXTeleoperationManager teleoperationPanel;
   private final RDXYoVariableClientPanel yoVariableClientPanel;
   private final RDXEnvironmentBuilder environmentBuilder;
   private final ROS2ControllerHelper vrROS2ControllerHelper;
   private final RDXVRModeManager vrModeManager;

   private final ValkyrieRetargetingParameters retargetingParameters;
   private ValkyrieRDXProcessManagerPanel processManagerPanel;
   private ROS2LogRecord ros2LogRecord;

   private static ValkyrieRobotModel createRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.REAL_ROBOT, ValkyrieRobotVersion.PHYSICAL_REALITY);
   }

   public ValkyrieRDXTeleoperationUI()
   {
      ValkyrieRobotModel robotModel = createRobotModel();
      ROS2Node ros2Node = new ROS2NodeBuilder().build("teleoperation_ui");
      RealtimeROS2Node realtimeRos2Node = new ROS2NodeBuilder().buildRealtime("teleoperation_ui_realtime");

      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      realtimeRos2Node.spin();

      baseUI = new RDXBaseUI("Valkyrie Teleoperation UI");

      perceptionVisualizersPanel = new ValkyrieRDXPerceptionVisualizersPanel(ros2Node, syncedRobot, new ROS2PeerClockOffsetEstimator(ros2Node));

      yoVariableClientPanel = new RDXYoVariableClientPanel("Controller",
                                                           NetworkParameters.getHost(NetworkParameterKeys.robotController),
                                                           DataServerSettings.DEFAULT_PORT);
      baseUI.getImGuiPanelManager().addPanel(yoVariableClientPanel);

      if (USE_SIMULATION)
      {
         processManagerPanel = new ValkyrieRDXProcessManagerPanel(robotModel);
         baseUI.getImGuiPanelManager().addPanel(processManagerPanel);
         processManagerPanel.getKinematicsSimulationProcess().getKinematicsSimulationParameters().setCreateYoVariableServer(true);
      }

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

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            if (USE_SIMULATION)
            {
               baseUI.create(RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL, RDXSceneLevel.GROUND_TRUTH);
            }
            else
            {
               baseUI.create();
            }

            environmentBuilder.create();
            teleoperationPanel.create(baseUI);

            perceptionVisualizersPanel.create(baseUI);

            ValkyrieKinematicsStreamingToolboxParameters ValkyrieKSTParameters = new ValkyrieKinematicsStreamingToolboxParameters();
            ValkyrieKSTParameters.setDefault(false, robotModel);

            vrModeManager.create(baseUI,
                                 syncedRobot,
                                 perceptionVisualizersPanel,
                                 vrROS2ControllerHelper,
                                 retargetingParameters,
                                 USE_SIMULATION,
                                 ValkyrieKSTParameters,
                                 USE_SIMULATION && RECORD_VR_MOTION,
                                 null,
                                 null);

            baseUI.getVRManager().getTeleporter().setLinkBeforeCameraReferenceFrame(syncedRobot.getReferenceFrames().getChestFrame());
            baseUI.getVRManager().getTeleporter().setRobotCameraReferenceFrames(syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.LEFT), syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT));

            if (USE_SIMULATION)
            {
               processManagerPanel.getKinematicsSimulationProcess().start();
            }

            environmentBuilder.loadEnvironment(RDXCustomSceneLoader.getEnvironmentName(DEMO_SCENE));

            baseUI.getVRManager().enableVR();
            vrModeManager.setMode(RDXVRMode.WHOLE_BODY_IK_STREAMING);
            if (RECORD_VR_MOTION)
            {
               List<ROS2Topic<?>> topics = new ArrayList<>();
               { // KST output
                  topics.add(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));
               }
               ROS2LogTimeSource timeSource = ROS2LogTimeSource.SYSTEM;
               ROS2LogSerialization serialization = ROS2LogSerialization.JSON;
               // Assign to the member variable
               ros2LogRecord = new ROS2LogRecord(robotModel.getSimpleRobotName(), topics, timeSource, serialization);
            }
         }

         @Override
         public void render()
         {
            syncedRobot.update();
            perceptionVisualizersPanel.update();
            teleoperationPanel.update();
            vrModeManager.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            if (USE_SIMULATION)
            {
               processManagerPanel.dispose();
            }
            yoVariableClientPanel.destroy();
            teleoperationPanel.destroy();
            environmentBuilder.destroy();
            vrModeManager.destroy();
            perceptionVisualizersPanel.destroy();
            ros2Node.destroy();
            realtimeRos2Node.destroy();
            baseUI.dispose();
            if (ros2LogRecord != null)
            {
               ros2LogRecord.destroy();
            }
         }
      });
   }

   public static void main(String[] args)
   {
      new ValkyrieRDXTeleoperationUI();
   }
}
