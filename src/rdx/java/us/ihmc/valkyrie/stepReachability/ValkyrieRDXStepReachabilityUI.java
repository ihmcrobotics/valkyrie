package us.ihmc.valkyrie.stepReachability;

import com.badlogic.gdx.Gdx;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.modelFileLoaders.SdfLoader.SDFGraphics3DObject;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieRDXStepReachabilityUI
{
   public ValkyrieRDXStepReachabilityUI()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

      SDFGraphics3DObject.DEFAULT_APPEARANCE = YoAppearance.LightGray();
      robotModel.setTransparency(0.2);
      robotModel.setRobotDefinitionMutator(robotModel.getRobotDefinitionMutator().andThen(RobotDefinitionTools.jointLimitRemover()));

      SharedMemoryMessager messager = new SharedMemoryMessager(StepReachabilityMessagerAPI.API);
      messager.startMessager();

      RDXBaseUI baseUI = new RDXBaseUI("Step Reachability UI");

      RDXStepReachabilityPanel stepReachabilityPanel = new RDXStepReachabilityPanel();

      baseUI.getImGuiPanelManager().addPanel(stepReachabilityPanel.getWindowName(), stepReachabilityPanel::render);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            stepReachabilityPanel.create(robotModel, messager);
            baseUI.getPrimaryScene().addRenderableProvider(stepReachabilityPanel);
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            stepReachabilityPanel.dispose();
            baseUI.dispose();
         }
      });
   }

   public void exit()
   {
      Gdx.app.exit();
   }

   public static void main(String[] args)
   {
      new ValkyrieRDXStepReachabilityUI();
   }
}
