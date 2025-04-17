package us.ihmc.valkyrie.operatorInterface;

import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Switch;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.humanoidOperatorInterface.DRCOperatorInterface;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieUIParameters;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

public class ValkyrieOperatorUserInterface
{
   public static void main(String[] args)
   {
	   JSAP jsap = new JSAP();

		// having the program argument "--realRobot" toggles this switch
	   Switch runningOnRealRobot = new Switch("runningOnRealRobot").setLongFlag("realRobot");
	   
	   try
	   {
		   jsap.registerParameter(runningOnRealRobot);
		   
		   JSAPResult config = jsap.parse(args);
		   
		   if (config.success())
		   {
			   boolean runningOnRealRobotBoolean = config.getBoolean(runningOnRealRobot.getID());
			   RobotTarget target;

			   if(runningOnRealRobotBoolean)
			   {
				   target = RobotTarget.REAL_ROBOT;
			   }
			   else
			   {
				   target = RobotTarget.SCS;
			   }

			   ValkyrieRobotModel robotModel= new ValkyrieRobotModel(target, ValkyrieRosControlController.VERSION);
			   ValkyrieUIParameters uiParameters = new ValkyrieUIParameters(robotModel.getRobotVersion(),
			                                                                robotModel.getRobotPhysicalProperties(),
			                                                                robotModel.getJointMap());
			   
			   DRCOperatorInterface.startUserInterface(robotModel, uiParameters);
		   }
	   }
	   catch(JSAPException e)
	   {
		   e.printStackTrace();
	   }
   }
}
