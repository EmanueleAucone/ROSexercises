ESERCIZIO 2

-Comandi da lanciare:

roslaunch exercise2 test.launch SATmin_longVel:=valore SATmax_longVel:=valore SATmin_yawRate:=valore SATmax_yawRate:=valore

rqt_plot 

rostopic pub -r valore /controller/high_level_command exercise2/ControllerCmand "longitudinal_velocity: valore
yaw_angular_velocity: valore"

-Note:

se su rqt_plot non compare in automatico, aggiungere: 
	/controller/high_level_command/longitudinal_velocity
	/controller/high_level_command/yaw_angular_velocity
	/turtle1/cmd_vel/linear/x
	/turtle1/cmd_vel/angular/z
che sono rispettivamente i comandi di alto livello di velocità longitudinale e yaw rate inviati, e quelli effettivamente mandati al robot, dunque soggetti a saturazione se i valori eccedono i bound.


