
tc = rtt.getTC()
depl = tc:getPeer("Deployer")
depl:import("ocl")

depl:import("UniversalRobotsUR5")
depl:loadComponent("ur5", "UniversalRobotsUR5")
ur5 = depl:getPeer("ur5")

depl:import("URRealTime")
depl:loadComponent("urrtt", "URRealTime")
urrtt = depl:getPeer("urrtt")

ur5:configure()

urrtt:configure()
time_period = 1.0/60.0
ur5:setPeriod(time_period)


urrtt_q_inputport = urrtt:getPort("URRTTDesiredJointPosition")
print(urrtt_q_inputport)
urrtt_q_outputport = urrtt:getPort("URRTTActualJointPosition")
urrtt_qdot_outputport = urrtt:getPort("URRTTActualJointVelocity")


ur5_q_inputport = ur5:getPort("UR5JointPosition")
ur5_qdot_inputport = ur5:getPort("UR5JointVelocity")
ur5_q_outputport = ur5:getPort("UR5DesiredJointPosition")
print(urrtt_q_inputport)


urrtt_q_outputport:connect(ur5_q_inputport)
ur5_q_outputport:connect(urrtt_q_inputport)

urrtt:start()
ur5:start()
