

tc = rtt.getTC()
depl = tc:getPeer("Deployer")
depl:import("ocl")
depl:import("UniversalRobotsUR5")

depl:loadComponent("ur5", "UniversalRobotsUR5")
ur5 = depl:getPeer("ur5")
ur5:configure()
--ur:start()


