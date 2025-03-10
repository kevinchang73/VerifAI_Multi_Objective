param map = localPath('../../multi_objective/maps/Town01.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model


behavior simple():
    while True:
        take SetThrottleAction(1)
        


behavior do_nothing():
    while True:
        wait

 
ego = new Car with behavior AutopilotBehavior()
adv = new Car with behavior FollowLaneBehavior()


import bench