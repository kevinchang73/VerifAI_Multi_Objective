from scenic.domains.driving.roads import Network
model scenic.simulators.carla.model
from realization import Realization, RealizationObject, ObjectState

behavior bench():

    realization = globalParameters['realization']
    realization.network = Network.fromFile(globalParameters['map'])

    objects = simulation().objects[:-1]
    max_steps = realization.max_steps
    objs = []
    for obj in objects:
        objs.append(RealizationObject(obj.shape.mesh.copy(), obj.occupiedSpace.dimensions, type(obj).__name__))
    realization.objects = objs

    step = 0
    while step < max_steps:
        objects = simulation().objects[:-1]

        for i in range(len(objects)):
            obj = realization.objects[i]
            obj.trajectory.append(ObjectState(obj.position, obj.velocity, obj.orientation, step))

        step += 1
        #if step == max_steps:
        #    break
        
        wait


BENCH_OBJ = new Bench with behavior bench()