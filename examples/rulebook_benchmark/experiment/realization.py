class Realization():
    def __init__(self, max_steps, ego_index=0):
        self.network = None
        self.objects = None
        self.max_steps = max_steps
        self.ego_index = ego_index

    def get_ego(self):
        try:
            return self.objects[self.ego_index]
        except IndexError:
            raise Exception(f"Error: Ego index {self.ego_index} not found in objects list")

    def get_object(self, object_index):
        try:
            return self.objects[object_index]
        except IndexError:
            raise Exception(f"Error: Object index {object_index} not found in objects list")
        
    def get_object_non_ego(self, object_index):
        try:
            return self.objects_non_ego[object_index]
        except IndexError:
            raise Exception(f"Error: Object index {object_index} not found in non-ego objects list")

    def get_object_state(self, object_index, step):
        if 0 <= object_index < len(self.objects):
            return self.objects[object_index].get_state(step)
        else:
            raise Exception(f"Error: Object index {object_index} not found in objects list")
        
    def get_object_non_ego_state(self, object_index, step):
        if 0 <= object_index < len(self.objects_non_ego):
            return self.objects_non_ego[object_index].get_state(step)
        else:
            raise Exception(f"Error: Object index {object_index} not found in non-ego objects list")

    def get_ego_state(self, step):
        return self.get_ego().get_state(step)

    def get_world_state(self, step):
        states = []
        for i in range(len(self.objects)):
            states.append(self.get_object_state(i, step))
        return WorldState(states, step, self.ego_index)

    def set_ego_index(self, ego_index):
        if 0 <= ego_index < len(self.objects):
            self.ego_index = ego_index
        else:
            raise Exception(f"Error: Proposed ego index {ego_index} out of list bounds")

    def get_ego_index(self):
        return self.ego_index

    @property
    def trajectory(self):
        trajectory = []
        for i in range(self.max_steps):
            states = self.get_world_state(i)
            trajectory.append(states)
        return trajectory

    @property
    def objects_non_ego(self):
        return self.objects[:self.ego_index] + self.objects[self.ego_index + 1:]


class RealizationObject():
    def __init__(self, mesh, dimensions, object_type):
        self.mesh = mesh
        self.dimensions = dimensions
        self.object_type = object_type
        self.trajectory = []

    def get_state(self, step):
        try:
            return self.trajectory[step]
        except IndexError:
            raise Exception(f"Error: Step {step} not found in object trajectory")


class State():
    def __init__(self, position, velocity, orientation, step):
        self.position = position
        self.velocity = velocity
        self.orientation = orientation
        self.step = step

    @property
    def orientation_trimesh(self):
        return self.orientation._trimeshEulerAngles()


class WorldState():
    def __init__(self, states, step, ego_index):
        self.ego_index = ego_index
        self.states = states
        self.step = step

    def get_ego_state(self):
        try:
            return self.states[self.ego_index]
        except IndexError:
            raise Exception(f"Error: Ego index {self.ego_index} not found in states list")

    def get_object_state(self, object_index):
        try:
            return self.states[object_index]
        except IndexError:
            raise Exception(f"Error: Object index {object_index} not found in states list")

    @property
    def states_non_ego(self):
        return self.states[:self.ego_index] + self.states[self.ego_index + 1:]









# TODO: timesteps vs timestamps: multiple agents may be off sync/ does scenic use timestamps?