from trimesh.transformations import compose_matrix
from scenic.core.regions import MeshVolumeRegion, EmptyRegion
import shapely
from realization import Realization
"""
keys of realization: 
network: scenic object for road network
max_steps
mesh: meshes for each object in the scene (trimesh)
object_type: type of each object in the scene ('Person', 'Car'...)
trajectory: list of dictionaries, where each dictionary contains the state information

keys of each state in trajectory:
position: list of positions of each object [(x, y, z), (x, y, z)...]
orientation: list of orientations of each object [(yaw, pitch, roll, w), (yaw, pitch, roll, w)...]
orientation_trimesh: " " [(yaw, pitch, roll), (yaw, pitch, roll)...]
velocity: list of velocities of each object [(x, y, z), (x, y, z)...]
step: corresponding timestep
"""

def rule_collision(realization, object_type="Pedestrian"):
    trajectory = realization["trajectory"]
    max_steps = realization["max_steps"]
    object_types = realization["object_type"]
    num_objects = len(object_types)
    ego_mesh = realization["mesh"][0]
    ego_dimension = realization["dimensions"][0]
    max_violation = 0
    
    for i in range(max_steps-1):
        ego_pos = trajectory[i]["position"][0]
        ego_orientation = trajectory[i]["orientation"][0]
        ego_region = MeshVolumeRegion(mesh=ego_mesh, dimensions=ego_dimension, position=ego_pos, rotation=ego_orientation)
        ego_velocity = trajectory[i+1]["velocity"][0]
        ego_velocity_before = trajectory[i]["velocity"][0]
        for j in range(1, num_objects):
            if object_types[j] != object_type:
                continue
            adv_mesh = realization["mesh"][j]
            adv_dimension = realization["dimensions"][j]
            adv_pos = trajectory[i]["position"][j]
            adv_orientation = trajectory[i]["orientation"][j]
            adv_region = MeshVolumeRegion(mesh=adv_mesh, dimensions=adv_dimension, position=adv_pos, rotation=adv_orientation)
            adv_velocity = trajectory[i+1]["velocity"][j]
            adv_velocity_before = trajectory[i]["velocity"][j]
            if ego_region.intersects(adv_region):
                
                v_ego_delta = [ego_velocity[i] - ego_velocity_before[i] for i in range(3)]
                v_adv_delta = [adv_velocity[i] - adv_velocity_before[i] for i in range(3)]
                v_ego_delta_norm = sum([v**2 for v in v_ego_delta])**0.5
                v_adv_delta_norm = sum([v**2 for v in v_adv_delta])**0.5
                max_violation = max(max_violation, v_ego_delta_norm, v_adv_delta_norm)
    return max_violation
    
    
def rule_collision(realization, object_type="Pedestrian"):
    # re-write the function to use the new realization object
    max_steps = realization.max_steps
    ego = realization.get_ego()
    objects = [obj for obj in realization.objects_non_ego if obj.object_type == object_type]
    max_violation = 0
    
    for i in range(max_steps-1):
        ego_state = ego.get_state(i)
        ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=ego_state.position, rotation=ego_state.orientation)
        ego_velocity_before = ego_state.velocity
        ego_velocity_after = ego.get_state(i+1).velocity
        for obj in objects:
            obj_state = obj.get_state(i)
            obj_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=obj_state.position, rotation=obj_state.orientation)
            obj_velocity_before = obj_state.velocity
            obj_velocity_after = obj.get_state(i+1).velocity
            if ego_region.intersects(obj_region):
                ego_delta = ego_velocity_after - ego_velocity_before
                obj_delta = obj_velocity_after - obj_velocity_before
                
                ego_delta_norm = ego_delta.norm()
                obj_delta_norm = obj_delta.norm()
                
                
                max_violation = max(max_violation, ego_delta_norm, obj_delta_norm)
        
    return max_violation
        
def rule_vehicle_collision(realization):
    return max(rule_collision(realization, "Car"), rule_collision(realization, "Truck"))

def rule_vru_collision(realization):
    return max(rule_collision(realization, "Pedestrian"), rule_collision(realization, "Bicycle"))
    
    



def rule_stay_in_drivable_area(realization):
    network = realization["network"]
    drivable_region = network.drivableRegion
    trajectory = realization["trajectory"]
    max_steps = realization["max_steps"]
    ego_mesh = realization["mesh"][0]
    ego_dimension = realization["dimensions"][0]
    max_violation = 0
    
    for i in range(max_steps):
        ego_pos = trajectory[i]["position"][0]
        ego_orientation = trajectory[i]["orientation"][0]
        ego_region = MeshVolumeRegion(mesh=ego_mesh, dimensions=ego_dimension, position=ego_pos, rotation=ego_orientation)
        ego_polygon = ego_region.boundingPolygon.polygons
        drivable_polygon = drivable_region.polygons
        distance = shapely.hausdorff_distance(ego_polygon, drivable_polygon)
        max_violation = max(max_violation, distance)

    return max_violation



def rule_stay_in_drivable_area(realization):
    network = realization.network
    drivable_region = network.drivableRegion
    ego = realization.get_ego()
    max_violation = 0
    
    for state in ego.trajectory:
        ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=state.position, rotation=state.orientation)
        ego_polygon = ego_region.boundingPolygon.polygons
        drivable_polygon = drivable_region.polygons
        distance = shapely.hausdorff_distance(ego_polygon, drivable_polygon)
        max_violation = max(max_violation, distance)
        




def vru_clearance(realization, on_road=False):
    threshold = 2
    network = realization["network"]
    trajectory = realization["trajectory"]
    max_steps = realization["max_steps"]
    object_types = realization["object_type"]
    num_objects = len(object_types)
    ego_mesh = realization["mesh"][0]
    ego_dimension = realization["dimensions"][0]
    drivable_region = network.drivableRegion
    max_violation = 0
    
    for i in range(max_steps):
        ego_pos = trajectory[i]["position"][0]
        ego_orientation = trajectory[i]["orientation"][0]
        ego_region = MeshVolumeRegion(mesh=ego_mesh, dimensions=ego_dimension, position=ego_pos, rotation=ego_orientation)
        for j in range(1, num_objects):
            if object_types[j] != "Pedestrian" and object_types[j] != "Bicycle":
                continue
            
            
            adv_mesh = realization["mesh"][j]
            adv_dimension = realization["dimensions"][j]
            adv_pos = trajectory[i]["position"][j]
            adv_orientation = trajectory[i]["orientation"][j]
            adv_region = MeshVolumeRegion(mesh=adv_mesh, dimensions=adv_dimension, position=adv_pos, rotation=adv_orientation)
                        
            ego_polygon = ego_region.boundingPolygon.polygons
            adv_polygon = adv_region.boundingPolygon.polygons
            distance = ego_polygon.distance(adv_polygon)
            violation = threshold - distance
            
            if (on_road and drivable_region.intersects(adv_region)) or (not on_road and not drivable_region.intersects(adv_region)):
                max_violation = max(max_violation, violation)
            
    return max_violation


def vru_clearance(realization, on_road=False, threshold = 2):
    ego = realization.get_ego()
    objects = [obj for obj in realization.objects_non_ego if obj.object_type in ["Pedestrian", "Bicycle"]]
    drivable_region = realization.network.drivableRegion
    max_violation = 0
    
    for state in ego.trajectory:
        ego_region = MeshVolumeRegion(mesh=ego.mesh, dimensions=ego.dimensions, position=state.position, rotation=state.orientation)
        for obj in objects:
            obj_state = obj.get_state(state.step)
            obj_region = MeshVolumeRegion(mesh=obj.mesh, dimensions=obj.dimensions, position=obj_state.position, rotation=obj_state.orientation)
            ego_polygon = ego_region.boundingPolygon.polygons
            obj_polygon = obj_region.boundingPolygon.polygons
            distance = ego_polygon.distance(obj_polygon)
            violation = threshold - distance
            if (on_road and drivable_region.intersects(obj_region)) or (not on_road and not drivable_region.intersects(obj_region)):
                max_violation = max(max_violation, violation)
    return max_violation
    



def vru_clearance_on_road(realization):
    return vru_clearance(realization, on_road=True)


def vru_clearance_off_road(realization):
    return vru_clearance(realization, on_road=False)


def vru_acknowledgement(realization, proximity=5, deceleration=0.2,  timesteps=10):

    trajectory = realization["trajectory"]
    max_steps = realization["max_steps"]
    object_types = realization["object_type"]
    num_objects = len(object_types)
    ego_mesh = realization["mesh"][0]
    ego_dimension = realization["dimensions"][0]
    max_violation = 0
    
    for i in range(max_steps - timesteps):
        ego_pos = trajectory[i]['position'][0]
        ego_velocity = trajectory[i]['velocity'][0]
        ego_next_velocity = trajectory[i+1]['velocity'][0]
        
        ego_future_pos = trajectory[i+timesteps]['position'][0]
        adv_future_pos = trajectory[i+timesteps]['position'][1]
        
        distance = sum([(ego_future_pos[j] - adv_future_pos[j])**2 for j in range(3)])**0.5
        if distance < proximity:
            ego_velocity_norm = sum([v**2 for v in ego_velocity])**0.5
            ego_next_velocity_norm = sum([v**2 for v in ego_next_velocity])**0.5
            violation = deceleration - (ego_velocity_norm - ego_next_velocity_norm)
            max_violation = max(max_violation, violation)
            
    return max_violation
                
                
                
def vru_acknowledgement(realization, proximity=5, deceleration=0.2,  timesteps=10):
    ego = realization.get_ego()
    objects = realization.objects_non_ego
    max_violation = 0
    
    for i in range(realization.max_steps - timesteps):
        ego_state = ego.get_state(i)
        ego_future_state = ego.get_state(i+timesteps)
        ego_velocity = ego_state.velocity
        ego_next_velocity = ego_future_state.velocity
        ego_future_pos = ego_future_state.position
        for obj in objects:
            adv_current_pos = obj.get_state(i).position
            distance = (ego_future_pos - adv_current_pos).norm()
            if distance < proximity:
                ego_velocity_norm = ego_velocity.norm()
                ego_next_velocity_norm = ego_next_velocity.norm()
                violation = deceleration - (ego_velocity_norm - ego_next_velocity_norm)
                max_violation = max(max_violation, violation)
    return max_violation

        
                

        
        
    
    