from pxr import Usd, UsdGeom, UsdLux, UsdShade, Sdf, Gf
import omni.ext
import omni.usd
import omni.kit.app
import paho.mqtt.client as mqtt
import queue


"""##################################################
################ Globals ################"""
update_queue = queue.Queue()    # Synchronization queue
update_omniverse_subscription = None
client = None
stage = None
"""##################################################"""


"""##################################################
################ Communication Layer ################"""
def on_connack(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("newmotion_nti")
    client.subscribe("fire_nti")
    client.subscribe("newtouch_nti")
    client.subscribe("distance_nti1")
    client.subscribe("distance_nti2")
    client.subscribe("distance_nti3")

def on_message(client, userdata, msg):
    topic = msg.topic
    payload= msg.payload.decode()
    update_queue.put((topic, payload))
    print("Received message on topic {}: {}".format(topic, payload))
"""##################################################"""


"""##################################################
################ Synchronization Layer ################"""
def update_scene():
    topic, payload = update_queue.get()
    try:
        if topic == "newmotion_nti":
            if int(payload):
                update_prim_local_orientation(stage, "/world/door", 0, 0, 90)
                print("Door Opened")
            else:
                update_prim_local_orientation(stage, "/world/door", 0, 0, 0)
                print("Door Closed")

        elif topic == "newtouch_nti":
            if int(payload):
                for i in range(1,7):
                    lamp_block(stage, f"environment/spot_{i}", 600000)
                    set_cube_color(stage, "/world/touch_sensor", (0,1,0))
                    print("Light On")

            else:
                for i in range(1,7):
                    lamp_block(stage, f"environment/spot_{i}", 0)
                    set_cube_color(stage, "/world/touch_sensor", (0,0,0))
                    print("Light OFF")

        elif topic == "fire_nti":
            if int(payload):
                set_cube_color(stage, "/world/fire_sensor", (1,0,0))
                print("Fire")

            else:
                set_cube_color(stage, "/world/fire_sensor", (0,0,0))
               
        elif topic == "distance_nti1":
            if float(payload):
                update_prim_world_translation(stage, "/world/chair_1", float(payload), axis=1)
            else:
                update_prim_world_translation(stage, "/world/chair_1", 0, axis=1)

        elif topic == "distance_nti2":
            if float(payload):
                update_prim_world_translation(stage, "/world/chair_2", float(payload), axis=1)
            else:
                update_prim_world_translation(stage, "/world/chair_2", 0, axis=1)
        
        elif topic == "distance_nti3":
            if float(payload):
                update_prim_world_translation(stage, "/world/chair_3", float(payload), axis=1)
            else:
                update_prim_world_translation(stage, "/world/chair_3", 0, axis=1)
        else:
            print(f"Unknown topic: {topic}") 
    
    except ValueError:
        print(f"Invalid payload format for topic '{topic}': {payload}")
"""##################################################"""


"""##################################################
################ Visualization Layer ################"""
#OpenUSD-Based Edit APIs
def lamp_block(stage, light_path, light_intensity):
    light_prim = stage.GetPrimAtPath(light_path)
    light = UsdLux.SphereLight(light_prim)
    intensity = light_intensity
    light.GetIntensityAttr().Set(intensity)
    print(f"💡 Lamp intensity set to {intensity}")
    #stage.GetRootLayer().Save()
    
def set_cube_color(stage, object_path, color):
    prim = stage.GetPrimAtPath(object_path)
    material = UsdShade.Material.Define(stage, object_path + "/RedMaterial")
    shader = UsdShade.Shader.Define(stage, object_path + "/RedMaterial/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(prim).Bind(material)
    #stage.Save()
    #stage.GetRootLayer().Save()

"""==================================================
    @brief  Sets a prim’s local rotation to specified Euler angles, preserving its world-space position.
    
    This function applies the given XYZ Euler angles (in degrees) to the prim’s local rotation, retaining
    its current world-space position. The transform stack is updated with the new rotation and preserved
    position, and the stage is saved to persist changes.
    
    @param  stage      The USD stage containing the prim.
    @param  prim_path  The path to the prim (e.g., "/Scope1/cube").
    @param  x          X-axis rotation angle (degrees).
    @param  y          Y-axis rotation angle (degrees).
    @param  z          Z-axis rotation angle (degrees).
    
    @retval None
    
    @note   - Requires a valid, Xformable prim.
            - Applies rotation in XYZ order, local space.
            - Preserves world-space position with translation.
            - Uses translation, then rotation, for correct placement.
            - Operates at default time code (non-animated).
            - Recommended for static reorientation.
    
    @example
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateNew("scene.usda")
    cube = UsdGeom.Cube.Define(stage, "/cube")
    update_prim_local_orientation(stage, "/cube", 45.0, 0.0, 0.0)
    # Sets rotation to [45, 0, 0], preserves position
    =================================================="""
def update_prim_local_orientation(stage, prim_path, x, y, z):
    # 1. Get the prim at the specified path
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"Error: Prim at {prim_path} is invalid.")
        return

    # 2. Convert the prim to Xformable for transform operations
    xform = UsdGeom.Xformable(prim)
    if not xform:
        print(f"Error: Prim at {prim_path} is not Xformable.")
        return

    # 3. Get the current world transform
    time = Usd.TimeCode.Default()

    # 3.1 Get the current world postion
    world_transform = xform.ComputeLocalToWorldTransform(time)
    world_position = world_transform.ExtractTranslation()

    # 4. Create a rotation vector from the provided Euler angles
    new_rotation = Gf.Vec3f(x, y, z)

    # 5. Clear existing transform ops to avoid stacking multiple translates
    xform.ClearXformOpOrder()

    # 6. Add Add a new transform op
    # 6.1 Add the new XYZ rotation op
    rotate_op = xform.AddRotateXYZOp()
    rotate_op.Set(new_rotation)

    # 6.2 Add the old translation op
    translate_op = xform.AddTranslateOp()
    translate_op.Set(world_position)

    # 7. Adjust the op order, so the the new op is the last one
    # xform.SetXformOpOrder([translate_op, rotate_op])  

    # 8. Save the stage to persist the changes
    #stage.Save()

    print(f"[Update] Prim rotated to: {new_rotation} at position: {world_position}")

"""==================================================
    @brief  Updates a prim’s world-space translation while preserving its rotation.
    
    This function shifts the prim’s world-space position by the specified offset distance along the chosen
    axis (X, Y, or Z), retaining its existing rotation as XYZ Euler angles. The transform stack is updated
    with a new translation and the preserved rotation, and the stage is saved to persist changes.
    
    @param  stage          The USD stage containing the prim to update.
    @param  prim_path      The path to the prim (e.g., "/Scope1/cube") as a string or Sdf.Path.
    @param  offset_distance The distance to offset the prim’s position along the specified axis.
    @param  axis           The axis to translate along (1 for X, 2 for Y, 3 for Z).
    
    @retval None
    
    @note   - Ensure the stage is valid and the prim is Xformable.
            - Updates position in world space, accounting for parent transforms.
            - Preserves existing rotation (XYZ order) if present.
            - Applies translation, then rotation, for consistent positioning.
            - Operates at the default time code (non-animated).
            - Invalid axis values (not 1, 2, or 3) trigger an error and no changes.
    
    @example
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateNew("scene.usda")
    cube = UsdGeom.Cube.Define(stage, "/cube")
    update_prim_world_translation(stage, "/cube", 5.5, 1)
    # Updates the cube's translation to [5.5, 0.0, 0.0] in world space while preserving the rotation.
    update_prim_world_translation(stage, "/cube", 10.6, 2)
    # Updates the cube's translation to [5.5, 10.6, 0.0] in world space while preserving the rotation.
    update_prim_world_translation(stage, "/cube", 15.0, 2)
    # Updates the cube's translation to [5.5, 25.6, 0] in world space while preserving the rotation.
    =================================================="""
def update_prim_world_translation(stage, prim_path, offset_distance, axis=1):
    # 1. Get the prim
    offset_distance = offset_distance / 100 # Convert Sonic Readings To Meters
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"Error: Prim at {prim_path} is invalid.")
        return

    # 2. Convert the prim to Xformable for transform operations
    xform = UsdGeom.Xformable(prim)
    if not xform:
        print(f"Error: Prim at {prim_path} is not Xformable.")
        return

    # 3. Get the current world transform
    ## Get the original world transform <<<============================================
    time = Usd.TimeCode.Default()
    time_stack = []
    if len(time_stack) == 0:
        time_stack.append(time)
    else:
        pass

    # 3.1 Get the current world postioncktime_stack
    world_transform = xform.ComputeLocalToWorldTransform(time_stack[0])
    world_position = world_transform.ExtractTranslation()

    # 3.2 Get the current world rotation
    world_rotation = Gf.Vec3f(0, 0, 0)  # Default to no rotation
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            world_rotation = op.Get()

    # 4. Calculate new translation based on the user-defined axis
    if axis == 1:
        new_translation = world_position + Gf.Vec3d(offset_distance, 0.0, 0.0)
    elif axis == 2:
        new_translation = world_position + Gf.Vec3d(0.0, offset_distance, 0.0)
    elif axis == 3:
        new_translation = world_position + Gf.Vec3d(0.0, 0.0, offset_distance)
    else:
        print("Error: Invalid axis. Use 1 for X, 2 for Y, or 3 for Z.")
        return

    # 5. Clear existing transform ops to avoid stacking multiple translates
    xform.ClearXformOpOrder()

    # 6. Add a new transform op
    

    # 6.1 Add the new translation op 
    translate_op = xform.AddTranslateOp()
    translate_op.Set(new_translation)

    # 6.2 Add the old rotation op 
    rotation_op = xform.AddRotateXYZOp()
    rotation_op.Set(world_rotation)

    # 7. Adjust the op order, so the the new op is the last one
    # xform.SetXformOpOrder([rotation_op, translate_op])  

    # 8. Save the stage to persist the changes
    #stage.Save()

    print(f"[Update] Prim translated to: {new_translation} with rotation: {world_rotation}")
"""##################################################"""


"""##################################################
################ Main Layer ################"""
def mqtt_configuration():
    # Create an MQTT client instance
    client = mqtt.Client()
    client.on_connect = on_connack
    client.on_message = on_message
    # Connect to the MQTT broker
    client.connect("10.204.25.207", 1883, 60)
    return client

def omniverse_configuration(stage):

    """Subscribe to OmniKit's update stream."""
    # Set up a subscription to the IsaacSim update event stream.
    # This ensures 'on_update' runs every frame on the main thread.
    app = omni.kit.app.get_app()
    update_stream = app.get_update_event_stream()
    update_omniverse_subscription = update_stream.create_subscription_to_pop(update_scene, order=0)
    return update_omniverse_subscription

def main():
    global client, update_omniverse_subscription, stage

    # 1. Omniverse Configuration
    stage = omni.usd.get_context().get_stage()
    update_omniverse_subscription = omniverse_configuration(stage)
    print("[Script] Subscribed to the update event stream for main-thread updates.")

    # 2. Configure and start the MQTT client in the background.
    client= mqtt_configuration()
    client.loop_start()
    print("MQTT client started")
"""##################################################"""

main()  #Program Startting Point