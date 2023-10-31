import bpy

def add_point_light(name="My Point Light", location=(0, 0, 0)):
    """Add a point light to the scene at the specified location and name it."""
    
    # Ensure we're in Object mode
    bpy.ops.object.mode_set(mode='OBJECT')
    
    # Add the light source
    bpy.ops.object.light_add(type='POINT', location=location)
    
    # Name the light source
    bpy.context.active_object.name = name
    
    # Optional: You can set additional properties of the light here if desired
    # For example: bpy.context.active_object.data.energy = 1000

# Call the function to add and name the light
add_point_light(name="My Custom Light", location=(0, 0, 5))
