import bpy

# Create a new point light
light_data = bpy.data.lights.new(name="MyLight", type='POINT')
light_data.energy = 100.0  # Adjust the light intensity as needed
light_data.use_nodes = False  # Disable light nodes

# Create a new object with the light data
light_object = bpy.data.objects.new(name="MyLightObject", object_data=light_data)

# Link the light object to the scene
scene = bpy.context.scene
scene.collection.objects.link(light_object)

# Position the light object
light_object.location = (0, 0, 3)  # Adjust the location as needed

# Set the light object as the active object (optional)
bpy.context.view_layer.objects.active = light_object
light_object.select_set(True)

# Finally, update the scene
scene.update()
