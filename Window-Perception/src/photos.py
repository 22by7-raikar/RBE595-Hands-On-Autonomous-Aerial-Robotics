    #import bpy
    #import math

    #def position_camera(cam, angle_x, angle_y, radius, focus_target):
    #    # Position the camera
    #    cam.location = (0, -radius, 0)
    #    
    #    # Clear existing track_to constraints from the camera
    #    for constraint in cam.constraints:
    #        if constraint.type == 'TRACK_TO':
    #            cam.constraints.remove(constraint)

    #    # Add track_to constraint to point the camera at the focus_target
    #    track_to = cam.constraints.new(type='TRACK_TO')
    #    track_to.target = focus_target
    #    track_to.track_axis = 'TRACK_NEGATIVE_Z'
    #    track_to.up_axis = 'UP_Y'
    #    
    #    # Rotate the camera around the Z-axis
    #    cam.rotation_euler[2] = math.radians(angle_x)
    #    
    #    # Rotate the camera up or down around its local X-axis
    #    cam.rotation_euler[0] = math.radians(angle_y)

    ## Ensure you're in object mode
    #bpy.ops.object.mode_set(mode='OBJECT')

    ## Get the focus target and camera
    #focus_target = bpy.data.objects['Plane.001']
    #cam = bpy.data.objects['NN_cam']

    ## Parameters for camera positioning
    #angles_x = [0, 90, 180, 270]  # Around the object in the XZ plane
    #angles_y = [0, 30, 60, -30, -60]  # Above and below the object
    #radius = 10  # Distance from the focus_target

    ## Position camera at different angles and render images
    #render_path = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/Dataset/Train/Images/"  # Modify this path as needed
    #for ax in angles_x:
    #    for ay in angles_y:
    #        position_camera(cam, ax, ay, radius, focus_target)
    #        bpy.context.scene.camera = cam
    #        bpy.context.scene.render.filepath = f"{render_path}render_{ax}_{ay}.png"
    #        bpy.ops.render.render(write_still=True)


    #import bpy
    #import math

    #def position_camera(cam, angle_x, angle_y, x,y, focus_target):
    #    # Position the camera
    #    cam.location = (x, y, 0)
    #    
    #    # Clear existing track_to constraints from the camera
    #    for constraint in cam.constraints:
    #        if constraint.type == 'TRACK_TO':
    #            cam.constraints.remove(constraint)

    #    # Add track_to constraint to point the camera at the focus_target
    #    track_to = cam.constraints.new(type='TRACK_TO')
    #    track_to.target = focus_target
    #    track_to.track_axis = 'TRACK_NEGATIVE_Z'
    #    track_to.up_axis = 'UP_Y'
    #    
    #    # Rotate the camera around the Z-axis
    #    cam.rotation_euler[2] = math.radians(angle_x)
    #    
    #    # Rotate the camera up or down around its local X-axis
    #    cam.rotation_euler[0] = math.radians(angle_y)

    ## Ensure you're in object mode
    #bpy.ops.object.mode_set(mode='OBJECT')

    ## Get the focus target and camera
    #focus_target = bpy.data.objects['Plane.001']
    #cam = bpy.data.objects['NN_cam.001'] #Change to NN_cam for other camera photo generation

    ## Parameters for camera positioning
    #angles_x = [0, 90, 180, 270]  # Around the object in the XZ plane
    #angles_y = [0, 30, 60, -30, -60]  # Above and below the object
    #radiusy = [0,0.5,1,1.5]  # Distance from the focus_target
    #radiusx = [0,0.5,1,1.5]
    ## Position camera at different angles and render images
    #render_path = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/Dataset/Train/Images/"  # Modify this path as needed
    #for y in radiusy:
    #    for x in radiusx:
    #        position_camera(cam, 0, 0, x,y, focus_target)
    #        bpy.context.scene.camera = cam
    #        bpy.context.scene.render.filepath = f"{render_path}render{x}_{y}.png"
    #        bpy.ops.render.render(write_still=True)
import bpy
import bmesh
import math
from mathutils import Matrix
import json

def position_camera(cam, angle_x, angle_y, x, y, z, focus_target):
    cam.location = (x, y, z)
    
    for constraint in cam.constraints:
        if constraint.type == 'TRACK_TO':
            cam.constraints.remove(constraint)

    track_to = cam.constraints.new(type='TRACK_TO')
    track_to.target = focus_target
    track_to.track_axis = 'TRACK_NEGATIVE_Z'
    track_to.up_axis = 'UP_Y'
    
    cam.rotation_euler[2] = math.radians(angle_x)
    cam.rotation_euler[0] = math.radians(angle_y)

def world_to_camera_view(scene, camera, coord):
    co_local = camera.matrix_world.normalized().inverted() @ coord
    z = -co_local.z

    camera_data = camera.data
    frame = [-v for v in camera_data.view_frame(scene=scene)[:3]]
    if camera_data.type == 'ORTHO':
        is_ortho = True
        scale = camera_data.ortho_scale
    else:
        is_ortho = False
        sensor_width = camera_data.sensor_width
        sensor_height = camera_data.sensor_height
        sensor_fit = camera_data.sensor_fit
        if (sensor_fit == 'VERTICAL' or
            (sensor_fit == 'AUTO' and sensor_width < sensor_height)):
            scale = sensor_width / sensor_width
        else:
            scale = sensor_height / sensor_width

    if not is_ortho:
        if co_local.z == 0.0:
            return None, None
        else:
            frame = [(v / (v.z / z)) for v in frame]

    min_x, max_x = frame[1].x, frame[2].x
    min_y, max_y = frame[0].y, frame[1].y

    x = (co_local.x - min_x) / (max_x - min_x)
    y = (co_local.y - min_y) / (max_y - min_y)

    return x, y

def adjust_lighting(light_name, intensity):
    light = bpy.data.objects.get(light_name)
    if light and light.type == 'LIGHT':
        light.data.energy = intensity
        print(f"Adjusted {light_name} intensity to {intensity}")
    else:
        print(f"Light source {light_name} not found or is not a lamp object.")


def main():
    bpy.ops.object.mode_set(mode='OBJECT')
    focus_target = bpy.data.objects.get('Plane')
    cam = bpy.data.objects.get('NN_cam.001')

    angles_x = [0, 90, 180, 270]
    angles_y = [0, 30, 60, -30, -60]
    radiusz = [0, 0.5, 1, 1.5, 2.0]
    radiusy = [0, 0.5, 1, 1.5, 2.0]
    radiusx = [0, 0.5, 1, 1.5, 2.0]

    # Adjust the lighting condition by modifying this value
    # For Eevee or Cycles, you might control lighting through environment or lamp strengths

    light_name = "MyLightSource"
    intensities = [0.0,10,20,30,100]

    render_path = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/raw_images/train/1"
    json_file_path = "/home/anuj/Desktop/AerialRobotics/apairaikar_p3a/datasets/mydataset/annotations/image_vertex_mapping_1.json"
    image_width = bpy.context.scene.render.resolution_x * bpy.context.scene.render.resolution_percentage / 100
    image_height = bpy.context.scene.render.resolution_y * bpy.context.scene.render.resolution_percentage / 100

    image_vertex_mapping = {}
    for az in radiusz:
        for ay in radiusy:
            for ax in radiusx:
                for intensity in intensities:
                    position_camera(cam, 0, 0, ax, ay, az, focus_target)
                    bpy.context.scene.camera = cam

                    # Adjust the lighting condition
                    adjust_lighting(light_name, intensity)

                    bpy.context.scene.render.filepath = f"multcond1{render_path}render{ax}_{ay}_{az}_{intensity}_1.png"
                    bpy.ops.render.render(write_still=True)

                    image_file_path = f"{render_path}render{ax}_{ay}_{az}_1.png"
                    if focus_target and focus_target.type == 'MESH':
                        bm = bmesh.new()
                        bm.from_mesh(focus_target.data)
                        bm.transform(focus_target.matrix_world)

                        image_data = []
                        seen_pixel_coordinates = set()  # Store the pixel coordinates that we've already added for this image

                        for v in bm.verts:
                            x, y = world_to_camera_view(bpy.context.scene, cam, v.co)

                            if x is not None and y is not None:
                                pixel_x = x * image_width
                                pixel_y = image_height - y * image_height

                                pixel_coordinates = (round(pixel_x, 2), round(pixel_y, 2))

                                if pixel_coordinates not in seen_pixel_coordinates:
                                    image_data.append({
                                        'vertex_index': v.index,
                                        'image_coordinates': pixel_coordinates
                                    })
                                    seen_pixel_coordinates.add(pixel_coordinates)

                        bm.free()
                        image_vertex_mapping[image_file_path] = image_data
                    else:
                        print("Object 'Plane' not found or it's not a mesh.")
                
    with open(json_file_path, 'w') as json_file:
        json.dump(image_vertex_mapping, json_file, indent=4)

    print(f"JSON data saved to: {json_file_path}")

if __name__ == "__main__":
    main()
