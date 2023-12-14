import bpy
import random
import bmesh
import json

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

def main():

    bpy.ops.object.mode_set(mode='OBJECT')
    focus_target = bpy.data.objects.get('Plane')
    cam = bpy.data.objects.get('Camera.001')

    image_width = bpy.context.scene.render.resolution_x * bpy.context.scene.render.resolution_percentage / 100
    image_height = bpy.context.scene.render.resolution_y * bpy.context.scene.render.resolution_percentage / 100

    image_vertex_mapping = {}

    # Set the world background to white
    bpy.context.scene.world.use_nodes = False
    bpy.context.scene.world.color = (1, 1, 1)

    # Set rendering settings
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    output_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/45/'

    # Store the initial camera location
    initial_location = bpy.context.scene.camera.location.copy()

    # Define parameters for camera movement
    step_size = 1.0
    num_steps = 5

    # Iterate over X, Y, and Z directions
    for direction in ['X', 'Y', 'Z']:
        # Reset camera location to the initial location
        bpy.context.scene.camera.location = initial_location

        # Move the camera along the positive direction
        for step in range(num_steps):
            # Adjust camera location based on the step size
            exec(f"bpy.context.scene.camera.location.{direction.lower()} += step_size")

            # Set the output file path for each image
            file_path = f"{output_path}rendered_image_{direction}_positive_{step + 1}.png"
            bpy.context.scene.render.filepath = file_path

            # Render the image
            bpy.ops.render.render(write_still=True)

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
                image_vertex_mapping[file_path] = image_data
            else:
                print("Object 'Plane' not found or it's not a mesh.")

        # Reset camera location to the initial location
        bpy.context.scene.camera.location = initial_location



        # Move the camera along the negative direction
        for step in range(num_steps):
            # Adjust camera location based on the step size
            exec(f"bpy.context.scene.camera.location.{direction.lower()} -= step_size")

            # Set the output file path for each image
            file_path = f"{output_path}rendered_image_{direction}_negative_{step + 1}.png"
            bpy.context.scene.render.filepath = file_path

            # Render the image
            bpy.ops.render.render(write_still=True)

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
                image_vertex_mapping[file_path] = image_data
            else:
                print("Object 'Plane' not found or it's not a mesh.")

    # Reset the camera location to the initial location
    bpy.context.scene.camera.location = initial_location

    # Reset the file path to the original value
    bpy.context.scene.render.filepath = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/rendered_image.png'

    # Save mapping data to a JSON file
    json_file_path = '/home/hasithab/Desktop/rbe595_aerial_vehicles/sbachimanchi_p5/P5Sim/dataset_images/annotations/image_vertex_mapping_45.json'
    with open(json_file_path, 'w') as json_file:
        json.dump(image_vertex_mapping, json_file, indent=4)

    print(f"JSON data saved to: {json_file_path}")

if __name__ == "__main__":
    main()