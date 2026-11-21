import bpy

def setup_render():
    scene = bpy.context.scene
    
    # Set render engine to Eevee for speed
    scene.render.engine = 'CYCLES'
    
    # Set resolution
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    
    # Set output path
    scene.render.filepath = "//output/render"
    scene.render.image_settings.file_format = 'PNG'
    
    # Set up camera
    camera = bpy.data.objects.get("Camera")
    if camera:
        camera.location = (5.0, -8.0, 5.0)
        camera.rotation_euler = (0.8, 0, 0.5)
    
    # Add area light
    if 'AreaLight' not in bpy.data.lights:
        light = bpy.data.lights.new(name="AreaLight", type='AREA')
        light.energy = 100
        light.size = 5
        light_obj = bpy.data.objects.new(name="AreaLight", object_data=light)
        light_obj.location = (0, -5, 10)
        bpy.context.collection.objects.link(light_obj)
    
    # Add ambient light
    if 'Ambient' not in bpy.data.lights:
        world = bpy.data.worlds.get("World")
        if world:
            world.light_settings.ambient_color = (0.1, 0.1, 0.1)
    
    print("Render setup complete.")
    return scene

if __name__ == "__main__":
    setup_render()
