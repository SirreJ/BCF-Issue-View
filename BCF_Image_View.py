bl_info = {
    "name": "BCF Issue View",
    "author": "Joel Simon Manolias",
    "version": (0, 1),
    "blender": (4, 1, 0),
    "location": "Properties > Scene > BCF Issue View",
    "description": "Adds a camera of an BCF Issue and can create a projection face to display this Issue. Also adds an annotation layer",
    "warning": "",
    "doc_url": "",
    "category": "Add Camera and Face",
}

import bpy
import mathutils
import bmesh
import xml.etree.ElementTree as ET
import os
import math
from mathutils import Matrix, Vector
from datetime import datetime

distance_between_layers = 0.01
number_cuts = 50

# create camera with bcf data via file path

def main_create_camera_with_BCF_data(context):
    
    # find absolute path
    absolute_path = bpy.path.abspath(bpy.context.scene.import_filepath)
    # path to XML_File
    absolute_snapshot_path = absolute_path

    base_path = os.path.dirname(absolute_snapshot_path)

    # get relative path to viewpoint and snapshot 
    viewpoint_path = "viewpoint.bcfv"
            
    # create absolute path to the files
    absolute_viewpoint_path = os.path.join(base_path, viewpoint_path)

    # define new tree and root
    tree = ET.parse(absolute_viewpoint_path)
    root = tree.getroot()

    # extract data
    location = root.find('.//PerspectiveCamera/CameraViewPoint')
    direction = root.find('.//PerspectiveCamera/CameraDirection')
    up_vector = root.find('.//PerspectiveCamera/CameraUpVector')
    field_of_view_bcf = root.find('.//PerspectiveCamera')

    camera_location = (
        float(location.find('X').text),
        float(location.find('Y').text),
        float(location.find('Z').text)
    )

    camera_direction = (
        float(direction.find('X').text),
        float(direction.find('Y').text),
        float(direction.find('Z').text)
    )

    camera_up_vector = (
        float(up_vector.find('X').text),
        float(up_vector.find('Y').text),
        float(up_vector.find('Z').text)
    )
    
    field_of_view = float(field_of_view_bcf.find('FieldOfView').text)
    
    # create camera in blender
    camera_data = bpy.data.cameras.new(name='Camera')
    camera_object = bpy.data.objects.new('Camera', camera_data)
    bpy.context.scene.collection.objects.link(camera_object)
    
    camera_object.location = camera_location

    # calculate rotation matrix
    forward = Vector(camera_direction).normalized()
    up = Vector(camera_up_vector).normalized()
    right = forward.cross(up).normalized()
    up_corrected = right.cross(forward).normalized()
    
    rotation_matrix = Matrix((
        right,
        up_corrected,
        -forward
    )).transposed().to_4x4()
    
    camera_object.matrix_world = rotation_matrix
    camera_object.location = camera_location

    camera_data.angle = math.radians(field_of_view)
    
    # camera name
    camera_object.name = "Camera Issue"
    bpy.context.view_layer.objects.active = camera_object
    camera = camera_object
    
    # set resolution
    img = bpy.data.images.load(absolute_snapshot_path)
    width = img.size[0]
    height = img.size[1]
    bpy.context.scene.render.resolution_x = width
    bpy.context.scene.render.resolution_y = height
    bpy.context.scene.render.pixel_aspect_x = 1.0
    bpy.context.scene.render.pixel_aspect_y = 1.0
        
    camera_name = "Camera Issue"
    bpy.context.scene.camera = camera
    
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.region_3d.view_perspective = 'CAMERA'
    
    # is this the right way to adjust the Perspective of the picture?
    if context.scene.focal_length.value !=0:
        camera.data.lens = context.scene.focal_length.value
    if context.scene.sensor_width.value !=0:
        camera.data.sensor_width = context.scene.sensor_width.value
    
    return camera, camera_name, absolute_snapshot_path
                                 
# get the closeset objekt in fromt of the camera

def get_first_object_in_view(camera):
    
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()

    # cameramatrix and origin
    cam_matrix = camera.matrix_world.normalized()
    cam_origin = cam_matrix.to_translation()
    cam_forward = cam_matrix.to_3x3() @ mathutils.Vector((0.0, 0.0, -1.0))

    # set origin and ray direction
    ray_origin = cam_origin
    ray_direction = cam_forward

    closest_obj = None
    closest_distance = float('inf')
        
    # get the closest object
    for obj in scene.objects:
        if obj.type == 'MESH' and obj.visible_get():
            obj_eval = obj.evaluated_get(depsgraph)
            # check if the object can be evaluated
            if obj_eval:
                matrix = obj.matrix_world.inverted()
                ray_origin_local = matrix @ ray_origin
                ray_direction_local = matrix.to_3x3() @ ray_direction

                success, location, normal, face_index = obj_eval.ray_cast(ray_origin_local, ray_direction_local)
                if success:
                    distance = (location - ray_origin_local).length
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_obj = obj
            else:
                print("Warning: The object '{}' can not be evaluated.".format(obj.name))

    return closest_obj

# get the main face of the closest object to create a face for projection

def get_largest_visible_face(camera, obj):    
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()

    # get camera information
    cam_matrix = camera.matrix_world.normalized()
    cam_origin = cam_matrix.to_translation()
    cam_forward = cam_matrix.to_3x3() @ mathutils.Vector((0.0, 0.0, -1.0))
        
    # get object information relative to camera
    matrix = obj.matrix_world.inverted()
    ray_origin_local = matrix @ cam_origin
    ray_direction_local = matrix.to_3x3() @ cam_forward

    obj_eval = obj.evaluated_get(depsgraph)
    largest_face = None
    largest_area = 0
        
    # get the face with the greatest area in the scene
    for poly in obj_eval.data.polygons:
        success, location, normal, face_index = obj_eval.ray_cast(ray_origin_local, ray_direction_local)
        if success and face_index == poly.index:
            area = poly.area
            if area > largest_area:
                largest_area = area
                largest_face = poly
                
    return largest_face

# get faces with similar normal vectors

def get_face_with_similar_normal(obj, largest_face):

    obj_data = obj.data
    largest_normal = largest_face.normal
    selected_faces = []

    # compare every face of the object
    for face in obj_data.polygons:
        if face == largest_face:
            continue

        # get the face normal
        normal = face.normal

        # compare face normal
        if (largest_normal - normal).length < 0.001:
            selected_faces.append(face.index)

    for face_index in selected_faces:
        obj.data.polygons[face_index].select = True

    return selected_faces
        
# creating a projection face of the object in the center

def create_mesh_from_faces(similar_faces, obj, distance, number_cuts):
        
    # create a new mesh
    mesh = bpy.data.meshes.new("NewMesh")
    new_obj = bpy.data.objects.new("NewFace", mesh)
    bpy.context.collection.objects.link(new_obj)

    bm = bmesh.new()

    for face_index in similar_faces:
        face = obj.data.polygons[face_index]

        vertices = [obj.matrix_world @ obj.data.vertices[vert_idx].co for vert_idx in face.vertices]
        bm_verts = [bm.verts.new(vert) for vert in vertices]

        bm.faces.new(bm_verts)
        bm.faces.ensure_lookup_table()

    bm.to_mesh(mesh)
    bm.free()
    
    # subdivide surface
    bpy.context.view_layer.objects.active = new_obj
    new_obj.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    # create triangular faces to prevent a bad image outcome
    bpy.ops.mesh.quads_convert_to_tris(quad_method='BEAUTY', ngon_method='BEAUTY')
    bpy.ops.mesh.subdivide(number_cuts=number_cuts)
    bpy.ops.object.mode_set(mode='OBJECT')
    
    newmesh = bpy.data.meshes.new("NewMesh")
    annotation_obj = bpy.data.objects.new("AnnotationLayer", newmesh)
    bpy.context.collection.objects.link(annotation_obj)

    bm = bmesh.new()

    for face_index in similar_faces:
        face = obj.data.polygons[face_index]

        vertices = [obj.matrix_world @ obj.data.vertices[vert_idx].co for vert_idx in face.vertices]
        bm_verts = [bm.verts.new(vert) for vert in vertices]

        bm.faces.new(bm_verts)
        bm.faces.ensure_lookup_table()

        # normal direction
        normal = face.normal.normalized()
        normal_world = obj.matrix_world.to_3x3() @ normal

        # new position of face
        for vert in bm_verts:
            vert.co += normal_world * distance * 2
          
    bm.to_mesh(newmesh)
    bm.free()        

    return new_obj, annotation_obj

# create uv perspective from view

def uv_perspective_from_view(new_obj):

    # make sure the object is in object mode
    bpy.ops.object.mode_set(mode='OBJECT')

    # set object to active
    bpy.context.view_layer.objects.active = new_obj
    new_obj.select_set(True)

    # change to edit mode
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action = 'SELECT')
    
    bpy.ops.uv.smart_project(rotate_method='AXIS_ALIGNED_Y')
    bpy.ops.object.mode_set(mode='OBJECT')
    
    uv_map = new_obj.data.uv_layers.new(name="UVMap Project") 
    
    new_obj.data.uv_layers.active_index = len(new_obj.data.uv_layers) - 1  
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    
    # this part of the code has been created by Harry McKenzie and was found here https://blender.stackexchange.com/questions/316333/context-is-incorrect-error-from-project-from-view on 7.6.2024
    #----------------------------------------------------------------------------------------------------------
    
    area_type = 'VIEW_3D'
    areas  = [area for area in bpy.context.window.screen.areas if area.type == area_type]

    with bpy.context.temp_override(
        window=bpy.context.window,
        area=areas[0],
        region=[region for region in areas[0].regions if region.type == 'WINDOW'][0],
        screen=bpy.context.window.screen
    ):
        bpy.ops.uv.project_from_view(camera_bounds=True, correct_aspect=False, scale_to_bounds=False)
    #----------------------------------------------------------------------------------------------------------
    
    bpy.ops.object.mode_set(mode='OBJECT')

# adding the picture of the issue as texture

def adding_issue_material(obj, absolute_snapshot_path):
    
    # ensure the object is selected and active
    bpy.context.view_layer.objects.active = obj
    bpy.context.view_layer.objects.active.select_set(True)

    # create a new material or use the current material
    if not obj.data.materials:
        mat = bpy.data.materials.new(name="NewFaceMaterial")
        obj.data.materials.append(mat)
    else:
        mat = obj.data.materials[0]

    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # clear existing nodes
    for node in nodes:
        nodes.remove(node)

    obj.data.uv_layers.active_index = len(obj.data.uv_layers) - 1  

    # add nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    texture_node = nodes.new(type='ShaderNodeTexImage')
    uv_node = nodes.new(type='ShaderNodeUVMap')
    uv_node.uv_map = obj.data.uv_layers.active.name
    
    # set alpha value
    bsdf_node.inputs['Alpha'].default_value = 1

    # load the image texture
    texture_node.image = bpy.data.images.load(absolute_snapshot_path)
    texture_node.extension = 'CLIP'  

    # connect nodes
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    links.new(texture_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(uv_node.outputs['UV'], texture_node.inputs['Vector'])
    
    bpy.ops.object.mode_set(mode='OBJECT')        

# create a new image texture of the projection face to replace it with a face of lower point amount
    
def get_image_from_orthogonal_view(obj):
    
    # path to folder
    folder_path = bpy.path.abspath('//BCFIssueViewImages')

    # only create a folder
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # ensure the object is selected and active
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)   
    bpy.ops.object.mode_set(mode='OBJECT')  
        
    # set the new uv map as active
    obj.data.uv_layers.active_index = 0

    # create a blanck texture
    new_image = bpy.data.images.new(name="New_Image_Projection_Orthogonal", width=4096, height=4096)

    # set the new texture in material to make changes to the image texture
    if obj.data.materials:
        material = obj.data.materials[0]
    else:
        material = bpy.data.materials.new(name="Material")
        obj.data.materials.append(material)

    if not material.use_nodes:
        material.use_nodes = True

    nodes = material.node_tree.nodes
    texture_node = nodes.new(type='ShaderNodeTexImage')
    texture_node.image = new_image
    material.node_tree.nodes.active = texture_node
    
    # set render engine to cycles and prepare baking
    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.samples = 1
    bpy.context.scene.render.bake.use_pass_direct = False
    bpy.context.scene.render.bake.use_pass_indirect = False
    bpy.context.scene.render.bake.use_pass_color = True
    bpy.context.scene.render.bake.margin = 0
    bpy.context.view_layer.objects.active = obj 
    bpy.ops.object.bake(type='DIFFUSE')
    
    # save the baked image to a file
    # create a unique name with time
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    file_name = f"image_projection_{timestamp}.png"
    file_path = os.path.join(folder_path, file_name)
    new_image.filepath_raw = file_path
    new_image.file_format = 'PNG'
    new_image.save()
    
    obj.data.uv_layers.active_index = len(obj.data.uv_layers) - 1  
    
    return file_path, folder_path

# create a face that fits to the colored part of the image

def create_and_adjust_projection_face(similar_faces, obj, distance, image, source, camera):

    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    
    # create a face
    
    mesh = bpy.data.meshes.new("NewMesh")
    new_obj = bpy.data.objects.new("ProjectionFace", mesh)
    bpy.context.collection.objects.link(new_obj)

    bm = bmesh.new()

    for face_index in similar_faces:
        face = obj.data.polygons[face_index]

        vertices = [obj.matrix_world @ obj.data.vertices[vert_idx].co for vert_idx in face.vertices]
        bm_verts = [bm.verts.new(vert) for vert in vertices]

        bm.faces.new(bm_verts)
        bm.faces.ensure_lookup_table()

        # normal direction
        normal = face.normal.normalized()
        normal_world = obj.matrix_world.to_3x3() @ normal

        # new position of face
        for vert in bm_verts:
            vert.co += normal_world * distance
                          
    bm.to_mesh(mesh)
    bm.free()
    
    # create UV map
    bpy.context.view_layer.objects.active = new_obj
    bpy.context.view_layer.objects.active.select_set(True)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action = 'SELECT')
    bpy.ops.uv.smart_project(rotate_method='AXIS_ALIGNED_Y')

    # apply material
    if not new_obj.data.materials:
        mat = bpy.data.materials.new(name="NewFaceMaterial")
        new_obj.data.materials.append(mat)
    else:
        mat = new_obj.data.materials[0]

    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # clear existing nodes
    for node in nodes:
        nodes.remove(node)

    # add and connect nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    texture_node = nodes.new(type='ShaderNodeTexImage')
    uv_node = nodes.new(type='ShaderNodeUVMap')
    uv_node.uv_map = new_obj.data.uv_layers.active.name
    bsdf_node.inputs['Alpha'].default_value = 1
    texture_node.image = bpy.data.images.load(image)
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    links.new(texture_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(uv_node.outputs['UV'], texture_node.inputs['Vector'])
    bpy.ops.object.mode_set(mode='OBJECT')  
    
    num_vertices_before_cut = len(new_obj.data.vertices)
    
    scene = bpy.context.scene
    cam = camera.data
    frame = cam.view_frame(scene=scene)
    mat = camera.matrix_world.normalized()
    verts = [mat @ v for v in frame]
    
    # create mesh to fit the view of the camera
    mesh = bpy.data.meshes.new(name="CameraViewMesh")
    mesh.from_pydata(verts, [], [(0, 1, 2, 3)])
    mesh.update()
    
    cut_obj = bpy.data.objects.new("CameraViewMesh", mesh)
    bpy.context.collection.objects.link(cut_obj)
    
    bpy.context.view_layer.objects.active = cut_obj
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    
    # turn mesh into curve
    cut_obj.select_set(True)
    bpy.ops.object.convert(target='CURVE')
    bpy.ops.object.select_all(action='DESELECT')
    
    bpy.context.scene.camera = camera
    
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.region_3d.view_perspective = 'CAMERA'
                    
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for region in area.regions:
                if region.type == 'WINDOW':
                    with bpy.context.temp_override(area=area, region=region):
                        # set the new object as active and switch to edit mode
                        bpy.context.view_layer.objects.active = new_obj
                        bpy.ops.object.mode_set(mode='EDIT')
                        bpy.ops.mesh.select_all(action='SELECT')
                        
                        # switch to object mode, select the cut object, and switch back to edit mode
                        bpy.ops.object.mode_set(mode='OBJECT')
                        cut_obj.select_set(True)
                        bpy.context.view_layer.objects.active = new_obj
                        bpy.ops.object.mode_set(mode='EDIT')
                        
                        # do knife project from the camera view
                        bpy.ops.mesh.knife_project(cut_through=True)
                        
                        bpy.ops.object.mode_set(mode='OBJECT')
                        cut_obj.select_set(False)
                        bpy.ops.object.select_all(action='DESELECT')
                        
    num_vertices_after_cut = len(new_obj.data.vertices)
    if num_vertices_before_cut < num_vertices_after_cut:
        # separate the visual part of the issue from the rest
        bpy.context.view_layer.objects.active = new_obj
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.separate(type='SELECTED')
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
                
        bpy.context.view_layer.objects.active = new_obj
        bpy.context.view_layer.objects.active.select_set(True)
        bpy.ops.object.delete()
        bpy.ops.object.select_all(action='DESELECT')
    
    return cut_obj

# make the annotation_obj transparent to prepare the editing with texture paint  
    
def prepare_annotation_layer(obj, folder_path):
    # ensure the object is selected and active
    bpy.context.view_layer.objects.active = obj
    bpy.context.view_layer.objects.active.select_set(True)

    # make sure the object is in object mode
    bpy.ops.object.mode_set(mode='OBJECT')

    # change to edit mode
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action = 'SELECT')
    
    # create UV map
    bpy.ops.uv.unwrap(method='ANGLE_BASED', margin=0)


    # create a new material or use the current material
    if not obj.data.materials:
        mat = bpy.data.materials.new(name="NewFaceMaterial")
        obj.data.materials.append(mat)
    else:
        mat = obj.data.materials[0]

    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # clear existing nodes
    for node in nodes:
        nodes.remove(node)

    # add nodes
    output_node = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf_node = nodes.new(type='ShaderNodeBsdfPrincipled')
    texture_node = nodes.new(type='ShaderNodeTexImage')
    uv_node = nodes.new(type='ShaderNodeUVMap')
    uv_node.uv_map = obj.data.uv_layers.active.name
    
    # create a new image as base for annotations
    image_name = "New_Image"
    width = 1024
    height = 1024
    color = (0, 0, 0, 0)
    image = bpy.data.images.new(name=image_name, width=width, height=height, alpha=True, float_buffer=False)
    image.generated_color = color
    
    # save the image to a file
    
    # create a unique name with time
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    file_name = f"issue_annotation_{timestamp}.png"
    file_path = os.path.join(folder_path, file_name)
    image.filepath_raw = file_path
    image.file_format = 'PNG'
    image.save()
    
    texture_node.image = image
    
    # connect nodes
    links.new(bsdf_node.outputs['BSDF'], output_node.inputs['Surface'])
    links.new(texture_node.outputs['Color'], bsdf_node.inputs['Base Color'])
    links.new(texture_node.outputs['Alpha'], bsdf_node.inputs['Alpha'])
    links.new(uv_node.outputs['UV'], texture_node.inputs['Vector'])

    bpy.ops.object.mode_set(mode='OBJECT')
    
# create face from camera view

def main_set_camera_and_create_face(context):
    
    # create camera
    camera, camera_name, absolute_snapshot_path = main_create_camera_with_BCF_data(context)

    # find the first object in the view of the camera
    first_object = get_first_object_in_view(camera)
    if first_object:
        print(f"Das gefundene Objekt ist '{first_object.name}'.")
        # get the largest visible face of the object
        largest_face = get_largest_visible_face(camera, first_object)

        if largest_face:
            # get similar faces with the same normal vectors
            similar_faces = get_face_with_similar_normal(first_object,largest_face)
            
            if similar_faces:
                # create a new face at a distance in the direction of the face normal
                new_object, annotation_obj = create_mesh_from_faces(similar_faces, first_object, distance_between_layers, number_cuts)
                uv_perspective_from_view(new_object)
                print(f"Neue Fläche für Objekt '{first_object.name}' basierend auf der größten sichtbaren Fläche erzeugt.")
            else:
                print("No similar faces found.")
        else:
            print("No visible faces found.")
    else:
        print("No object found in the direction of the camera.")
    
    adding_issue_material(new_object, absolute_snapshot_path)
    orthogonal_image, image_folder_path = get_image_from_orthogonal_view(new_object)
    cut_rectangle = create_and_adjust_projection_face(similar_faces, first_object, distance_between_layers, orthogonal_image, new_object, camera)
    bpy.ops.object.select_all(action='DESELECT')
    prepare_annotation_layer(annotation_obj, image_folder_path)
    bpy.ops.object.select_all(action='DESELECT')
    
    # delet anything that will not be used
    bpy.context.view_layer.objects.active = camera
    bpy.context.view_layer.objects.active.select_set(True)
    bpy.context.view_layer.objects.active = new_object
    bpy.context.view_layer.objects.active.select_set(True)
    bpy.context.view_layer.objects.active = cut_rectangle
    bpy.context.view_layer.objects.active.select_set(True)
    bpy.ops.object.delete()
    bpy.ops.object.select_all(action='DESELECT')
    
# just creating the camera    

def main_create_camera(context):
    
    camera, camera_name, absolute_snapshot_path = main_create_camera_with_BCF_data(context)
    bpy.context.scene.camera = camera
    
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D':
                    space.region_3d.view_perspective = 'CAMERA'
    
class SetFocalLength(bpy.types.PropertyGroup):
        value: bpy.props.FloatProperty(
        name="Set Focal Length",
        description="Focal Length in mm. Base Values with 0.00",
        default=0.00,
        min=0.00,
        max=1000.00
    )
    
class SetSensorWidth(bpy.types.PropertyGroup):
        value: bpy.props.FloatProperty(
        name="Set Sensor Width",
        description="Sensor Width in mm. Base Values with 0.00",
        default=0.00,
        min=0.00,
        max=1000.00
    )
    
class CreateIssueCamera(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "object.create_issue_camera"
    bl_label = "Create Camera"

    def execute(self, context):
        main_create_camera(context)
        return {'FINISHED'}   

class SetCameraAndCreateFace(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "object.set_camera_and_create_face"
    bl_label = "Create Projection Face"

    def execute(self, context):
        main_set_camera_and_create_face(context)
        return {'FINISHED'}

# create a layout for buttons in properties scene

def menu_func(self, context):
    self.layout.operator(SetCameraAndCreateFace.bl_idname, text=SetCameraAndCreateFace.bl_label)

class LayoutPanel(bpy.types.Panel):
    """Creates a Panel in the scene context of the properties editor"""
    bl_label = "BCF Issue View"
    bl_idname = "SCENE_PT_layout"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        # import BCF Markup file of the issue with viewpoint
        layout.prop(scene, "import_filepath", text="Import BCF Issue Pircture")
        row = layout.row()
        
        # set focal length
        layout.prop(scene.focal_length, "value")
        row = layout.row()
         
        # set sensor width
        layout.prop(scene.sensor_width, "value")
        row = layout.row()
        
        # create projection face button
        layout.label(text="Create Issue Camera:")
        row = layout.row()
        row.scale_y = 1
        row.operator("object.create_issue_camera")
        
        # create projection face button
        layout.label(text="Create Issue Visualisation:")
        row = layout.row()
        row.scale_y = 1
        row.operator("object.set_camera_and_create_face")

# register classes
def register():
    bpy.utils.register_class(SetCameraAndCreateFace)  
    bpy.types.VIEW3D_MT_object.append(menu_func)
    bpy.utils.register_class(LayoutPanel)
    bpy.utils.register_class(CreateIssueCamera)
    bpy.utils.register_class(SetFocalLength)
    bpy.utils.register_class(SetSensorWidth)
    # temporary data
    bpy.types.Scene.import_filepath = bpy.props.StringProperty(subtype="FILE_PATH")
    bpy.types.Scene.focal_length = bpy.props.PointerProperty(type=SetFocalLength)
    bpy.types.Scene.sensor_width = bpy.props.PointerProperty(type=SetSensorWidth)

def unregister():
    bpy.utils.unregister_class(SetCameraAndCreateFace)
    bpy.types.VIEW3D_MT_object.remove(menu_func)
    bpy.utils.unregister_class(LayoutPanel)
    bpy.utils.unregister_class(CreateIssueCamera)
    bpy.utils.unregister_class(SetFocalLength)
    bpy.utils.unregister_class(SetSensorWidth)
    # temporary data
    del bpy.types.Scene.import_filepath
    del bpy.types.Scene.focal_length
    del bpy.types.Scene.sensor_width

if __name__ == "__main__":
    register()