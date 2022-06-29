import bpy
import bmesh
import zipfile
from io import TextIOWrapper
from math import *
from itertools import chain
from mathutils import *

bl_info = {
    "name": "Multi-Face Parallax Mapping",
    "author": "DongLao Warior",
    "version": (1, 0),
    "blender": (2, 76, 0),
    "location": "View3D > T-Panel > MFPM",
    "description": "Generate textures, vertex cols and osl script for MFPM",
    "warning": "",
    "wiki_url": "",
    "category": "",
}

# function

def RemoveCols(context):
    vertex_colors = context.active_object.data.vertex_colors
    while len(vertex_colors) > 0:
        vertex_colors.remove(vertex_colors[0])


def VertexCols(context, MaxSize):
    RemoveCols(context)
    mesh = context.active_object.data
    mesh.vertex_colors.new('IntCenter')
    mesh.vertex_colors.new('FloatCenter')

    faces = mesh.polygons

    index = 0

    gam = [
        0, 8, 20, 29, 33, 38, 40, 46, 49, 53, 56, 58, 61, 64, 66, 68, 70, 73, 75, 77, 79, 81, 82, 84, 86, 88, 90, 91, 93, 94,
        96, 98, 99, 101, 102, 103, 105, 106, 108, 109, 110, 111, 113, 114, 115, 116, 118, 119, 120, 121, 122, 123, 125, 126, 
        127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147
    ]

    def gammacorrect(a):
        gamma = [0,0,0]
        for i in range(3):
            gamma[i] = gam[a[i]] / 255.0
        return tuple(gamma) 

    for i in range(len(faces)):
        x, y, z = faces[i].center
        xcode = (x + MaxSize) * 37 / MaxSize
        red1 = floor(xcode)
        red2 = round((xcode % 1) * 74)
        ycode = (y + MaxSize) * 37 / MaxSize
        green1 = floor(ycode)
        green2 = round((ycode % 1) * 74)
        zcode = (z + MaxSize) * 37 / MaxSize
        blue1 = floor(zcode)
        blue2 = round((zcode % 1) * 74)

        rgb01 = (red1, green1, blue1)
        rgb02 = (red2, green2, blue2)

        temp = index
        for k in range(3):
            mesh.vertex_colors[0].data[index].color = gammacorrect(rgb01)
            index += 1

        index = temp
        for k in range(3):
            mesh.vertex_colors[1].data[index].color = gammacorrect(rgb02)
            index += 1

def CoordinateMaps(context, folder, maxsize):
    FOLDER = folder
    BOUND_MIN =  Vector(context.active_object.bound_box[0])
    DIMENSIONS = context.active_object.dimensions
    MAX_SIZE = maxsize

    findex = [[[0] * 128 for x in range(128)] for y in range(128)]
    
    ob = context.active_object
    mesh = ob.data
    bme = bmesh.new()
    bme.from_mesh(mesh)
    bmefaces = bme.faces
    bmeverts = bme.verts
    uvCoord = mesh.uv_layers.active.data
    faces = mesh.polygons
    vertices = mesh.vertices

    
    def Angle(Vector1, Vector2):
        return acos(Vector1.normalized().dot(Vector2.normalized()))

    def Rotate(MainPnt, RotAng, Axis):
        rote = Quaternion(Axis, RotAng)
        NewPnt = Vector(MainPnt)
        NewPnt.rotate(rote)
        return NewPnt

    def Parametric(Direction, DirU, DirV):
        CurrentNor = DirU.cross(DirV)
        RotAxis = CurrentNor.cross(Vector((0,0,1)))
        RotCoord = Angle(CurrentNor, Vector((0,0,1)))

        NewDirection = Rotate(Direction, RotCoord, RotAxis)
        NewDirU = Rotate(DirU, RotCoord, RotAxis)
        NewDirV = Rotate(DirV, RotCoord, RotAxis)

        DetM = NewDirU[0] * NewDirV[1] - NewDirU[1] * NewDirV[0]
        if DetM == 0: return (0, 0)
        return (
            (NewDirection[0] * NewDirV[1] - NewDirection[1] * NewDirV[0]) / DetM,
            (NewDirection[1] * NewDirU[0] - NewDirection[0] * NewDirU[1]) / DetM
        )
    
    def CheckInside(plain, mainpnt):
        fn = plain.normal
        vn = [plain.verts[x].normal for x in range(3)]
        vc = [plain.verts[x].co for x in range(3)]
        dz = (mainpnt - vc[0]).dot(fn)
        newvn = [vc[x] + vn[x] * dz / vn[x].dot(fn) for x in range(3)]
        para = Parametric(mainpnt - newvn[2], newvn[0] - newvn[2], newvn[1] - newvn[2])
        if para[0] < 0 or para[1] < 0 or para[0] + para[1] > 1: return False
        return True

    def MulWise(Vec1, Vec2):
        return Vector((Vec1[0]*Vec2[0], Vec1[1]*Vec2[1], Vec1[2]*Vec2[2]))

    CUR_POS = Vector((0,0,0))
    for i in range(128):
        for j in range(128):
            for k in range(128):
                CUR_POS = BOUND_MIN + MulWise(DIMENSIONS, (k + 0.5, j + 0.5, i + 0.5)) / 128
                closest = ob.closest_point_on_mesh(CUR_POS)
                faceindex = closest[-1]
                intersect = closest[0]
                facev = faces[faceindex].vertices
                bme.faces.ensure_lookup_table()
                bedge = bmefaces[faceindex].edges
                stop = False
                for l in range(3):
                    vindex = facev[l]
                    vcoord = vertices[vindex]
                    if vcoord.co == intersect:
                        bme.verts.ensure_lookup_table()
                        linkface = bmeverts[vindex].link_faces
                        for m in range(len(linkface)):
                            if CheckInside(linkface[m], CUR_POS):
                                faceindex = linkface[m].index
                                break
                        stop = True
                        break  
                if not stop: 
                    for l in range(3):
                        curedge = bedge[l]
                        ver1, ver2 = curedge.verts[0].co, curedge.verts[1].co
                        
                        if (ver1 - intersect).cross(intersect - ver2).length == 0:
                            face1, face2 = curedge.link_faces
                            faceindex = face1.index if CheckInside(face1, CUR_POS) else face2.index
                            break
                
                findex[i][j][k] = faceindex

    def GenerateImg(NAME, CallBack, *props):
        newImage = bpy.data.images.new(NAME, 2048, 1024, False)

        def generatePixels():
            for i in range(128):
                for j in range(128):
                    for k in range(128):
                        red, green, blue = CallBack(findex[i][j][k], *props)
                        yield red, green, blue, 1
                        
        newImage.pixels = tuple(chain.from_iterable(generatePixels()))
        newImage.filepath_raw = FOLDER + NAME + '.png'
        newImage.save()

    # IMG1

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v1 = vertices[face[0]].co

        colval1 = (v1[0] + MAX_SIZE) * 127.5 / MAX_SIZE
        red = floor(colval1) / 255
        colval2 = (v1[1] + MAX_SIZE) * 127.5 / MAX_SIZE
        green = floor(colval2) / 255
        colval3 = (v1[2] + MAX_SIZE) * 127.5 / MAX_SIZE
        blue = floor(colval3) / 255

        return (red, green, blue)

    GenerateImg('IMG1', CallBack, faces)

    # IMG2

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v1 = vertices[face[0]].co

        colval1 = (v1[0] + MAX_SIZE) * 127.5 / MAX_SIZE
        red = round((colval1 - floor(colval1)) * 255) / 255
        colval2 = (v1[1] + MAX_SIZE) * 127.5 / MAX_SIZE
        green = round((colval2 - floor(colval2)) * 255) / 255
        colval3 = (v1[2] + MAX_SIZE) * 127.5 / MAX_SIZE
        blue = round((colval3 - floor(colval3)) * 255) / 255  

        return (red, green, blue)

    GenerateImg('IMG2', CallBack, faces)

    # IMG3

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v2 = vertices[face[1]].co

        colval1 = (v2[0] + MAX_SIZE) * 127.5 / MAX_SIZE
        red = floor(colval1) / 255
        colval2 = (v2[1] + MAX_SIZE) * 127.5 / MAX_SIZE
        green = floor(colval2) / 255
        colval3 = (v2[2] + MAX_SIZE) * 127.5 / MAX_SIZE
        blue = floor(colval3) / 255

        return (red, green, blue)

    GenerateImg('IMG3', CallBack, faces)

    # IMG4

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v2 = vertices[face[1]].co

        colval1 = (v2[0] + MAX_SIZE) * 127.5 / MAX_SIZE
        red = round((colval1 - floor(colval1)) * 255) / 255
        colval2 = (v2[1] + MAX_SIZE) * 127.5 / MAX_SIZE
        green = round((colval2 - floor(colval2)) * 255) / 255
        colval3 = (v2[2] + MAX_SIZE) * 127.5 / MAX_SIZE
        blue = round((colval3 - floor(colval3)) * 255) / 255  

        return (red, green, blue)

    GenerateImg('IMG4', CallBack, faces)

    # IMG5

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v3 = vertices[face[2]].co

        colval1 = (v3[0] + MAX_SIZE) * 127.5 / MAX_SIZE
        red = floor(colval1) / 255
        colval2 = (v3[1] + MAX_SIZE) * 127.5 / MAX_SIZE
        green = floor(colval2) / 255
        colval3 = (v3[2] + MAX_SIZE) * 127.5 / MAX_SIZE
        blue = floor(colval3) / 255

        return (red, green, blue)

    GenerateImg('IMG5', CallBack, faces)

    # IMG6

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v3 = vertices[face[2]].co

        colval1 = (v3[0] + MAX_SIZE) * 127.5 / MAX_SIZE
        red = round((colval1 - floor(colval1)) * 255) / 255
        colval2 = (v3[1] + MAX_SIZE) * 127.5 / MAX_SIZE
        green = round((colval2 - floor(colval2)) * 255) / 255
        colval3 = (v3[2] + MAX_SIZE) * 127.5 / MAX_SIZE
        blue = round((colval3 - floor(colval3)) * 255) / 255      

        return (red, green, blue)

    GenerateImg('IMG6', CallBack, faces)

    # IMG7

    def CallBack(faceindex, faces, uvCoord):
        indices = faces[faceindex].loop_indices
        v1 = uvCoord[indices[0]].uv

        colval1 = v1[0] * 255
        floorval1 = floor(colval1)
        red = floorval1 / 255

        colval2 = v1[1] * 255
        floorval2 = floor(colval2)
        green = floorval2 / 255
    
        blue =  round((colval1 - floorval1) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG7', CallBack, faces, uvCoord)

    # IMG8

    def CallBack(faceindex, faces, uvCoord):
        indices = faces[faceindex].loop_indices
        v1 = uvCoord[indices[0]].uv
        v2 = uvCoord[indices[1]].uv

        colval1 = v2[0] * 255
        floorval1 = floor(colval1)
        red = floorval1 / 255

        colval2 = v2[1] * 255
        floorval2 = floor(colval2)
        green = floorval2 / 255

        colval3 = v1[1] * 255
        floorval3 = floor(colval3)
        blue =  round((colval3 - floorval3) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG8', CallBack, faces, uvCoord)

    # IMG9

    def CallBack(faceindex, faces, uvCoord):
        indices = faces[faceindex].loop_indices
        v2 = uvCoord[indices[1]].uv
        v3 = uvCoord[indices[2]].uv

        colval1 = v3[0] * 255
        floorval1 = floor(colval1)
        red = floorval1 / 255

        colval2 = v3[1] * 255
        floorval2 = floor(colval2)
        green = floorval2 / 255

        colval3 = v2[0] * 255
        floorval3 = floor(colval3)
        blue =  round((colval3 - floorval3) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG9', CallBack, faces, uvCoord)

    # IMG0

    def CallBack(faceindex, faces, uvCoord):
        indices = faces[faceindex].loop_indices
        v2 = uvCoord[indices[1]].uv
        v3 = uvCoord[indices[2]].uv

        colval1 = v3[0] * 255
        floorval1 = floor(colval1)
        red =  round((colval1 - floorval1) * 255) / 255

        colval2 = v3[1] * 255
        floorval2 = floor(colval2)
        green =  round((colval2 - floorval2) * 255) / 255

        colval3 = v2[1] * 255
        floorval3 = floor(colval3)
        blue =  round((colval3 - floorval3) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG0', CallBack, faces, uvCoord)

    # IMG11

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v1 = vertices[face[0]].normal
        red = floor((v1[0] + 1) / 2 * 255) / 255
        green = floor((v1[1] + 1) / 2 * 255) / 255
        blue = floor((v1[2] + 1) / 2 * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG11', CallBack, faces)

    # IMG12

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v1 = vertices[face[0]].normal
        colval1 = (v1[0] + 1) / 2 * 255
        red = round((colval1 - floor(colval1)) * 255) / 255
        colval2 = (v1[1] + 1) / 2 * 255
        green = round((colval2 - floor(colval2)) * 255) / 255
        colval3 = (v1[2] + 1) / 2 * 255
        blue = round((colval3 - floor(colval3)) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG12', CallBack, faces)

    # IMG13

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v2 = vertices[face[1]].normal
        red = floor((v2[0] + 1) / 2 * 255) / 255
        green = floor((v2[1] + 1) / 2 * 255) / 255
        blue = floor((v2[2] + 1) / 2 * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG13', CallBack, faces)

    # IMG14

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v2 = vertices[face[1]].normal
        colval1 = (v2[0] + 1) / 2 * 255
        red = round((colval1 - floor(colval1)) * 255) / 255
        colval2 = (v2[1] + 1) / 2 * 255
        green = round((colval2 - floor(colval2)) * 255) / 255
        colval3 = (v2[2] + 1) / 2 * 255
        blue = round((colval3 - floor(colval3)) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG14', CallBack, faces)

    # IMG15

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v3 = vertices[face[2]].normal
        red = floor((v3[0] + 1) / 2 * 255) / 255
        green = floor((v3[1] + 1) / 2 * 255) / 255
        blue = floor((v3[2] + 1) / 2 * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG15', CallBack, faces)

    # IMG16

    def CallBack(faceindex, faces):
        face = faces[faceindex].vertices
        v3 = vertices[face[2]].normal
        colval1 = (v3[0] + 1) / 2 * 255
        red = round((colval1 - floor(colval1)) * 255) / 255
        colval2 = (v3[1] + 1) / 2 * 255
        green = round((colval2 - floor(colval2)) * 255) / 255
        colval3 = (v3[2] + 1) / 2 * 255
        blue = round((colval3 - floor(colval3)) * 255) / 255

        return (red, green, blue)

    GenerateImg('IMG16', CallBack, faces)

def OslMFPM(context, osl_dir, mat_name, max_size, depth, light, heightname, anglename, vernorname, diffname, normalname, pbrname):
    osltext = bpy.data.texts.new('MFPM.osl')
    with zipfile.ZipFile(osl_dir) as z:
        with TextIOWrapper(z.open('MFPMosl.txt'), encoding='utf-8') as oslscript:
            for line in oslscript:
                osltext.write(line)

    osltext = bpy.data.texts.new('InitPosition.osl')
    with zipfile.ZipFile(osl_dir) as z:
        with TextIOWrapper(z.open('initPosition.txt'), encoding='utf-8') as oslscript:
            for line in oslscript:
                osltext.write(line)

    osltext = bpy.data.texts.new('InitCenter.osl')
    with zipfile.ZipFile(osl_dir) as z:
        with TextIOWrapper(z.open('initCenter.txt'), encoding='utf-8') as oslscript:
            for line in oslscript:
                osltext.write(line)

    osltext = bpy.data.texts.new('CorrectNormal.osl')
    with zipfile.ZipFile(osl_dir) as z:
        with TextIOWrapper(z.open('correctNormal.txt'), encoding='utf-8') as oslscript:
            for line in oslscript:
                osltext.write(line)

    mat = bpy.data.materials.get(mat_name) or bpy.data.materials.new(mat_name)
    context.active_object.data.materials.append(mat)
    mat.use_nodes = True 
    node_tree = mat.node_tree
    nodes = node_tree.nodes
    node = [None] * 14
    node[0] = nodes.new('ShaderNodeScript')
    node[0].script = bpy.data.texts['MFPM.osl']
    node[0].inputs[0].default_value = heightname
    node[0].inputs[1].default_value = anglename
    node[0].inputs[2].default_value = vernorname
    node[0].inputs[3].default_value = diffname
    node[0].inputs[4].default_value = normalname
    node[0].inputs[5].default_value = pbrname
    node[0].inputs['MAXSIZE'].default_value = max_size
    node[0].inputs['Depth'].default_value = depth
    node[0].inputs['Light'].default_value = light
    node[0].location = (-300,0)
    node[1] = nodes.new('ShaderNodeScript')
    node[1].script = bpy.data.texts['InitPosition.osl']
    node[1].location = (-600,-380)
    node[2] = nodes.new('ShaderNodeScript')
    node[2].script = bpy.data.texts['InitCenter.osl']
    node[2].inputs['MaxSize'].default_value = max_size
    node[2].location = (-600,-580)
    node[3] = nodes.new('ShaderNodeTexCoord')
    node[3].location = (-800,-440)
    node[4] = nodes.new('ShaderNodeAttribute')
    node[4].attribute_name = 'IntCenter'
    node[4].location = (-800, -540)
    node[5] = nodes.new('ShaderNodeAttribute')
    node[5].attribute_name = 'FloatCenter'
    node[5].location = (-800, -640)
    node[6] = nodes.new('ShaderNodeNewGeometry')
    node[6].location = (-900, -140)
    node[7] = nodes.new('ShaderNodeVectorTransform')
    node[7].location = (-600, -140)
    node[8] = nodes.new('ShaderNodeVectorTransform')
    node[8].location = (-600, -320)
    node[9] = nodes.new('ShaderNodeScript')
    node[9].script = bpy.data.texts['CorrectNormal.osl']
    node[9].location = (-465,-325)
    node[10] = nodes.new('ShaderNodeMixRGB')
    node[10].blend_type = 'MULTIPLY'
    node[10].inputs[0].default_value = 0.8
    node[10].location = (200, -30)
    node[11] = nodes.new('ShaderNodeBsdfDiffuse')
    node[11].location = (400, -30)
    node[12] = nodes.new('ShaderNodeBsdfTransparent')
    node[12].location = (400, -130)
    node[13] = nodes.new('ShaderNodeMixShader')
    node[13].location = (600, -80)

    node_tree.links.new(node[3].outputs["Generated"], node[1].inputs["Vector"])
    node_tree.links.new(node[1].outputs["InitPosition"], node[0].inputs["InitPosition"])
    node_tree.links.new(node[4].outputs["Vector"], node[2].inputs["IntCenter"])
    node_tree.links.new(node[5].outputs["Vector"], node[2].inputs["FloatCenter"])
    node_tree.links.new(node[2].outputs["InitCenter"], node[0].inputs["InitCenter"])
    node_tree.links.new(node[6].outputs["Incoming"], node[7].inputs["Vector"])
    node_tree.links.new(node[7].outputs["Vector"], node[0].inputs["Incoming"])
    node_tree.links.new(node[6].outputs["True Normal"], node[8].inputs["Vector"])
    node_tree.links.new(node[8].outputs["Vector"], node[9].inputs["TrueNormal"])
    node_tree.links.new(node[3].outputs["Object"], node[0].inputs["TruePosition"])
    node_tree.links.new(node[3].outputs["UV"], node[0].inputs["UVMap"])
    node_tree.links.new(node[6].outputs["Backfacing"], node[9].inputs["BackFace"])
    node_tree.links.new(node[9].outputs["CorrectTrueNormal"], node[0].inputs["TrueNormal"])
    node_tree.links.new(node[0].outputs["Diffuse"], node[10].inputs["Color1"])
    node_tree.links.new(node[0].outputs["Shadow"], node[10].inputs["Color2"])
    node_tree.links.new(node[10].outputs["Color"], node[11].inputs["Color"])
    node_tree.links.new(node[11].outputs["BSDF"], node[13].inputs[2])
    node_tree.links.new(node[12].outputs["BSDF"], node[13].inputs[1])
    node_tree.links.new(node[0].outputs["Alpha"], node[13].inputs[0])
    node_tree.links.new(node[13].outputs["Shader"], nodes["Material Output"].inputs["Surface"])

    dimensions = context.active_object.dimensions
    node[1].inputs["Dimension"].default_value = dimensions
    bpy.ops.view3d.snap_cursor_to_selected()
    mw = Matrix(context.active_object.matrix_world)
    Matrix.invert(mw)
    bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')
    bounding_box = mw * context.active_object.location
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    node[1].inputs["BoundingBox"].default_value = bounding_box
    node[0].inputs["BoundMax"].default_value = bounding_box + dimensions / 2
    node[0].inputs["BoundMin"].default_value = bounding_box - dimensions / 2

    for n in range(len(node)):
        if n > 0:
            node[n].hide = True
            for s in node[n].inputs:
                s.hide = True
            for s in node[n].outputs:
                s.hide = True

# operator

class RemoveNCButton(bpy.types.Operator):
    bl_idname = "parallax.removevertexcols"
    bl_label = "Remove All Vertex Cols"

    def execute(self, context):
        RemoveCols(context)
        return {'FINISHED'}

class InitNCButton(bpy.types.Operator):
    bl_idname = "parallax.addvertexcols"
    bl_label = "Generate Vertex Cols"

    MaxSize = bpy.props.FloatProperty(name = 'Max Size', default = 10)

    def execute(self, context):
        MaxSize = self.MaxSize
        VertexCols(context, MaxSize)
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

class AngleMapsButton(bpy.types.Operator):
    bl_idname = "parallax.anglemaps"
    bl_label = "Generate Angle Maps"

    folder = bpy.props.StringProperty(name = 'Save Location', default = 'E:\MFPM\\textures\\')
    maxsize = bpy.props.FloatProperty(name = 'Max Object Size', default = 10)

    def execute(self, context):
        folder = self.folder
        maxsize = self.maxsize
        CoordinateMaps(context, folder, maxsize)
        return {'FINISHED'}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

class OslButton(bpy.types.Operator):
    bl_idname = "parallax.osl"
    bl_label = "Create Osl Script"

    osl_dir = bpy.props.StringProperty(name = 'Addon Directory', default = 'E:\MFPM\MFPM.zip')
    mat_name = bpy.props.StringProperty(name = 'Material Name', default = 'MFPM Material')
    max_size = bpy.props.FloatProperty(name = 'Max Size', default = 10)
    depth = bpy.props.FloatProperty(name = 'Depth', default = 0.3)
    light = bpy.props.FloatVectorProperty(name = 'Light Position', default = (0, 0, 0), subtype = 'XYZ')
    heightname = bpy.props.StringProperty(name = 'Height Map', default = '//HeightMap.png')
    anglename = bpy.props.StringProperty(name = 'Angle Map', default = '//IMG.png')
    vernorname = bpy.props.StringProperty(name = 'Vertex Normal Map', default = '//VertexNormal.png')
    diffname = bpy.props.StringProperty(name = 'Diffuse Map', default = '//DiffuseMap.png')
    normalname = bpy.props.StringProperty(name = 'Normal Map', default = '//NormalMap.png')
    pbrname = bpy.props.StringProperty(name = 'PBR Map', default = '//PBRMap.png')

    def execute(self, context):
        mat_name = self.mat_name
        osl_dir = self.osl_dir
        max_size = self.max_size
        depth = self.depth
        light = self.light
        heightname = self.heightname
        anglename = self.anglename
        vernorname = self.vernorname
        diffname = self.diffname
        normalname = self.normalname
        pbrname = self.pbrname
        OslMFPM(context, osl_dir, mat_name, max_size, depth, light, heightname, anglename, vernorname, diffname, normalname, pbrname)
        return {'FINISHED'}

    def invoke(self, context, event):
        self.light = bpy.data.objects['Point'].location
        return context.window_manager.invoke_props_dialog(self)
# panel

class MFPMPanel(bpy.types.Panel):
    bl_label = "MFPM Panel"
    bl_idname = "MFPM"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    bl_category = "MFPM"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.label(text = 'Vertex Cols')
        row = layout.row()
        row.operator(RemoveNCButton.bl_idname, text='Remove All Vertex Cols', icon='COLOR')
        row = layout.row()
        row.operator(InitNCButton.bl_idname, text='Generate Vertex Cols', icon='COLOR')
        row = layout.row()
        row.label(text = 'Angle Maps')
        row = layout.row()
        row.operator(AngleMapsButton.bl_idname, text='Generate Angle Maps', icon='IMAGE_COL')
        row = layout.row()
        row.operator(OslButton.bl_idname, text='Create Material', icon='MATERIAL')

# register

_classes = [
    RemoveNCButton,
    InitNCButton,
    AngleMapsButton,
    OslButton,
    MFPMPanel
]

def register():
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in _classes:
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()




