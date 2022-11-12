import scipy.spatial as spatial
import numpy as np
import maya.cmds as cmd
import maya.mel as mel
import maya.api.OpenMaya as om
import random
from functools import partial
import traceback
import sys


maya_useNewAPI = True
# Math functions
def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)

def floatJitter(a, b):
    return a + (random.uniform(0, b) - b / 2)

class RopeSystem(object):
    def __init__(self, name,object,tool):
        self.tool = tool
        self.name = name
        self.ropes = []
        #mesh & Convex
        self.selectedObject = []
        self.convexObject = createConvexHull( getVertexPosition(object),object)
        print("Created System: " + self.name)
        #rings
        self.rings = []

        #Systems
        self.isActive = True
        self.ropeSystem = []
        self.ropeSystemGroup = "-"

    def create_rope(self, subdivisions=14):
        # Create Locator for each ring-
        ropelocator = createLocator(self.convexObject)

        # Get Sizes of object based on bounding_box of convex surface
        convex_surface = cmd.listRelatives(self.convexObject, shapes=True)[0]
        bounding_box = cmd.exactWorldBoundingBox(convex_surface)
        sizes = 0
        for i in range(2):
            csize = abs(bounding_box[i] - bounding_box[i + 3])
            if csize > sizes: sizes = csize

        radius = sizes / 2
        radius *= np.sqrt(2)

        name = self.name +"_Rope_"+ str(len(self.ropes))
        name = name.replace(" ", "")

        new_ring = RopeRing(name,
                    cmd.circle(normal=(0, 1, 0), r=radius * 1.1, s=subdivisions, n=name), ropelocator,self)
        temp_constraint = cmd.parentConstraint(ropelocator, new_ring.mesh, mo=0)

        # save original point positions in array on the ring for update features
        ring_nurbs = cmd.listRelatives(new_ring.mesh, shapes=True)[0]
        for point in range(subdivisions):
            new_ring.originalPositions.append(cmd.pointPosition(ring_nurbs + ".cv" + str([point]), w=True))

        bounds = cmd.exactWorldBoundingBox(self.convexObject)
        new_ring.bounds_size = bounds[3]+bounds[4]+bounds[5]/3
        self.ray_ring(new_ring)
        self.createCable(new_ring,0)
        self.updateSystem(new_ring)
        self.ropes.append(new_ring)

        return new_ring

    def ray_ring(self,ring:object, reset=False):
        ring_nurbs = cmd.listRelatives(ring.mesh, shapes=True)[0]
        points = cmd.getAttr(ring_nurbs + '.spans')
        convex_center_position = cmd.objectCenter(self.convexObject, gl=True)

        selection = cmd.select(self.convexObject)
        sel = om.MGlobal.getActiveSelectionList()
        dagPath = sel.getDagPath(0)
        fnMesh = om.MFnMesh(dagPath)

        # Global Setup values
        faceIDs = None
        triIDs = None
        IDsSorted = False
        testbothdir = False
        maxParam = 999999
        worldSpace = om.MSpace.kWorld
        accelPar = fnMesh.autoUniformGridParams()
        sortHits = True
        hitPoints = om.MFloatPointArray()
        hitRayParams = om.MFloatArray()
        hitFaces = om.MIntArray()
        hitTris = None
        hitBarys1 = None
        hitBarys2 = None
        tolerance = 0.0001

        for point in range(points):
            # get Cv pos
            pointPos = cmd.pointPosition(ring_nurbs + ".cv" + str([point]), w=True)

            if (reset):
                pointPos = ring.originalPositions[point][0], pointPos[1], ring.originalPositions[point][2]

            pointSource = np.array([pointPos[0], pointPos[1], pointPos[2]])
            destinationPos = np.array([convex_center_position[0], pointPos[1], convex_center_position[2]])
            rayDir = destinationPos - pointSource
            raySource = om.MFloatPoint(pointPos[0], pointPos[1], pointPos[2], 1.0)
            ray_dir = om.MFloatVector(rayDir[0], rayDir[1], rayDir[2])

            hit = fnMesh.closestIntersection(raySource, ray_dir,
                                             worldSpace, 99999, False)

            if hit:
                hitPoint, hitRayParam, hitFace, hitTriangle, hitBary1, hitBary2 = hit
                x, y, z, _ = hitPoint
                cmd.move(x, y, z, ring_nurbs + ".cv" + str([point]), wd=True)

        selection = cmd.select(ring.locator)

    def deleteRope(self,rope2delete):
        delete(rope2delete)

    def createCable(self,ring, sweepType: int = 0):
        cmd.select(ring.mesh[0])
        sweepMesh = mel.eval('sweepMeshFromCurve -oneNodePerCurve ' + str(sweepType))
        cmd.rename(sweepMesh,'SM_'+ring.name)
        self.tool.indexRope += 1
        ring.index = self.tool.indexRope
        ring.parent_Rings()

    def updateSystem(self,ring:object):
        name = ring.name.replace("'", "")
        name = name.replace("[", "")
        name = name.replace("]", "")
        cmd.scriptJob(attributeChange=[ring.locator[0]+'.translate', partial(self.ray_ring,ring,True)],
                      protected=True)

class RopeRing(object):
    def __init__(self,name,mesh,locator,parent):
        self.name = name
        self.mesh = mesh
        self.parent = parent
        self.originalPositions = []
        self.locator = locator
        self.curveWarp = []
        self.sweep = False
        self.bounds_size = 1
        self.sweepNode =[]
        self.index = -1
        #curveWrap/sweep variables
        self.radius = 0.05
        self.collumn_subdivisions = 8
        self.row_subdivisions = 30
        self.rotation = 0
        self.twist = 0
        self.twistRear = 0
        self.taperAm = 1

    def add_sweep_node(self,sweep_name):
        self.sweepNode=sweep_name

    def set_sweepattributes(self,tool):
        sweepMeshName = 'SM_' + self.name
        cmd.setAttr(sweepMeshName+".interpolationMode",1)
        cmd.setAttr(sweepMeshName+".scaleProfileX",
                    cmd.floatSliderGrp(tool.radius_sl, q=True, v=True))
        cmd.setAttr(sweepMeshName+".profilePolySides",
                    cmd.intSliderGrp(tool.collumn_subdivisions_sl, q=True, v=True))
        cmd.setAttr(sweepMeshName+".interpolationSteps",
                    cmd.intSliderGrp(tool.row_subdivisions_sl, q=True, v=True))
        cmd.setAttr(sweepMeshName+".rotateProfile",
                    cmd.floatSliderGrp(tool.rotation_sl, q=True, v=True))
        cmd.setAttr(sweepMeshName+".twist",
                    cmd.floatSliderGrp(tool.twist_sl, q=True, v=True))
        cmd.setAttr(sweepMeshName+".taper",
                    cmd.floatSliderGrp(tool.taperAm_sl, q=True, v=True))

    def parent_Rings(self):
        cmds.parent(self.mesh,self.locator)

class RopeTool(object):
    def __init__(self,width=400):
        self.window = "Rope_Tool"
        self.title = "Rope Tool"
        self.size = (width/2,400)
        self.width = width
        self.ropeLs = []
        self.systemsLs = []
        self.indexRope=0
        self.packed_systems = []
        self.selectedSystem = None
        self.selectedRope = None
        self.active = False

        #Internal Variables
        self.radius_sl = 0.1
        self.collumn_subdivisions_sl = 8
        self.row_subdivisions_sl = 30
        self.rotation_sl = 0
        self.twist_sl = 0
        self.twistRear_sl = 0
        self.taperAm_sl = 0
        self.debugtext = None
        self.createUI()

    #Function
    def deleteSystem(self,system_2_delete):
        delete(system2Delete)

    def createSystem(self, ignore):
        selection = cmd.ls(sl=True,o=True)
        if len(selection) == 1:
            print("objectFound")
            self.selectedSystem = RopeSystem(str(selection[0]),selection,self)
            setSelection(self.selectedSystem)
            cmds.textScrollList(self.systemsList, e=True, append=self.selectedSystem.name)

            self.packed_systems.append(self.selectedSystem)
            self.update_systems()

            # cmd.group(em=True, name= str(selectedSystem.name))
            # return new_system
        else:
            print("Object not found")

    def get_selected_system(self):
        currentName = cmds.textScrollList(self.systemsList, q=1, si=1)
        for system in self.packed_systems:
            if system.name == currentName[0]:
                self.selectedSystem = system
                print("changed selection to" + str(self.selectedSystem.name))
                if len(system.ropes)>0:
                    self.selectedRope = system.ropes[0]
                    active=True
                else:
                    active=False

        self.update_rope_ui()
        self.get_slider_settings(active)

    def set_selected_rope(self):
        # get current selected item in list
        current_rope = cmds.textScrollList(self.ropeList, q=1, si=1)

        #check if rope name is in ropes
        for rope in self.selectedSystem.ropes:
            if rope.name == current_rope[0]:
                self.selectedRope = rope
                print("Selected Ring: " + str(self.selectedRope.name))
                self.get_slider_settings(True)
                cmd.select(self.selectedRope.locator)

    def update_rope_ui(self):
        #clear list
        cmd.textScrollList(self.ropeList, e=True, removeAll=True)
        if self.selectedSystem != None:
            if len(self.selectedSystem.ropes)>0:
                #get ropes of object selected in list
                for rope in self.selectedSystem.ropes:
                    cmds.textScrollList(self.ropeList, e=True, append= rope.name)

    def update_systems(self):
        # Clear all items in list.
        cmds.textScrollList(self.systemsList, e=True, removeAll=True)

        for system in self.packed_systems:
            cmds.textScrollList(self.systemsList, e=True, append = system.name)

    def create_cable(self,ignore):
        self.selectedRope = self.selectedSystem.create_rope()
        self.active=True
        self.get_slider_settings(True)
        self.set_slider_settings(None)
        self.update_rope_ui()

    def delete_system(self,ignore):
            # lookup system index and set to be deleted system
            index_at_list = self.packed_systems.index(self.selectedSystem)
            todelete = self.selectedSystem

            # select system above the one to be deleted
            if len(self.packed_systems) > 1:
                self.selectedSystem = self.packed_systems[index_at_list-1]
                print("Selected System: " + str(self.selectedSystem.name))
            else:
                self.selectedSystem = None

            # remove system and update scrollList
            self.packed_systems.pop(index_at_list)
            self.update_systems()
            self.update_rope_ui()

    def delete_rope(self,ignore):
        if len(self.selectedSystem.ropes)>1:
            # lookup ring index and set to be deleted ring
            index_at_list = self.selectedSystem.ropes.index(self.selectedRope)
            todelete = self.selectedRope
            # select ring above the one to be deleted
            self.selectedRope = self.selectedSystem.ropes[index_at_list-1]
            print("Selected Ring: " + str(self.selectedRope.name))
            # remove ring and update scrollList
            self.selectedSystem.ropes.pop(index_at_list)
            self.update_rope_ui()
            cmd.delete(todelete.locator)
        else:
            print("Can't delete Last ring")

    def convert_system(self,ignore):
        cmd.delete(self.selectedSystem.convexObject)
        for rope in self.selectedSystem.ropes:
            cmd.delete(rope.locator)

        self.delete_system(None)

    def eFunc(self,ignore):
        print("ddwweez")

    def set_slider_settings(self,ignore):
        if self.selectedRope != None:
            print("change var of " + self.selectedRope.name)
            self.selectedRope.set_sweepattributes(self)

    def get_slider_settings(self,active,):
        #Get ring values to apply on refreshed or initialized sliders
        if self.active==True:
            ring = self.selectedRope
            cmd.floatSliderGrp(self.radius_sl, e=True, v=ring.radius,enable=active,max=ring.bounds_size)
            cmd.intSliderGrp(self.collumn_subdivisions_sl, e=True, v=ring.collumn_subdivisions,enable=active)
            cmd.intSliderGrp(self.row_subdivisions_sl, e=True, v=ring.row_subdivisions,enable=active)
            cmd.floatSliderGrp(self.rotation_sl, e=True, v=ring.rotation,enable=active)
            cmd.floatSliderGrp(self.twist_sl, e=True, v=ring.twist,enable=active)
            cmd.floatSliderGrp(self.twistRear_sl, e=True, v=ring.twistRear,enable=active)
            cmd.floatSliderGrp(self.taperAm_sl, e=True, v=ring.taperAm ,enable=active)

    #internal func
    def xLayout(self,start=True):
        if start:
            cmd.rowLayout(numberOfColumns=2, columnWidth2=(self.width / 2, self.width / 2))
        else:
            cmd.setParent('..')

    def RingSettings(self):
        self._layout = cmd.formLayout(p=self.window)
        tab = cmd.tabLayout(p=self._layout)
        ring = self.selectedRope
        cmd.formLayout(self._layout,e=True, af=[
            (tab,"top",0),
            (tab,"right",0)
            ])

        tab01 = cmd.columnLayout(p=tab)

        cmd.text(label="Rope Radius", align="center",w=200)
        self.radius_sl = cmd.floatSliderGrp(min=0.01, max=10, value=0, step=0.01,field=True,dc=self.set_slider_settings,enable=False)

        cmd.text(label="collumn_subdivisions", align="center", w=self.width)
        self.collumn_subdivisions_sl = cmd.intSliderGrp(min=0, max=128, value=0, step=1,field=True,dc=self.set_slider_settings,enable=False)

        cmd.text(label="row_subdivisions", align="center", w=self.width)
        self.row_subdivisions_sl = cmd.intSliderGrp(min=0, max=200, value=0, step=1,field=True,dc=self.set_slider_settings,enable=False)

        tab02 = cmd.columnLayout(p=tab)

        cmd.text(label="rotation", align="center", w=self.width)
        self.rotation_sl = cmd.floatSliderGrp(min=-360, max=360, value=0, step=0.1,field=True,dc=self.set_slider_settings,enable=False)

        cmd.text(label="Twist", align="center", w=self.width)
        self.twist_sl = cmd.floatSliderGrp(min=-2, max=2, value=0, step=0.1,field=True,dc=self.set_slider_settings,enable=False)

        cmd.text(label="twistRear", align="center", w=self.width)
        self.twistRear_sl = cmd.floatSliderGrp(min=0.01, max=1000, value=0, step=0.1,field=True,dc=self.set_slider_settings,enable=False)

        cmd.text(label="Taper", align="center", w=self.width)
        self.taperAm_sl = cmd.floatSliderGrp(min=0, max=5, value=1, step=0.1,field=True,dc=self.set_slider_settings,enable=False)

        cmd.tabLayout(tab, edit=True, tabLabel=((tab01, 'General'),
                                                (tab02, 'Modifiers')
                                                ))

    def combineRopes(self,ignore):
       for rope in self.selectedSystem.ropes:
           print(rope.name)
           selection = cmd.select(rope.name)
           vertexAmount = cmd.polyEvaluate(v=True)
           cmd.detachCurve(rope)
           pointPos = cmd.pointPosition(rope.name + ".cv" + str(vertexAmount - 1), w=True)

    def createUI(self):
        #close other window if open
        if cmd.window(self.window, exists=True,rtf=True):
            cmd.deleteUI(self.window, window=True)

        self.window = cmd.window(self.window, title=self.title, widthHeight=self.size, rtf=True)
        self.c_layout = cmd.columnLayout(columnAttach=('both', 8), columnWidth=self.width,bgc=(0.2, 0.2, 0.2))

        cmd.text(label=self.title, p=self.c_layout)
        cmd.separator(p=self.c_layout, style='in', h=50, w=self.width)

        self.debugtext = cmd.text(label="Select an Object to Start", al="center")
        cmd.separator(p=self.c_layout, style='in', h=50, w=self.width)

        self.xLayout()
        #Create System If object is selected is valid

        cmd.button(label="Create System",w=self.width / 2, h=50, command = self.createSystem)
        cmd.button(label="Convert System", w=self.width / 2, h=50, command = self.eFunc)

        self.xLayout(False)

        self.xLayout()
        cmd.button(label="Add Rope", w=self.width / 2, h=50, command= self.create_cable)
        cmd.button(label="Delete Rope", w=self.width / 2, h=50, command= self.eFunc)
        self.xLayout(False)

        self.xLayout()
        cmd.text(label="Systems", align="center", w=self.width / 2)
        cmd.text(label="Ropes", align="center", w=self.width / 2)
        self.xLayout(False)

        self.xLayout()
        # scrolllists for Systems and containing rings
        self.systemsList = cmds.textScrollList(numberOfRows=8, allowMultiSelection=True,
                                               append=[],
                                               selectItem='SystemTestOne', showIndexedItem=4,dcc=self.get_selected_system)
        self.ropeList = cmds.textScrollList(numberOfRows=8, allowMultiSelection=True,
                                            append=[],
                                            selectItem='Rope-One', showIndexedItem=4,dcc=self.set_selected_rope)
        self.xLayout(False)

        self.xLayout()
        cmd.button(label="Delete Rope", w=self.width / 2, h=15, command=self.delete_rope)
        cmd.button(label="Combined Ropes", w=self.width / 2, h=15, command=self.combineRopes)
        self.xLayout(False)

        self.xLayout()
        cmd.button(label="Delete System", w=self.width / 2, h=15, command = self.delete_system)
        cmd.button(label="Delete Rope", w=self.width / 2, h=15, command = self.delete_rope)
        self.xLayout(False)


        cmd.button(label="Convert System",p=self.c_layout, w=self.width / 2, h=50, command=self.convert_system)

        # Show indivual sliders
        self.RingSettings()

        cmd.showWindow(self.window)


def setSelection(self):
    self.selectedObject = cmd.ls(sl=True,long=True)
    print("Selection Set to: "+ self.name )

def getVertexPosition(object):
    vertexPositions = []
    vertexAmount = cmd.polyEvaluate(object, v=True)
    for i in range(vertexAmount):
        ptPos = cmd.xform(str(object[0])+".vtx["+str(i)+"]",q=True,t=True,ws=True)
        vertexPositions.append(ptPos)
    return vertexPositions

def createConvexHull(vertexPositions,object):
    chull = spatial.ConvexHull(vertexPositions)
    indices = chull.simplices
    vertices = [om.MPoint(tp[0], tp[1], tp[2]) for tp in chull.points]
    poligon_counts = [len(x) for x in indices]
    poly_connect = [x for y in chull.simplices for x in y]

    transform_name = cmd.createNode('transform',name=object[0]+"_Convex")
    transform_mobj = om.MGlobal.getSelectionListByName(transform_name).getDependNode(0)
    mesh_fn = om.MFnMesh()
    mesh_fn.create(vertices, poligon_counts, poly_connect, parent=transform_mobj)

    # Set Center of object
    cmd.sets(transform_name, e=True, forceElement='initialShadingGroup')
    cmd.xform(transform_name, cp=1)

    # Clean up mesh and set normal
    cmd.polyClean(ce=True,cv=True)
    cmd.polySetToFaceNormal()
    cmd.polyNormal( nm=2 )
    cmd.polyNormal( nm=3)
    cmd.polySoftEdge(a=180)
    return transform_name

def createLocator(convexTransform):
    centerPosition = cmd.objectCenter(convexTransform, gl=True)
    cmd.hide(convexTransform)
    ropelocator = cmd.spaceLocator(p=centerPosition, n='RopeHandle',a=True)
    cmd.xform(ropelocator, cp=1)
    return ropelocator

def convertObjectToFN(surfaceTransform:object):
    om.MGlobal.selectByName(surfaceTransform)
    selection = om.MSelectionList()
    item = om.MDagPath()
    selection.getDagPath(item)
    item.extendToShape()
    fnMesh = om.MFnMesh(item)
    return fnMesh

def createProfile(ring):
    # spawns the cable profile
    #Singular Cable
    cablegeometry = cmd.polyCylinder(sh=20, sa=50,radius=0.05)
    cb = cmd.rename(cablegeometry[0], ring.mesh[0] + "Geometry")
    cmd.parent(ring.mesh[0] + 'Geometry', ring.mesh[0].strip("|"))
    # additionalDeformers(True,cb) #not handy at all for what I need
    return cb

def additionalDeformers(_,profile):
    taperM = cmd.nonLinear(profile, typ="flare")
    cmd.rename(taperM[0], profile + "taperM")
    cmd.parent(taperM[1], profile)
    # cmd.hide(taperM[1])


ropeTool = RopeTool()



