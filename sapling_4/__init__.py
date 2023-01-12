# -*- coding: utf-8 -*-

#====================== BEGIN GPL LICENSE BLOCK ======================
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
#======================= END GPL LICENSE BLOCK ========================

bl_info = {
    "name": "Sapling_4",
    "author": "Andrew Hale (TrumanBlending), modified by Aaron Buchler 2015-2020",
    "version": (0, 4, 0),
    "blender": (2, 80, 0),
    "location": "View3D > Add > Curve",
    "description": "Adds a parametric tree.",
    "category": "Add Curve"}
##"Originally based on the method presented by Jason Weber & Joseph Penn in their paper 'Creation and Rendering of Realistic Trees'."##

if "bpy" in locals():
    import importlib
    importlib.reload(utils)
else:
    from sapling_4 import utils

import bpy
import time
import os
import ast

#import cProfile

from mathutils import *
from math import pi, sin, degrees, radians, atan2, copysign, ceil
from random import random, uniform, seed, choice, getstate, setstate
from bpy.props import *

from sapling_4.utils import *

useSet = False
is_first = False

shapeList = [('0', 'Conical (0)', 'Shape = 0'),
            ('1', 'Spherical (1)', 'Shape = 1'),
            ('2', 'Hemispherical (2)', 'Shape = 2'),
            ('3', 'Cylindrical (3)', 'Shape = 3'),
            ('4', 'Tapered Cylindrical (4)', 'Shape = 4'),
            ('5', 'Flame (5)', 'Shape = 5'),
            ('6', 'Inverse Conical (6)', 'Shape = 6'),
            ('7', 'Tend Flame (7)', 'Shape = 7')]

shapeList3 = [('0', 'Conical', ''),
            ('6', 'Inverse Conical', ''),
            ('1', 'Spherical', ''),#
            ('2', 'Hemispherical', ''),#
            ('3', 'Cylindrical', ''),
            ('4', 'Tapered Cylindrical', ''),
            ('10', 'Inverse Tapered Cylindrical', ''),
            ('5', 'Flame', ''),#
            ('7', 'Tend Flame', ''),
            ('8', 'Custom Shape', '')]

shapeList4 = [('0', 'Conical', ''),
            ('6', 'Inverse Conical', ''),
            ('1', 'Spherical', ''),
            ('2', 'Hemispherical', ''),
            ('3', 'Cylindrical', ''),
            ('4', 'Tapered Cylindrical', ''),
            ('10', 'Inverse Tapered Cylindrical', ''),
            ('5', 'Flame', ''),
            ('7', 'Tend Flame', '')]

leaftypes = [('0', 'Rotated Alternate', 'leaves rotate around the stem and face upwards'),
             ('1', 'Rotated Opposite', 'pairs of leaves rotate around the stem and face upwards'),
             ('2', 'Alternate', 'leaves sprout alternately from each side of the stem, uses rotate angle'),
             ('3', 'Opposite', 'pairs of leaves sprout from opposite sides of stem, uses rotate angle'),
             ('4', 'Palmately Compound', 'multiple leaves radiating from stem tip, uses rotate angle for spread angle'),
             ('5', 'Radial', 'leaves rotate around the stem (for needles)')]

axes = [(("+0"), "X", ""),
        (("+1"), "Y", ""),
        (("+2"), "Z", ""),
        (("-0"), "-X", ""),
        (("-1"), "-Y", ""),
        (("-2"), "-Z", "")]


handleList = [('0', 'Auto', 'Auto'),
                ('1', 'Vector', 'Vector')]

settings = [('0', 'Geometry', 'Geometry'),
            ('1', 'Branch Radius', 'Branch Radius'),
            ('2', 'Branch Splitting', 'Branch Splitting'),
            ('3', 'Branch Growth', 'Branch Growth'),
            ('5', 'Leaves', 'Leaves'),
            ('6', 'Armature', 'Armature')]

branchmodes = [("original", "Original", "rotate around each branch"),
              ("rotate", "Rotate", "evenly distribute  branches to point outward from center of tree"),
              ("distance", "Distance", "remove overlapping branches")]

attachmenttypes = [('0','Alternate', ''), ('1','Opposite', '')]


def getPresetpaths():
    """Return paths for both local and user preset folders"""
    userDir = os.path.join(bpy.utils.script_path_user(), 'presets', 'operator', 'sapling_4')
    
    if os.path.isdir(userDir):
        pass
    else:
        os.makedirs(userDir)
    
    script_file = os.path.realpath(__file__)
    directory = os.path.dirname(script_file)
    localDir = os.path.join(directory, "presets")
    
    return (localDir, userDir)


class ExportData(bpy.types.Operator):
    """This operator handles writing presets to file"""
    bl_idname = 'sapling.exportdata'
    bl_label = 'Export Preset'

    data: StringProperty()

    def execute(self, context):
        # Unpack some data from the input
        data, filename, overwrite = eval(self.data)

        fpath1 = os.path.join(getPresetpaths()[0], filename + '.py')
        fpath2 = os.path.join(getPresetpaths()[1], filename + '.py')
        
        if os.path.exists(fpath1):
            # If it exists in built-in presets then report an error
            self.report({'ERROR_INVALID_INPUT'}, 'Can\'t have same name as built-in preset')
            return {'CANCELLED'}
        elif (not os.path.exists(fpath2)) or (os.path.exists(fpath2) and overwrite):
            #if (it does not exist) or (exists and overwrite) then write file
            if data:
                # If it doesn't exist, create the file with the required data
                f = open(os.path.join(getPresetpaths()[1], filename + '.py'), 'w')
                f.write(data)
                f.close()
                return {'FINISHED'}
            else:
                return {'CANCELLED'}
        else:
            # If it exists then report an error
            self.report({'ERROR_INVALID_INPUT'}, 'Preset Already Exists')
            return {'CANCELLED'}
            

class ImportData(bpy.types.Operator):
    """This operator handles importing existing presets"""
    bl_idname = 'sapling.importdata'
    bl_label = 'Import Preset'

    filename: StringProperty()

    def execute(self, context):
        # Make sure the operator knows about the global variables
        global settings, useSet, is_first
        # Read the preset data into the global settings
        try:
            f = open(os.path.join(getPresetpaths()[0], self.filename), 'r')
        except (FileNotFoundError, IOError):
            f = open(os.path.join(getPresetpaths()[1], self.filename), 'r')
        settings = f.readline()
        f.close()
        #print(settings)
        settings = ast.literal_eval(settings)
        
        #use old attractup
        if type(settings['attractUp']) == float:
            atr = settings['attractUp']
            settings['attractUp'] = [0, 0, atr, atr]
        
        #use old leaf rotations
        if 'leafDownAngle' not in settings:
            l = settings['levels']
            settings['leafDownAngle'] = settings['downAngle'][min(l, 3)]
            settings['leafDownAngleV'] = settings['downAngleV'][min(l, 3)]
            settings['leafRotate'] = settings['rotate'][min(l, 3)]
            settings['leafRotateV'] = settings['rotateV'][min(l, 3)]
        
        #use old leaf settings
        if 'leafType' not in settings:
            settings['leafType'] = '0'
        if settings['leaves'] < 0:
            settings['leaves'] = abs(settings['leaves'])
            settings['leafType'] = '4'
        if settings['leafRotate'] < 1:
            settings['leafType'] = '2'
        
        if 'noTip' not in settings:
            settings['noTip'] = False
        
        #zero curveback
        if 'curveBack' in settings:
            settings['curveBack'] = [0, 0, 0, 0]
            
        # Set the flag to use the settings
        useSet = True
        is_first = True
        return {'FINISHED'}


def presetAsDict(filename):
    try:
        f = open(os.path.join(getPresetpaths()[0], filename), 'r')
    except (FileNotFoundError, IOError):
        f = open(os.path.join(getPresetpaths()[1], filename), 'r')
    settings = f.readline()
    f.close()
    settings = ast.literal_eval(settings)
    return settings


class PresetMenu(bpy.types.Menu):
    """Create the preset menu by finding all preset files
    in the preset directory"""
    bl_idname = "SAPLING_MT_preset" #was sapling.presetmenu
    bl_label = "Presets"

    def draw(self, context):
        # Get all the sapling presets
        presets = [a for a in os.listdir(getPresetpaths()[0]) if a[-3:] == '.py']
        presetsUser = [a for a in os.listdir(getPresetpaths()[1]) if a[-3:] == '.py']
        layout = self.layout
        # Append all to the menu
        if "PreviousSettings.py" in presetsUser:
            p = "PreviousSettings.py"
            layout.operator("sapling.importdata", text=p[:-3]).filename = p
            presetsUser.remove("PreviousSettings.py")
        layout.separator()
        for p in presets:
            layout.operator("sapling.importdata", text=p[:-3]).filename = p
        layout.separator()
        for p in presetsUser:
            layout.operator("sapling.importdata", text=p[:-3]).filename = p


class AddTree(bpy.types.Operator):
    bl_idname = "curve.tree_add"
    bl_label = "Sapling: Add Tree"
    bl_options = {'REGISTER', 'UNDO'}
    
    def objectList(self, context):
        objects = []
        bObjects = bpy.data.objects
        for obj in bObjects:
            if (obj.type in ['MESH', 'CURVE', 'SURFACE']) and (obj.name not in ['tree', 'leaves']):
                objects.append((obj.name, obj.name, ""))
        
        return objects

    def update_tree(self, context):
        self.do_update = True
        
    def update_leaves(self, context):
        if self.showLeaves:
            self.do_update = True
        else:
            self.do_update = False
    
    def no_update_tree(self, context):
        self.do_update = False

    do_update: BoolProperty(name='Do Update',
        default=True, options={'HIDDEN'})

    chooseSet: EnumProperty(name='Settings',
        description='Choose the settings to modify',
        items=settings,
        default='0', update=no_update_tree)
    
    bevel: BoolProperty(name='Bevel',
        description='Whether the curve is beveled',
        default=False, update=update_tree)
    showLeaves: BoolProperty(name='Show Leaves',
        description='Whether the leaves are shown',
        default=False, update=update_tree)
    useArm: BoolProperty(name='Use Armature',
        description='Whether the armature is generated',
        default=False, update=update_tree)
    seed: IntProperty(name='Random Seed',
        description='The seed of the random number generator',
        default=0, update=update_tree)
    handleType: EnumProperty(name='Handle Type',
        description='The type of bezier curve handles',
        items=handleList,
        default='0', update=update_tree)
    bevelRes: IntProperty(name='Bevel Resolution',
        description='The bevel resolution of the curves',
        min=0,
        max=32,
        default=0, update=update_tree)
    resU: IntProperty(name='Curve Resolution',
        description='The resolution along the curves',
        min=1,
        default=4, update=update_tree)
    
    levels: IntProperty(name='Levels',
        description='Number of recursive branches',
        min=1,
        max=6,
        soft_max=4,
        default=3, update=update_tree)
    length: FloatVectorProperty(name='Length',
        description='The relative lengths of each branch level',
        min=0.000001,
        default=[1, 0.3, 0.6, 0.45],
        size=4, update=update_tree)
    lengthV: FloatVectorProperty(name='Length Variation',
        description='Branch and split length variations for each level',
        min=0.0,
        max=1.0,
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    taperCrown: FloatProperty(name='Taper Crown',
        description='Shorten trunk splits toward outside of tree',
        min=0.0,
        soft_max=1.0,
        default=0, update=update_tree)
    branches: IntVectorProperty(name='Branches',
        description='The number of branches grown at each level',
        min=0,
        default=[50, 30, 10, 10],
        size=4, update=update_tree)
    curveRes: IntVectorProperty(name='Curve Resolution',
        description='The number of segments on each branch',
        min=1,
        soft_max=16,
        default=[8, 5, 3, 1],
        size=4, update=update_tree)
    curve: FloatVectorProperty(name='Curvature',
        description='The angle of the end of the branch',
        default=[0, -40, -40, 0],
        size=4, update=update_tree)
    curveV: FloatVectorProperty(name='Curvature Variation',
        description='Variation of the curvature',
        default=[20, 50, 75, 0],
        size=4, update=update_tree)
    curveBack: FloatVectorProperty(name='Back Curvature',
        description='Curvature for the second half of a branch',
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    baseSplits: IntProperty(name='Base Splits',
        description='Number of trunk splits at its base',
        min=0,
        default=0, update=update_tree)
    segSplits: FloatVectorProperty(name='Segment Splits',
        description='Number of splits per segment',
        min=0,
        soft_max=1,
        max=3,
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    splitByLen: BoolProperty(name='Split relative to length',
        description='Split proportional to branch length',
        default=False, update=update_tree)
    rMode: EnumProperty(name="", #"Branching Mode"
        description='Branching and Rotation Mode',
        items=branchmodes,
        default="rotate", update=update_tree)
    splitStraight: FloatProperty(name="Split Straight",
        description="Straightness of trunk branch splits",
        min=0.0,
        max=1.0,
        default=0, update=update_tree)
    splitLength: FloatProperty(name="Split Length",
        description="Length of trunk branch splits, (similar to length variation but not random)",
        min=0.0,
        max=1.0,
        default=0, update=update_tree)
    splitAngle: FloatVectorProperty(name='Split Angle',
        description='Angle of branch splitting',
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    splitAngleV: FloatVectorProperty(name='Split Angle Variation',
        description='Variation in the split angle',
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    scale: FloatProperty(name='Scale',
        description='The tree scale',
        min=0.0,
        default=13.0, update=update_tree)
    scaleV: FloatProperty(name='Scale Variation',
        description='The variation in the tree scale',
        default=3.0, update=update_tree)
    attractUp: FloatVectorProperty(name='Vertical Attraction',
        description='Branch upward attraction',
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    attractOut: FloatVectorProperty(name='Outward Attraction',
        description='Branch outward attraction',
        default=[0, 0, 0, 0],
        min=0.0,
        max=1.0,
        size=4, update=update_tree)
    shape: EnumProperty(name='', #Shape
        description='The overall shape of the tree',
        items=shapeList3,
        default='7', update=update_tree)
    shapeS: EnumProperty(name='', #Secondary Branches Shape
        description='The shape of secondary splits',
        items=shapeList4,
        default='4', update=update_tree)
    customShape: FloatVectorProperty(name='',#Custom Shape
        description='Branch Length at \n(Base, Middle, Middle Position, Top)',
        size=4,
        min=.01,
        max=1,
        default=[.5, 1.0, .3, .5], update=update_tree)
    branchDist: FloatProperty(name='Branch Distribution',
        description='Adjust branch spacing to put more branches at the top or bottom of the tree',
        min=0.1,
        soft_max=10,
        default=1.0, update=update_tree)
    nrings: IntProperty(name='Whorls',
        description='number of whorls',
        min=0,
        default=0, update=update_tree)
    baseSize: FloatProperty(name='', # 'Trunk Height'
        description='Fraction of trunk with no branches (Base Size)',
        min=0.0,
        max=1.0,
        default=0.4, update=update_tree)
    baseSize_s: FloatProperty(name='', #Secondary Start Length
        description='Fraction of 2nd and higher levels with no branches',
        min=0.0,
        max=1.0,
        default=0.25, update=update_tree)
    leafBaseSize: FloatProperty(name='Leaf Start Length',
        description='Fraction of stem length with no leaves',
        min=0.0,
        max=1.0,
        default=0.20, update=update_tree)
    leafBaseLevel: IntProperty(name='Leaf Start Level',
        description='Level of branches on which leaves appear',
        min=0,
        max=4    ,
        default=1, update=update_tree)
    splitHeight: FloatProperty(name='Split Height',
        description='Fraction of tree height with no splits',
        min=0.0,
        max=1.0,
        default=0.2, update=update_tree)
    splitBias: FloatProperty(name='Split Bias',
        description='Put more splits at the top or bottom of the tree',
        soft_min=-2.0,
        soft_max=2.0,
        default=0.0, update=update_tree)
    ratio: FloatProperty(name='Ratio',
        description='Ratio of tree scale to base radius',
        min=0.0,
        default=0.015, update=update_tree)
    minRadius: FloatProperty(name='Minimum Radius',
        description='Minimum branch Radius',
        min=0.0,
        default=0.0, update=update_tree)
    closeTip: BoolProperty(name='Close Tip',
        description='Set radius at branch tips to zero',
        default=False, update=update_tree)
    rootFlare: FloatProperty(name='Root Flare',
        description='Root radius factor',
        min=1.0,
        default=1.0, update=update_tree)
    splitRadiusRatio: FloatProperty(name='Split Radius Ratio',
        description='Reduce radius after branch splits, (0 is auto)',
        min=0.0,
        max=1.0,
        default=0.75, update=update_tree)
    autoTaper: BoolProperty(name='Auto Taper',
        description='Calculate taper automaticly based on branch lengths',
        default=True, update=update_tree)
    taper: FloatVectorProperty(name='Taper',
        description='The fraction of tapering on each branch',
        min=0.0,
        max=1.0,
        default=[1, 1, 1, 1],
        size=4, update=update_tree)
    noTip: BoolProperty(name='No branch at stem tip',
        description='Useful for non-typical / abstract trees',
        default=False, update=update_tree)
    radiusTweak: FloatVectorProperty(name='Tweak Radius',
        description='multiply radius by this factor',
        min=0.0,
        max=1.0,
        default=[1, 1, 1, 1],
        size=4, update=update_tree)                 
    ratioPower: FloatProperty(name='Radius Ratio Power',
        description='Power which defines the radius of a branch compared to the radius of the branch it grew from',
        min=0.0,
        soft_min=1.0,
        default=1.2, update=update_tree)
    downAngle: FloatVectorProperty(name='Down Angle',
        description='The angle between a new branch and the one it grew from',
        default=[90, 60, 45, 45],
        size=4, update=update_tree)
    downAngleV: FloatVectorProperty(name='Down Angle Variation',
        description='Angle to decrease Down Angle by towards end of parent branch \n(negative values add random variation)',
        default=[0, -50, 10, 10],
        size=4, update=update_tree)
    useOldDownAngle: BoolProperty(name='Use old down angle variation',
        default=False, update=update_tree)
    useParentAngle: BoolProperty(name='Use parent angle',
        description='(first level) Rotate branch to match parent branch',
        default=True, update=update_tree)
    rotate: FloatVectorProperty(name='Rotate Angle',
        description='The angle of a new branch around the one it grew from \n(negative values make branches rotate opposite from the previous one)',
        default=[137.5, 137.5, 137.5, 137.5],
        size=4, update=update_tree)
    rotateV: FloatVectorProperty(name='Rotate Angle Variation',
        description='Variation in the rotate angle',
        default=[0, 0, 0, 0],
        size=4, update=update_tree)
    scale0: FloatProperty(name='Radius Scale',
        description='The scale of the trunk radius',
        min=0.0,
        default=1.0, update=update_tree)
    scaleV0: FloatProperty(name='Radius Scale Variation',
        description='Variation in the radius scale',
        min=0.0,
        max=1.0,
        default=0.2, update=update_tree)
    attachment: EnumProperty(name='Attachment',
        description='Type of branch arrangment',
        items=attachmenttypes,
        default='0', update=update_tree)
    leaves: IntProperty(name='Leaves',
        description='Maximum number of leaves per branch',
        min=0,
        default=25, update=update_tree)
    leafType: EnumProperty(name='Leaf Type',
        description='Type of leaf arrangment',
        items=leaftypes,
        default='0', update=update_leaves) #update_leaves update_tree
    
    leafDownAngle: FloatProperty(name='Leaf Down Angle',
        description='The angle between a new leaf and the branch it grew from',
        default=45, update=update_leaves)
    leafDownAngleV: FloatProperty(name='Leaf Down Angle Variation',
        description='Angle to decrease Down Angle by towards end of parent branch',
        min=0,
        default=10, update=update_tree)
    leafRotate: FloatProperty(name='Leaf Rotate Angle',
        description='The rotate angle for Alternate and Opposite leaves',
        default=90, update=update_tree)
    leafRotateV: FloatProperty(name='Rotation Variation',
        description='Add randomness to leaf orientation',
        default=0.0, update=update_leaves)
    
    leafObjZ: EnumProperty(name='',
        description='leaf tip axis',
        items=axes,
        default="+2", update=update_leaves)
    leafObjY: EnumProperty(name='',
        description='leaf top axis',
        items=axes,
        default="+1", update=update_leaves)
    #leafObjX: EnumProperty(name='',
    #    description='leaf side axis',
    #    items=axes,
    #    default="+0", update=update_leaves)
    
    leafScale: FloatProperty(name='Leaf Scale',
        description='The scaling applied to the whole leaf',
        min=0.0,
        default=0.17, update=update_leaves)
    leafScaleX: FloatProperty(name='Leaf Scale X',
        description='The scaling applied to the x direction of the leaf',
        min=0.0,
        default=1.0, update=update_leaves)
    leafScaleT: FloatProperty(name='Leaf Scale Taper',
        description='scale leaves toward the tip or base of the patent branch',
        min=-1.0,
        max=1.0,
        default=0.0, update=update_leaves)
    leafScaleV: FloatProperty(name='Leaf Scale Variation',
        description='randomize leaf scale',
        min=0.0,
        max=1.0,
        default=0.0, update=update_leaves)
    leafShape: EnumProperty(name='Leaf Shape',
        description='The shape of the leaves',
        items=(('hex', 'Hexagonal', '0'), ('rect', 'Rectangular', '1'), ('dFace', 'DupliFaces', '2'), ('dVert', 'DupliVerts', '3')),
        default='hex', update=update_leaves)
    leafDupliObj: EnumProperty(name='Leaf Object',
        description='Object to use for leaf instancing if Leaf Shape is DupliFaces or DupliVerts',
        items=objectList,
        update=update_leaves)

    leafangle: FloatProperty(name='Leaf Angle',
        description='Leaf vertical attraction',
        default=0.0, update=update_leaves)
    leafDist: EnumProperty(name='Leaf Distribution',
        description='The way leaves are distributed on branches',
        items=shapeList4,
        default='6', update=update_tree)
    
    armAnim: BoolProperty(name='Armature Animation',
        description='Whether animation is added to the armature',
        default=False, update=update_tree)
    previewArm: BoolProperty(name='Fast Preview',
        description=('Disable armature modifier, hide tree, and set bone display to wire, for fast playback \n'
                     'If Make Mesh is enabled: \nDisable skin modifier, hide curve tree and armature'),
        default=False, update=update_tree)
    leafAnim: BoolProperty(name='Leaf Animation',
        description='Whether animation is added to the leaves',
        default=False, update=update_tree)
    frameRate: FloatProperty(name='Animation Speed',
        description='Adjust speed of animation, relative to scene frame rate',
        min=0.001,
        default=1, update=update_tree)
    loopFrames: IntProperty(name='Loop Frames',
        description='Number of frames to make the animation loop for, zero is disabled',
        min=0,
        default=0, update=update_tree)
    
#    windSpeed: FloatProperty(name='Wind Speed',
#        description='The wind speed to apply to the armature',
#        default=2.0, update=update_tree)
#    windGust: FloatProperty(name='Wind Gust',
#        description='The greatest increase over Wind Speed',
#        default=0.0, update=update_tree)
    
    wind: FloatProperty(name='Overall Wind Strength',
        description='The intensity of the wind to apply to the armature',
        default=1.0, update=update_tree)

    gust: FloatProperty(name='Wind Gust Strength',
        description='The amount of directional movement, (from the positive Y direction)',
        default=1.0, update=update_tree)
    
    gustF: FloatProperty(name='Wind Gust Fequency',
        description='The Fequency of directional movement',
        default=0.075, update=update_tree)

    af1: FloatProperty(name='Amplitude',
        description='Multiplier for noise amplitude',
        default=1.0, update=update_tree)
    af2: FloatProperty(name='Frequency',
        description='Multiplier for noise fequency',
        default=1.0, update=update_tree)
    af3: FloatProperty(name='Randomness',
        description='Random offset in noise',
        default=4.0, update=update_tree)
    
    makeMesh: BoolProperty(name='Make Mesh',
        description='Convert curves to mesh, uses skin modifier, enables armature simplification',
        default=False, update=update_tree)
    armLevels: IntProperty(name='Armature Levels',
        description='Number of branching levels to make bones for, 0 is all levels',
        min=0,
        default=2, update=update_tree)
    boneStep: IntVectorProperty(name='Bone Length',
        description='Number of stem segments per bone',
        min=1,
        default=[1, 1, 1, 1],
        size=4, update=update_tree)
    matIndex: IntVectorProperty(name='Material Index',
        description='Material index for each split level (curves only)',
        min=0,
        max=3,
        default=[0, 0, 0, 0],
        size=4, update=update_tree)

    presetName: StringProperty(name='Preset Name',
        description='The name of the preset to be saved',
        default='',
        subtype='FILE_NAME', update=no_update_tree)
    limitImport: BoolProperty(name='Limit Import',
        description='Limited imported tree to 2 levels & no leaves for speed',
        default=True, update=no_update_tree)
    overwrite: BoolProperty(name='Overwrite',
        description='When checked, overwrite existing preset files when saving',
        default=False, update=no_update_tree)

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    def draw(self, context):

        layout = self.layout

        layout.prop(self, 'chooseSet')

        if self.chooseSet == '0':
            box = layout.box()
            box.label(text="Geometry:")
            row = box.row()
            row.prop(self, 'bevel')
            row.prop(self, 'makeMesh')
            
            row = box.row()
            row.prop(self, 'bevelRes')
            row.prop(self, 'resU')
            
            box.prop(self, 'handleType')
            row = box.row()
            row.prop(self, 'matIndex')
            
            #box.prop(self, 'shape')
            #row = box.row()
            #row.prop(self, 'customShape')
            #box.prop(self, 'shapeS')
            
            #box.label(text="")
            #box.prop(self, 'branchDist')
            #box.prop(self, 'nrings')
            
            box.label(text="")
            box.prop(self, 'seed')
            
            box.label(text="Tree Scale:")
            row = box.row()
            row.prop(self, 'scale')
            row.prop(self, 'scaleV')

            # Here we create a dict of all the properties.
            # Unfortunately as_keyword doesn't work with vector properties,
            # so we need something custom. This is it
            data = []
            for a, b in (self.as_keywords(ignore=("chooseSet", "presetName", "limitImport", "do_update", "overwrite", "leafDupliObj"))).items():
                # If the property is a vector property then add the slice to the list
                try:
                    len(b)
                    data.append((a, b[:]))
                # Otherwise, it is fine so just add it
                except:
                    data.append((a, b))
            # Create the dict from the list
            data = dict(data)

            row = box.row()
            row.prop(self, 'presetName')
            # Send the data dict and the file name to the exporter
            row.operator('sapling.exportdata').data = repr([repr(data), self.presetName, self.overwrite])
            row = box.row()
            row.label(text=" ")
            row.prop(self, 'overwrite')
            row = box.row()
            row.menu('SAPLING_MT_preset', text='Load Preset')
            row.prop(self, 'limitImport')
        
        elif self.chooseSet == '1':
            box = layout.box()
            box.label(text="Branch Radius:")
            
            row = box.row()
            row.prop(self, 'bevel')
            row.prop(self, 'bevelRes')
            
            box.prop(self, 'ratio')
            
            box.prop(self, 'minRadius')
            box.prop(self, 'closeTip')
            box.prop(self, 'rootFlare')
            box.prop(self, 'splitRadiusRatio')
            
            box.label(text="")
            box.label(text="Other:")
            
            row = box.row()
            row.prop(self, 'autoTaper')
            row.prop(self, 'noTip')
            
            row = box.row()
            row.prop(self, 'scale0')
            row.prop(self, 'scaleV0')
            box.prop(self, 'ratioPower')
            
            split = box.split()
            col = split.column()
            col.prop(self, 'taper')
            col = split.column()
            col.prop(self, 'radiusTweak')

        elif self.chooseSet == '2':
            box = layout.box()
            box.label(text="Branch Splitting:")
            box.prop(self, 'levels')
            box.prop(self, 'baseSplits')
            
            row = box.row()
            row.prop(self, 'splitHeight')
            row.prop(self, 'splitBias')
            
            row = box.row()
            split = row.split()
            col = split.column()
            col.label(text="Start Length:")
            col = split.column()
            col.prop(self, 'baseSize')
            col.prop(self, 'baseSize_s')
            
            #box.label(text="")
            box.prop(self, 'branchDist')
            box.prop(self, 'nrings')
            
            split = box.split()
            
            col = split.column()
            col.prop(self, 'branches')
            col.prop(self, 'splitAngle')
            col.prop(self, 'rotate')
            #col.prop(self, 'attractOut')
            
            col.label(text="Branch Attachment:")
            row = col.row()
            row.prop(self, 'attachment', expand=True)
            col.prop(self, 'splitByLen')
            #col.prop(self, 'taperCrown')
            
            col = split.column()
            col.prop(self, 'segSplits')
            col.prop(self, 'splitAngleV')
            col.prop(self, 'rotateV')
            
            col.label(text="Branching Mode:")
            col.prop(self, 'rMode')
            col.prop(self, 'splitStraight')
            col.prop(self, 'splitLength')
            

            box.column().prop(self, 'curveRes')

        elif self.chooseSet == '3':
            box = layout.box()
            box.label(text="Branch Growth:")
            
            #box.prop(self, 'taperCrown')
            
            row = box.row()
            split = row.split()
            col = split.column()
            #col.prop(self, 'length')
            col.label(text="Shape:")
            col = split.column()
            col.prop(self, 'shape')
            col.prop(self, 'shapeS')
            
            box.label(text="Custom Shape:")
            
            row = box.row()
            row.prop(self, 'customShape')
            
            split = box.split()
            
            col = split.column()
            col.prop(self, 'length')
            col.prop(self, 'downAngle')
            col.prop(self, 'curve')
            #col.prop(self, 'curveBack')
            col.prop(self, 'attractOut')
            
            col = split.column()
            
            col.prop(self, 'lengthV')
            col.prop(self, 'downAngleV')
            col.prop(self, 'curveV')
            col.prop(self, 'attractUp')
            
            #box.prop(self, 'useOldDownAngle')
            box.prop(self, 'useParentAngle')

        elif self.chooseSet == '5':
            box = layout.box()
            box.label(text="Leaves:")
            box.prop(self, 'showLeaves')
            box.prop(self, 'leafShape')
            box.prop(self, 'leafDupliObj')
            

            row = box.row()
            row.label(text="Leaf Object Axes:")
            row.prop(self, 'leafObjZ')
            row.prop(self, 'leafObjY')
            #row.prop(self, 'leafObjX')
            
            
            box.prop(self, 'leaves')
            box.prop(self, 'leafBaseSize')
            box.prop(self, 'leafDist')
            
            box.label(text="")
            box.prop(self, 'leafBaseLevel')
            box.prop(self, 'leafType')
            box.prop(self, 'leafangle')
            row = box.row()
            row.prop(self, 'leafDownAngle')
            row.prop(self, 'leafDownAngleV')
            
            row = box.row()
            row.prop(self, 'leafRotate')
            row.prop(self, 'leafRotateV')
            box.label(text="")
            
            row = box.row()
            row.prop(self, 'leafScale')
            row.prop(self, 'leafScaleX')
            
            row = box.row()
            row.prop(self, 'leafScaleT')
            row.prop(self, 'leafScaleV')

        elif self.chooseSet == '6':
            box = layout.box()
            box.label(text="Armature and Animation:")
            
            row = box.row()
            row.prop(self, 'useArm')
            row.prop(self, 'previewArm')
            
            row = box.row()
            row.prop(self, 'armAnim')
            row.prop(self, 'leafAnim')
            
            box.prop(self, 'frameRate')
            box.prop(self, 'loopFrames')
            
            #row = box.row()
            #row.prop(self, 'windSpeed')
            #row.prop(self, 'windGust')
            
            box.label(text='Wind Settings:')
            box.prop(self, 'wind')
            row = box.row()
            row.prop(self, 'gust')
            row.prop(self, 'gustF')
            
            box.label(text='Leaf Wind Settings:')
            box.prop(self, 'af1')
            box.prop(self, 'af2')
            box.prop(self, 'af3')
            
            box.label(text="")
            box.prop(self, 'makeMesh')
            box.label(text="Armature Simplification:")
            box.prop(self, 'armLevels')
            box.prop(self, 'boneStep')

    def execute(self, context):
        # Ensure the use of the global variables
        global settings, useSet, is_first
        start_time = time.time()
        # If we need to set the properties from a preset then do it here
        if useSet:
            for a, b in settings.items():
                setattr(self, a, b)
            if self.limitImport:
                setattr(self, 'levels', min(settings['levels'], 2))
                setattr(self, 'showLeaves', False)
            useSet = False
        if self.do_update:
            addTree(self)
            #cProfile.runctx("addTree(self)", globals(), locals())
            print("Tree creation in %0.1fs" %(time.time()-start_time))
            
            #backup most recent setengs in case of exit
            if not is_first:
                # Here we create a dict of all the properties.
                data = []
                for a, b in (self.as_keywords(ignore=("chooseSet", "presetName", "limitImport", "do_update", "overwrite", "leafDupliObj"))).items():
                    # If the property is a vector property then add the slice to the list
                    try:
                        len(b)
                        data.append((a, b[:]))
                    # Otherwise, it is fine so just add it
                    except:
                        data.append((a, b))
                # Create the dict from the list
                data = dict(data)

                #then save
                bpy.ops.sapling.exportdata("INVOKE_DEFAULT", data = repr([repr(data), "PreviousSettings", True]))
            
            is_first = False
            
            return {'FINISHED'}
        else:
            return {'PASS_THROUGH'}
    
    def invoke(self, context, event):
        global is_first
        is_first = True
        bpy.ops.sapling.importdata(filename = "Default Tree.py")
        return self.execute(context)


class AddMultipleTrees(bpy.types.Operator):
    bl_idname = "curve.trees_add"
    bl_label = "Sapling: Add Trees"
    bl_options = {'REGISTER', 'UNDO'}
    
    def objectList(self, context):
        objects = []
        bObjects = bpy.data.objects
#        try:
#            bObjects = bpy.data.objects
#        except AttributeError:
#            pass
#        else:
        for obj in bObjects:
            if (obj.type in ['MESH', 'CURVE', 'SURFACE']) and (obj.name not in ['tree', 'leaves']):
                objects.append((obj.name, obj.name, ""))
        return objects

    do_update: BoolProperty(name='Re-grow Trees',
        default=True)

    bevel: BoolProperty(name='Bevel',
        description='Whether the curve is beveled',
        default=False)
    showLeaves: BoolProperty(name='Show Leaves',
        description='Whether the leaves are shown',
        default=False)
    useArm: BoolProperty(name='Use Armature',
        description='Whether the armature is generated',
        default=False)
    seed: IntProperty(name='Random Seed',
        description='The seed of the random number generator',
        default=0)
    numTrees: IntProperty(name='Number of trees',
        description='Number of trees to generate while incrementing the seed',
        min=1,
        default=1)

    scale: FloatProperty(name='Scale',
        description='The tree scale (Scale)',
        min=0.0,
        default=13.0)
    scaleV: FloatProperty(name='Scale Variation',
        description='The variation in the tree scale',
        default=3.0)

    bevelRes: IntProperty(name='Bevel Resolution',
        description='The bevel resolution of the curves',
        min=0,
        max=32,
        default=0)
    resU: IntProperty(name='Curve Resolution',
        description='The resolution along the curves',
        min=1,
        default=4)
    
    leafShape: EnumProperty(name='Leaf Shape',
        description='The shape of the leaves',
        items=(('hex', 'Hexagonal', '0'), ('rect', 'Rectangular', '1'), ('dFace', 'DupliFaces', '2'), ('dVert', 'DupliVerts', '3')),
        default='hex')
    leafDupliObj: EnumProperty(name='Leaf Object',
        description='Object to use for leaf instancing if Leaf Shape is DupliFaces or DupliVerts',
        items=objectList)
    leafObjZ: EnumProperty(name='',
        description='leaf tip axis',
        items=axes,
        default="+2")
    leafObjY: EnumProperty(name='',
        description='leaf top axis',
        items=axes,
        default="+1")
    leafScale: FloatProperty(name='Leaf Scale',
        description='The scaling applied to the whole leaf (LeafScale)',
        min=0.0,
        default=0.17)
    
    armAnim: BoolProperty(name='Armature Animation',
        description='Whether animation is added to the armature',
        default=False)
    previewArm: BoolProperty(name='Fast Preview',
        description='Disable armature modifier, hide tree, and set bone display to wire, for fast playback',
        ##Disable skin modifier and hide tree and armature, for fast playback
        default=False)
    leafAnim: BoolProperty(name='Leaf Animation',
        description='Whether animation is added to the leaves',
        default=False)
    frameRate: FloatProperty(name='Animation Speed',
        description=('Adjust speed of animation, relative to scene frame rate'),
        min=0.001,
        default=1)
    loopFrames: IntProperty(name='Loop Frames',
        description='Number of frames to make the animation loop for, zero is disabled',
        min=0,
        default=0)
    
    makeMesh: BoolProperty(name='Make Mesh',
        description='Convert curves to mesh, uses skin modifier, enables armature simplification',
        default=False)
    armLevels: IntProperty(name='Armature Levels',
        description='Number of branching levels to make bones for, 0 is all levels',
        min=0,
        default=2)
    boneStep: IntVectorProperty(name='Bone Length',
        description='Number of stem segments per bone',
        min=1,
        default=[1, 1, 1, 1],
        size=4)

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT'

    def draw(self, context):

        layout = self.layout
        
        layout.prop(self, 'do_update', icon='CURVE_DATA')

        box = layout.box()
        box.label(text="Geometry:")
        
        row = box.row()
        row.menu('SAPLING_MT_preset', text='Load Preset')

        box.prop(self, 'seed')
        box.prop(self, 'numTrees')
        
        box.prop(self, 'bevel')
        
        row = box.row()
        row.prop(self, 'bevelRes')
        row.prop(self, 'resU')

        box.label(text="Tree Scale:")
        row = box.row()
        row.prop(self, 'scale')
        row.prop(self, 'scaleV')
        
        box = layout.box()
        box.label(text="Leaves:")
        
        box.prop(self, 'showLeaves')
        box.prop(self, 'leafShape')
        box.prop(self, 'leafDupliObj')
        row = box.row()
        row.label(text="Leaf Object Axes:")
        row.prop(self, 'leafObjZ')
        row.prop(self, 'leafObjY')
            
        #box.prop(self, 'leaves')
        box.prop(self, 'leafScale')

        box = layout.box()
        box.label(text="Armature and Animation:")
        
        row = box.row()
        row.prop(self, 'useArm')
        row.prop(self, 'previewArm')

        row = box.row()
        row.prop(self, 'armAnim')
        row.prop(self, 'leafAnim')

        box.prop(self, 'frameRate')
        box.prop(self, 'loopFrames')
        
        box.label(text="")
        box.prop(self, 'makeMesh')
        box.label(text="Armature Simplification:")
        box.prop(self, 'armLevels')
        box.prop(self, 'boneStep')


    def execute(self, context):
        # Ensure the use of the global variables
        global settings, useSet
        # If we need to set the properties from a preset then do it here
        #presetAsDict
        if useSet:
            for a, b in settings.items():
                setattr(self, a, b)
            useSet = False
        if self.do_update:
            #loop for multiple trees
            initSeed = self.seed
            for n in range(self.numTrees):
                setattr(self, 'seed', self.seed + n)
                space = 6
                grid = ceil(self.numTrees ** .5)
                x = (n // grid) * space
                y = (n % grid) * space
                bpy.context.scene.cursor.location = (x, y, 0)
                addTree(self)
            setattr(self, 'seed', initSeed)
            self.do_update = False
            return {'FINISHED'}
        else:
            return {'PASS_THROUGH'}
    
    def invoke(self, context, event):
#        global settings, useSet
#        useSet = True
        bpy.ops.sapling.importdata(filename = "Default Tree.py")
        return self.execute(context)


def menu_func(self, context):
    self.layout.operator(AddTree.bl_idname, text="Add Tree 4.0", icon='PLUGIN')
def menu_func2(self, context):
    self.layout.operator(AddMultipleTrees.bl_idname, text="Add Multiple Trees", icon='PLUGIN')

classes = (
    AddTree,
    AddMultipleTrees,
    PresetMenu,
    ImportData,
    ExportData,
)


def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    bpy.types.VIEW3D_MT_curve_add.append(menu_func)
    bpy.types.VIEW3D_MT_curve_add.append(menu_func2)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)
    bpy.types.VIEW3D_MT_curve_add.remove(menu_func2)
    bpy.types.VIEW3D_MT_curve_add.remove(menu_func)

if __name__ == "__main__":
    register()
