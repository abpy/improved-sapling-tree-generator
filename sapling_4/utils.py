# -*- coding: utf-8 -*-

# ##### BEGIN GPL LICENSE BLOCK #####
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
# ##### END GPL LICENSE BLOCK #####

print("version 4 imported")

import bpy
import time
import copy

from mathutils import *
from math import pi, sin, degrees, radians, atan2, copysign, cos, acos, sqrt
from math import floor, ceil
from random import random, uniform, seed, choice, getstate, setstate, randint
from bpy.props import *
from collections import deque, OrderedDict

tau = 2 * pi

# Initialise the split error and axis vectors
splitError = 0.0
zAxis = Vector((0, 0, 1))
yAxis = Vector((0, 1, 0))
xAxis = Vector((1, 0, 0))

# This class will contain a part of the tree which needs to be extended and the required tree parameters
class stemSpline:
    def __init__(self, spline, curvature, curvatureV, attractUp, segments, maxSegs, segLength, childStems, stemRadStart, stemRadEnd, splineNum, ofst, pquat):
        self.spline = spline
        self.p = spline.bezier_points[-1]
        self.curv = curvature
        self.curvV = curvatureV
        self.vertAtt = attractUp
        self.seg = segments
        self.segMax = maxSegs
        self.segL = segLength
        self.children = childStems
        self.radS = stemRadStart
        self.radE = stemRadEnd
        self.splN = splineNum
        self.offsetLen = ofst
        self.patentQuat = pquat
        
        self.curvSignx = choice([1, -1])
        self.curvSigny = choice([1, -1])
        self.splitSignx = choice([1, -1])
        self.splitSigny = choice([1, -1])
        
    # This method determines the quaternion of the end of the spline
    def quat(self):
        if len(self.spline.bezier_points) == 1:
            return ((self.spline.bezier_points[-1].handle_right - self.spline.bezier_points[-1].co).normalized()).to_track_quat('Z', 'Y')
        else:
            return ((self.spline.bezier_points[-1].co - self.spline.bezier_points[-2].co).normalized()).to_track_quat('Z', 'Y')
    # Update the end of the spline and increment the segment count
    def updateEnd(self):
        self.p = self.spline.bezier_points[-1]
        self.seg += 1

# This class contains the data for a point where a new branch will sprout
class childPoint:
    def __init__(self, coords, quat, radiusPar, offset, sOfst, lengthPar, parBone):
        self.co = coords
        self.quat = quat
        self.radiusPar = radiusPar
        self.offset = offset
        self.stemOffset = sOfst
        self.lengthPar = lengthPar
        self.parBone = parBone


# This function calculates the shape ratio
def shapeRatio(shape, ratio, custom=None):
    if shape == 0:
        return 0.05 + 0.95*ratio #0.2 + 0.8*ratio
    elif shape == 1:
        return 0.2 + 0.8*sin(pi*ratio)
    elif shape == 2:
        return 0.2 + 0.8*sin(0.5*pi*ratio)
    elif shape == 3:
        return 1.0
    elif shape == 4:
        return 0.5 + 0.5*ratio
    elif shape == 5:
        if ratio <= 0.7:
            return 0.05 + 0.95 * ratio/0.7
        else:
            return 0.05 + 0.95 * (1.0 - ratio)/0.3
    elif shape == 6:
        return 1.0 - 0.8*ratio
    elif shape == 7:
        if ratio <= 0.7:
            return 0.5 + 0.5*ratio/0.7
        else:
            return 0.5 + 0.5*(1.0 - ratio)/0.3
    elif shape == 9: # old custom shape
        r = 1 - ratio
        if r == 1:
            v = custom[3]
        elif r >= custom[2]:
            pos = (r - custom[2]) / (1 - custom[2])
            pos = pos * pos
            v = (pos * (custom[3] - custom[1])) + custom[1]
        else:
            pos = r / custom[2]
            pos = 1 - (1 - pos) * (1 - pos)
            v = (pos * (custom[1] - custom[0])) + custom[0]
        return v
    elif shape == 8:
        r = 1 - ratio
        custom[2] = min(custom[2], .99)
        if r == 1:
            v = custom[3]
        else:
            if r >= custom[2]:
                t = (r - custom[2]) / (1 - custom[2])
                p1 = custom[1]
                p2 = custom[3]
            else:
                t = r / custom[2]
                p1 = custom[0]
                p2 = custom[1]
                
            slope1 = (custom[3]-custom[1])/(1-custom[2])
            slope2 = (custom[1]-custom[0])/custom[2]
            slope = (slope1 + slope2) / 2
            flat = False
            
            if (slope1 > 0 > slope2) or (slope1 < 0 < slope2):
                slope = 0.0
                flat = True
            
            h1 = slope * ((1 - custom[2])/2) + custom[1]
            h2 = -slope * (custom[2]/2) + custom[1]
            
            if not flat:
                if (h1 < custom[3]) and (custom[0] > custom[3]):
                    h1 = custom[3]
                    slope = slope1*2
                    h2 = -slope * (custom[2]/2) + custom[1]
                if (h2 < custom[0]) and (custom[3] > custom[0]):
                    h2 = custom[0]
                    slope = slope2*2
                    h1 = slope * ((1 - custom[2])/2) + custom[1]
                if (h1 > custom[3]) and (custom[0] < custom[3]):
                    h1 = custom[3]
                    slope = slope1*2
                    h2 = -slope * (custom[2]/2) + custom[1]
                if (h2 > custom[0]) and (custom[3] < custom[0]):
                    h2 = custom[0]
                    slope = slope2*2
                    h1 = slope * ((1 - custom[2])/2) + custom[1]
                
            if r >= custom[2]:
                h = h1
            else:
                h = h2
            v = ((1-t)**2)*p1 + (2*t*(1-t))*h + (t**2)*p2
        return v
    elif shape == 10:
        return 0.5 + 0.5 * (1 - ratio)

# This function determines the actual number of splits at a given point using the global error
def splits(n):
    global splitError
    nEff = round(n + splitError, 0)
    splitError -= (nEff - n)
    return int(nEff)

def splits2(n):
    r = random()
    if r < n:
        return 1
    else:
        return 0

def splits3(n): #for 3+ splits #not used
    ni = int(n)
    nf = n - int(n)
    r = random()
    if r < nf:
        return ni + 1
    else:
        return ni + 0

# Determine the declination from a given quaternion
def declination(quat):
    tempVec = zAxis.copy()
    tempVec.rotate(quat)
    tempVec.normalize()
    return degrees(acos(tempVec.z))

# Determines the angle of upward rotation of a segment due to attractUp
def curveUp(attractUp, quat, curveRes):
    tempVec = yAxis.copy()
    tempVec.rotate(quat)
    tempVec.normalize()
    dec = radians(declination(quat))
    curveUpAng = attractUp*dec*abs(tempVec.z)/curveRes
    if (-dec + curveUpAng) < -pi:
        curveUpAng = -pi + dec
    if (dec - curveUpAng) < 0:
        curveUpAng = dec
    return curveUpAng

def curveDown(attractUp, quat, curveRes, length):
    #(sCurv, dirVec.to_track_quat('Z', 'Y'), stem.segMax, stem.segL * stem.segMax)
    tempVec = yAxis.copy()
    tempVec.rotate(quat)
    tempVec.normalize()
    dec = radians(declination(quat))
    curveUpAng = attractUp*abs(tempVec.z)*length/curveRes
    if (-dec + curveUpAng) < -pi:
        curveUpAng = -pi + dec
    if (dec - curveUpAng) < 0:
        curveUpAng = dec
    return curveUpAng

# Evaluate a bezier curve for the parameter 0<=t<=1 along its length
def evalBez(p1, h1, h2, p2, t):
    return ((1-t)**3)*p1 + (3*t*(1-t)**2)*h1 + (3*(t**2)*(1-t))*h2 + (t**3)*p2

# Evaluate the unit tangent on a bezier curve for t
def evalBezTan(p1, h1, h2, p2, t):
    return ((-3*(1-t)**2)*p1 + (-6*t*(1-t) + 3*(1-t)**2)*h1 + (-3*(t**2) + 6*t*(1-t))*h2 + (3*t**2)*p2).normalized()

# Determine the range of t values along a splines length where child stems are formed
def findChildPoints(stemList, numChild):
    numSegs = sum([len(n.spline.bezier_points) - 1 for n in stemList])
    numPerSeg = numChild/numSegs
    numMain = round(numPerSeg*stemList[0].segMax, 0)
    return [(a+1)/(numMain) for a in range(int(numMain))]

def findChildPoints2(numChild):
    return [(a+1)/(numChild) for a in range(int(numChild))]

def findChildPoints3(stemList, numChild, rp=0.5):
    maxSegs = stemList[0].segMax
    segNum = [0] * maxSegs
    for stem in stemList:
        segs = len(stem.spline.bezier_points)-2
        for n in range(0, segs+1):
            segNum[n] += 1
    segNum = segNum[::-1]
    
    childPoints = []
    for i, s in enumerate(segNum):      
        start = i / maxSegs
        end = (i+1) / maxSegs
        numPoints = int(round((numChild / maxSegs) / s ** rp))
        cp = [((a / numPoints) * (end - start) + start) for a in range(numPoints)]
        childPoints.extend(cp)
    return childPoints

def findChildPoints4(stemList, numChild):
    maxOffset = max([s.offsetLen + (len(s.spline.bezier_points) - 1) * s.segL for s in stemList])
    stemLengths = []
    for stem in stemList:
        stemLen = stem.offsetLen + stem.segL*(len(stem.spline.bezier_points) - 1)
        stemLengths.append(stemLen / maxOffset)
    
    print(stemLengths)
    
    return [(a+1)/(numChild) for a in range(int(numChild))]

def interpStem(stem, tVals, maxOffset, baseSize):
    points = stem.spline.bezier_points
    numSegs = len(points) - 1
    stemLen = stem.segL * numSegs
    
    checkBottom = stem.offsetLen / maxOffset
    checkTop = checkBottom + (stemLen / maxOffset)
    
    # Loop through all the parametric values to be determined
    tempList = deque()
    for t in tVals:
        if (t >= checkBottom) and (t <= checkTop) and (t < 1.0):
            scaledT = (t - checkBottom) / (checkTop - checkBottom)
            ofst = ((t - baseSize) / (checkTop - baseSize)) * (1 - baseSize) + baseSize
            
            length = numSegs * scaledT
            index = int(length)
            tTemp = length - index
            
            coord = evalBez(points[index].co, points[index].handle_right, points[index+1].handle_left, points[index+1].co, tTemp)
            quat = (evalBezTan(points[index].co, points[index].handle_right, points[index+1].handle_left, points[index+1].co, tTemp)).to_track_quat('Z', 'Y')
            radius = (1-tTemp)*points[index].radius + tTemp*points[index+1].radius # radius at the child point
            
            tempList.append(childPoint(coord, quat, (stem.radS, radius, stem.radE), t, ofst, stem.segMax * stem.segL, 'bone'+(str(stem.splN).rjust(3, '0'))+'.'+(str(index).rjust(3, '0'))))
        elif t == 1:
            #add stems at tip
            index = numSegs-1
            coord = points[-1].co
            quat = (points[-1].handle_right - points[-1].co).to_track_quat('Z', 'Y')
            radius = points[-1].radius
            tempList.append(childPoint(coord, quat, (stem.radS, radius, stem.radE), 1, 1, stem.segMax * stem.segL, 'bone'+(str(stem.splN).rjust(3, '0'))+'.'+(str(index).rjust(3, '0'))))
            
    return tempList

# round down bone number
def roundBone(bone, step):
    bone_i = bone[:-3]
    bone_n = int(bone[-3:])
    bone_n = int(bone_n / step) * step
    return bone_i + str(bone_n).rjust(3, '0')

# Convert a list of degrees to radians
def toRad(list):
    return [radians(a) for a in list]

def anglemean(a1, a2, fac):
    x1 = sin(a1)
    y1 = cos(a1)
    x2 = sin(a2)
    y2 = cos(a2)
    x = x1 + (x2 - x1) * fac
    y = y1 + (y2 - y1) * fac
    return atan2(x, y)

# convert quat to use declination without rotation
def convertQuat(quat):
    adir = zAxis.copy()
    adir.rotate(quat)
    dec = radians(declination(quat))
    axis = Vector((-adir[1], adir[0], 0))
    return Matrix.Rotation(dec, 3, axis)


# This is the function which extends (or grows) a given stem.
def growSpline(n, stem, numSplit, splitAng, splitAngV, splitStraight, splineList, hType, splineToBone, closeTip, splitRadiusRatio,
               minRadius, kp, splitHeight, outAtt, splitLength, lenVar, taperCrown, boneStep, rotate, rotateV, matIndex):
    
    #curv at base
    sCurv = stem.curv
    if (n == 0) and (kp <= splitHeight):
        sCurv = 0.0
    
    curveangle = sCurv + (uniform(0, stem.curvV) * kp * stem.curvSignx)
    curveVar = uniform(0, stem.curvV) * kp * stem.curvSigny
    stem.curvSignx *= -1
    stem.curvSigny *= -1
    
    curveVarMat = Matrix.Rotation(curveVar, 3, 'Y')
    
    # First find the current direction of the stem
    dir = stem.quat()
    
    #length taperCrown
    if n == 0:
        dec = declination(dir) / 180
        dec = dec ** 2
        tf = 1 - (dec * taperCrown * 30)
        tf = max(.1, tf)
    else:
        tf = 1.0
    tf = 1.0 #disabled
    
    #outward attraction
    if (n >= 0) and (kp > 0) and (outAtt > 0):
        p = stem.p.co.copy()
        d = atan2(p[0], -p[1])# + tau
        edir = dir.to_euler('XYZ', Euler((0, 0, d), 'XYZ'))
        d = anglemean(edir[2], d, (kp * outAtt))
        dirv = Euler((edir[0], edir[1], d), 'XYZ')
        dir = dirv.to_quaternion()
    
    if n == 0:
        dir = convertQuat(dir)
    
    if n != 0:
        splitLength = 0
    
    # If the stem splits, we need to add new splines etc
    if numSplit > 0:
        # Get the curve data
        cuData = stem.spline.id_data.name
        cu = bpy.data.curves[cuData]

        #calc split angles
        splitAng = splitAng/2
        if n == 0:
            angle = splitAng + uniform(-splitAngV, splitAngV)
        else:
            #angle = stem.splitSigny * (splitAng + uniform(-splitAngV, splitAngV))
            #stem.splitSigny = -stem.splitSigny
            angle = choice([1, -1]) * (splitAng + uniform(-splitAngV, splitAngV))
        if n > 0:
            #make branches flatter
            angle *= max(1 - declination(dir) / 90, 0) * .67 + .33
        
        spreadangle = stem.splitSignx * (splitAng + uniform(-splitAngV, splitAngV))
        stem.splitSignx = -stem.splitSignx
        
        branchStraightness = splitStraight#0
        if n == 0:
            branchStraightness = splitStraight
        
        if not hasattr(stem, 'rLast'):
            stem.rLast = radians(uniform(0, 360))
        
        br = rotate[0] + uniform(-rotateV[0], rotateV[0])
        branchRot = stem.rLast + br
        branchRotMat = Matrix.Rotation(branchRot, 3, 'Z')
        stem.rLast = branchRot
        
        # Now for each split add the new spline and adjust the growth direction
        for i in range(numSplit):
            #find split scale and length variation for split branches
            lenV = (1-splitLength) * uniform(1-lenVar, 1+(splitLength * lenVar))
            lenV = max(lenV, 0.01) * tf
            bScale = min(lenV, 1)
            
            #split radius factor
            splitR = splitRadiusRatio #0.707 #sqrt(1/(numSplit+1))
            
            if splitRadiusRatio == 0:
                splitR1 = sqrt(.5 * bScale)
                splitR2 = sqrt(1 - (.5 * bScale))
#            elif splitRadiusRatio == -1:
#                ra = lenV / (lenV + 1)
#                splitR1 = sqrt(ra)
#                splitR2 = sqrt(1-ra)
#            elif splitRadiusRatio == 0:
#                splitR1 = sqrt(0.5) * bScale
#                splitR2 = sqrt(1 - splitR1*splitR1)
#                
#                #splitR2 = sqrt(1 - (0.5 * (1-splitLength)))
            else:
                splitR1 = splitR * bScale
                splitR2 = splitR
            
            newSpline = cu.splines.new('BEZIER')
            newSpline.material_index = matIndex[n]
            newPoint = newSpline.bezier_points[-1]
            (newPoint.co, newPoint.handle_left_type, newPoint.handle_right_type) = (stem.p.co, 'VECTOR', 'VECTOR')
            newRadius = (stem.radS*(1 - stem.seg/stem.segMax) + stem.radE*(stem.seg/stem.segMax)) * splitR1
            newRadius = max(newRadius, minRadius)
            newPoint.radius = newRadius
            
            # Here we make the new "sprouting" stems diverge from the current direction
            divRotMat = Matrix.Rotation(angle * (1+branchStraightness) - curveangle, 3, 'X')
            dirVec = zAxis.copy()
            dirVec.rotate(divRotMat)
            
            #horizontal curvature variation
            dirVec.rotate(curveVarMat)
            
            if n == 0: #Special case for trunk splits
                dirVec.rotate(branchRotMat)
                
                ang = pi - ((tau) / (numSplit + 1)) * (i+1)
                dirVec.rotate(Matrix.Rotation(ang, 3, 'Z'))

            # Spread the stem out horizontally
            if n != 0: #Special case for trunk splits
                spreadMat = Matrix.Rotation(spreadangle * (1+branchStraightness), 3, 'Y')
                dirVec.rotate(spreadMat)
            
            dirVec.rotate(dir)
            
            # Introduce upward curvature
            upRotAxis = xAxis.copy()
            upRotAxis.rotate(dirVec.to_track_quat('Z', 'Y'))
            curveUpAng = curveUp(stem.vertAtt, dirVec.to_track_quat('Z', 'Y'), stem.segMax)
            upRotMat = Matrix.Rotation(-curveUpAng, 3, upRotAxis)
            dirVec.rotate(upRotMat)
            
            # Make the growth vec the length of a stem segment
            dirVec.normalize()
            
            #split length variation
            stemL = stem.segL * lenV
            dirVec *= stemL * tf
            ofst = stem.offsetLen + (stem.segL * (len(stem.spline.bezier_points) - 1))
            
            # Get the end point position
            end_co = stem.p.co.copy()

            # Add the new point and adjust its coords, handles and radius
            newSpline.bezier_points.add(1)
            newPoint = newSpline.bezier_points[-1]
            (newPoint.co, newPoint.handle_left_type, newPoint.handle_right_type) = (end_co + dirVec, hType, hType)
            
            newRadius = (stem.radS*(1 - (stem.seg + 1)/stem.segMax) + stem.radE*((stem.seg + 1)/stem.segMax)) * splitR1
            newRadius = max(newRadius, minRadius)
            nRadS = max(stem.radS * splitR1, minRadius)
            nRadE = max(stem.radE * splitR1, minRadius)
            if (stem.seg == stem.segMax-1) and closeTip:
                newRadius = 0.0
            newPoint.radius = newRadius
            
            # Add nstem to splineList
            nstem = stemSpline(newSpline, stem.curv, stem.curvV, stem.vertAtt, stem.seg+1, stem.segMax, stemL, stem.children,
                               nRadS, nRadE, len(cu.splines)-1, ofst, stem.quat())
            nstem.splitlast = 1 #numSplit #keep track of numSplit for next stem
            nstem.rLast = branchRot + pi
            nstem.splitSignx = stem.splitSignx
            if hasattr(stem, 'isFirstTip'):
                nstem.isFirstTip = True
            splineList.append(nstem)
            bone = 'bone'+(str(stem.splN)).rjust(3, '0')+'.'+(str(len(stem.spline.bezier_points)-2)).rjust(3, '0')
            bone = roundBone(bone, boneStep[n])
            splineToBone.append((bone, False, True, len(stem.spline.bezier_points)-2))
                
        # The original spline also needs to keep growing so adjust its direction too
        divRotMat = Matrix.Rotation(-angle * (1-branchStraightness) - curveangle, 3, 'X')
        dirVec = zAxis.copy()
        dirVec.rotate(divRotMat)
        
        #horizontal curvature variation
        dirVec.rotate(curveVarMat)
        
        if n == 0: #Special case for trunk splits
            dirVec.rotate(branchRotMat)
        
        #spread
        if n != 0: #Special case for trunk splits
            spreadMat = Matrix.Rotation(-spreadangle * (1-branchStraightness), 3, 'Y')
            dirVec.rotate(spreadMat)
        
        dirVec.rotate(dir)
        
        stem.splitlast = 1 #numSplit #keep track of numSplit for next stem
        
    else:
        # If there are no splits then generate the growth direction without accounting for spreading of stems
        dirVec = zAxis.copy()
        divRotMat = Matrix.Rotation(-curveangle, 3, 'X')
        dirVec.rotate(divRotMat)
        
        #horizontal curvature variation
        dirVec.rotate(curveVarMat)
        
        dirVec.rotate(dir)
        
        stem.splitlast = 0 #numSplit #keep track of numSplit for next stem
    
    # Introduce upward curvature
    upRotAxis = xAxis.copy()
    upRotAxis.rotate(dirVec.to_track_quat('Z', 'Y'))
    curveUpAng = curveUp(stem.vertAtt, dirVec.to_track_quat('Z', 'Y'), stem.segMax)
    upRotMat = Matrix.Rotation(-curveUpAng, 3, upRotAxis)
    dirVec.rotate(upRotMat)
    
    dirVec.normalize()
    dirVec *= stem.segL * tf

    # Get the end point position
    end_co = stem.p.co.copy() + dirVec

    stem.spline.bezier_points.add(1)
    newPoint = stem.spline.bezier_points[-1]
    (newPoint.co, newPoint.handle_left_type, newPoint.handle_right_type) = (end_co, hType, hType)
    
    newRadius = stem.radS*(1 - (stem.seg + 1)/stem.segMax) + stem.radE*((stem.seg + 1)/stem.segMax)
    if numSplit > 0:
        newRadius = max(newRadius * splitR2, minRadius)
        stem.radS = max(stem.radS * splitR2, minRadius)
        stem.radE = max(stem.radE * splitR2, minRadius)
    newRadius = max(newRadius, stem.radE)
    if (stem.seg == stem.segMax-1) and closeTip:
        newRadius = 0.0
    newPoint.radius = newRadius
    
    # Set bezier handles for first point.
    if len(stem.spline.bezier_points) == 2:
        tempPoint = stem.spline.bezier_points[0]
        if hType is 'AUTO':
            dirVec = zAxis.copy()
            dirVec.rotate(dir)
            dirVec = dirVec * stem.segL * 0.33
            (tempPoint.handle_left_type, tempPoint.handle_right_type) = ('ALIGNED', 'ALIGNED')
            tempPoint.handle_right = tempPoint.co + dirVec
            tempPoint.handle_left = tempPoint.co - dirVec
        elif hType is 'VECTOR':
            (tempPoint.handle_left_type, tempPoint.handle_right_type) = ('VECTOR', 'VECTOR')
        
    # Update the last point in the spline to be the newly added one
    stem.updateEnd()

def genLeafMesh(leafScale, leafScaleX, leafScaleT, leafScaleV, loc, quat, offset, index, downAngle, downAngleV, rotate, rotateV, oldRot,
                leaves, leafShape, leafangle, leafType, ln, leafObjRot):
    if leafShape == 'hex':
        verts = [Vector((0, 0, 0)), Vector((0.5, 0, 1/3)), Vector((0.5, 0, 2/3)), Vector((0, 0, 1)), Vector((-0.5, 0, 2/3)), Vector((-0.5, 0, 1/3))]
        edges = [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 0], [0, 3]]
        faces = [[0, 1, 2, 3], [0, 3, 4, 5]]
    elif leafShape == 'rect':
        #verts = [Vector((1, 0, 0)), Vector((1, 0, 1)), Vector((-1, 0, 1)), Vector((-1, 0, 0))]
        verts = [Vector((.5, 0, 0)), Vector((.5, 0, 1)), Vector((-.5, 0, 1)), Vector((-.5, 0, 0))]
        edges = [[0, 1], [1, 2], [2, 3], [3, 0]]
        faces = [[0, 1, 2, 3]]
    elif leafShape == 'dFace':
        verts = [Vector((.5, .5, 0)), Vector((.5, -.5, 0)), Vector((-.5, -.5, 0)), Vector((-.5, .5, 0))]
        edges = [[0, 1], [1, 2], [2, 3], [3, 0]]
        faces = [[0, 3, 2, 1]]
    elif leafShape == 'dVert':
        verts = [Vector((0, 0, 1))]
        edges = []
        faces = []

    vertsList = []
    facesList = []
    normal = Vector((0, 0, 1))
    
    if leafType in ['0', '5']:
        oldRot += radians(137.5)
    elif leafType == '1':
        if ln % 2:
            oldRot += radians(180)
        else: 
            oldRot += radians(137.5)
    elif leafType in ['2', '3']:
        oldRot = -copysign(rotate, oldRot)
    elif leafType == '4':
        rotMat = Matrix.Rotation(oldRot + uniform(-rotateV, rotateV), 3, 'Y')
        if leaves == 1:
            rotMat = Matrix.Rotation(0, 3, 'Y')
        else:
            oldRot += rotate / (leaves - 1)
    
    if leafType != '4':
        rotMat = Matrix.Rotation(oldRot + uniform(-rotateV, rotateV), 3, 'Z')
    
    # reduce downAngle if leaf is at branch tip
    if (offset == 1):
        if leafType in ['0', '1', '2', '5']:
            downAngle = downAngle * .67
        elif leafType == '3':
            if (leaves / 2) == (leaves // 2):
                downAngle = downAngle * .67
            else:
                downAngle = 0
                downAngleV = 0
    
    if leafType != '4':
        downV = -downAngleV * offset ** 2
        downRotMat = Matrix.Rotation(downAngle + downV + uniform(-rotateV*0, rotateV*0), 3, 'X')
    
    zVar = Matrix.Rotation(uniform(-rotateV, rotateV), 3, 'Z')
    
    #leaf scale variation
    if (leafType == '4') and (rotate != 0) and (leaves > 1):
        f = 1 - abs((oldRot - (rotate / (leaves - 1))) / (rotate / 2))
    else:
        f = offset
    
    if leafScaleT < 0:
        leafScale = leafScale * (1 - (1 - f) * -leafScaleT)
    else:
        leafScale = leafScale * (1 - f * leafScaleT)
        
    leafScale = leafScale * uniform(1 - leafScaleV, 1 + leafScaleV)
    
    if leafShape == 'dFace':
        leafScale = leafScale * .1
    
    #Rotate leaf vector
    m = Matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        
    if leafType in ['2', '3']:
        m.rotate(Euler((0, 0, radians(90))))
        if oldRot > 0:
            m.rotate(Euler((0, 0, radians(180))))

    if leafType != '4':
        m.rotate(downRotMat)

    m.rotate(rotMat)
    m.rotate(quat)
    
    # convert rotation for upward facing leaves
    if leafType in ['4', '5']:
        lRot = m
    else:
        v = zAxis.copy()
        v.rotate(m)
        lRot = v.to_track_quat('Z', 'Y')

    # For each of the verts we now rotate and scale them, then append them to the list to be added to the mesh
    for v in verts:
        v.z *= leafScale
        v.y *= leafScale
        v.x *= leafScaleX*leafScale
        
        if leafShape in ['dVert', 'dFace']:
            v.rotate(leafObjRot)
        
        v.rotate(Euler((0, 0, radians(180))))
        
        #rotate variation
        v.rotate(zVar)
        
        #leafangle
        v.rotate(Matrix.Rotation(radians(-leafangle), 3, 'X'))
        
        v.rotate(lRot)
    
    if leafShape == 'dVert':
        normal = verts[0]
        normal.normalize()
        v = loc
        vertsList.append([v.x, v.y, v.z])
    else:
        for v in verts:
            v += loc
            vertsList.append([v.x, v.y, v.z])
        for f in faces:
            facesList.append([f[0] + index, f[1] + index, f[2] + index, f[3] + index])
            
    return vertsList, facesList, normal, oldRot


def create_armature(armAnim, leafP, cu, frameRate, leafMesh, leafObj, leafVertSize, leaves, levelCount, splineToBone,
                    treeOb, treeObj, wind, gust, gustF, af1, af2, af3, leafAnim, loopFrames, previewArm, armLevels, makeMesh, boneStep):
    arm = bpy.data.armatures.new('tree')
    armOb = bpy.data.objects.new('treeArm', arm)
    armOb.location=bpy.context.scene.cursor.location
    bpy.context.scene.collection.objects.link(armOb)
    # Create a new action to store all animation
    newAction = bpy.data.actions.new(name='windAction')
    armOb.animation_data_create()
    armOb.animation_data.action = newAction
    arm.display_type = 'STICK'
    #arm.use_deform_delay = True
    # Add the armature modifier to the curve
    armMod = treeOb.modifiers.new('windSway', 'ARMATURE')
    if previewArm:
        armMod.show_viewport = False
        arm.display_type = 'WIRE'
        treeOb.hide_viewport = True
    armMod.use_apply_on_spline = True
    armMod.object = armOb
    armMod.use_bone_envelopes = True
    armMod.use_vertex_groups = False
    # If there are leaves then they need a modifier
    if leaves:
        armMod = leafObj.modifiers.new('windSway', 'ARMATURE')
        armMod.object = armOb
        armMod.use_bone_envelopes = False
        armMod.use_vertex_groups = True
    # Make sure all objects are deselected (may not be required?)
    for ob in bpy.data.objects:
        ob.select_set(state=False)
    
    fps = bpy.context.scene.render.fps
    animSpeed = (24 / fps) * frameRate

    # Set the armature as active and go to edit mode to add bones
    bpy.context.view_layer.objects.active = armOb
    bpy.ops.object.mode_set(mode='EDIT')
    # For all the splines in the curve we need to add bones at each bezier point
    for i, parBone in enumerate(splineToBone):
        if (i < levelCount[armLevels]) or (armLevels == -1) or (not makeMesh):
            s = cu.splines[i]
            b = None
            # Get some data about the spline like length and number of points
            numPoints = len(s.bezier_points) - 1
            
            #find branching level
            level = 0
            for l, c in enumerate(levelCount):
                if i < c:
                    level = l
                    break
            level = min(level, 3)
            
            step = boneStep[level]

            # Calculate things for animation
            if armAnim:
                splineL = numPoints * ((s.bezier_points[0].co - s.bezier_points[1].co).length)
                # Set the random phase difference of the animation
                bxOffset = uniform(0, tau)
                byOffset = uniform(0, tau)
                # Set the phase multiplier for the spline
                #bMult_r = (s.bezier_points[0].radius / max(splineL, 1e-6)) * (1 / 15) * (1 / frameRate)
                #bMult = degrees(bMult_r)  # This shouldn't have to be in degrees but it looks much better in animation
                bMult = (1 / max(splineL ** .5, 1e-6)) * (1 / 4)
                #print((1 / bMult) * tau) #print wavelength in frames

                windFreq1 = bMult * animSpeed
                windFreq2 = 0.7 * bMult * animSpeed
                if loopFrames != 0:
                    bMult_l = 1 / (loopFrames / tau)
                    fRatio = max(1, round(windFreq1 / bMult_l))
                    fgRatio = max(1, round(windFreq2 / bMult_l))
                    windFreq1 = fRatio * bMult_l
                    windFreq2 = fgRatio * bMult_l

            # For all the points in the curve (less the last) add a bone and name it by the spline it will affect
            nx = 0
            for n in range(0, numPoints, step):
                oldBone = b
                boneName = 'bone' + (str(i)).rjust(3, '0') + '.' + (str(n)).rjust(3, '0')
                b = arm.edit_bones.new(boneName)
                b.head = s.bezier_points[n].co
                nx += step
                nx = min(nx, numPoints)
                b.tail = s.bezier_points[nx].co

                b.head_radius = s.bezier_points[n].radius
                b.tail_radius = s.bezier_points[n + 1].radius
                b.envelope_distance = 0.001

#                # If there are leaves then we need a new vertex group so they will attach to the bone
#                if not leafAnim:
#                    if (len(levelCount) > 1) and (i >= levelCount[-2]) and leafObj:
#                        leafObj.vertex_groups.new(name=boneName)
#                    elif (len(levelCount) == 1) and leafObj:
#                        leafObj.vertex_groups.new(name=boneName)

                # If this is first point of the spline then it must be parented to the level above it
                if n == 0:
                    if parBone:
                        b.parent = arm.edit_bones[parBone]
                # Otherwise, we need to attach it to the previous bone in the spline
                else:
                    b.parent = oldBone
                    b.use_connect = True
                # If there isn't a previous bone then it shouldn't be attached
                if not oldBone:
                    b.use_connect = False

                # Add the animation to the armature if required
                if armAnim:
                    # Define all the required parameters of the wind sway by the dimension of the spline
                    #a0 = 4 * splineL * (1 - n / (numPoints + 1)) / max(s.bezier_points[n].radius, 1e-6)
                    a0 = 2 * (splineL / numPoints) * (1 - n / (numPoints + 1)) / max(s.bezier_points[n].radius, 1e-6)
                    a0 = a0 * min(step, numPoints)
                    #a0 = (splineL / numPoints) / max(s.bezier_points[n].radius, 1e-6)
                    a1 = (wind / 50) * a0
                    a2 = a1 * .65  #(windGust / 50) * a0 + a1 / 2

                    p = s.bezier_points[nx].co - s.bezier_points[n].co
                    p.normalize()
                    ag = (wind * gust / 50) * a0
                    a3 = -p[0] * ag
                    a4 = p[2] * ag

                    a1 = radians(a1)
                    a2 = radians(a2)
                    a3 = radians(a3)
                    a4 = radians(a4)

                    #wind bending
                    if loopFrames == 0:
                        swayFreq = gustF * (tau / fps) * frameRate  #animSpeed # .075 # 0.02
                    else:
                        swayFreq = 1 / (loopFrames / tau)

                    # Prevent tree base from rotating
                    if (boneName == "bone000.000") or (boneName == "bone000.001"):
                        a1 = 0
                        a2 = 0
                        a3 = 0
                        a4 = 0

                    # Add new fcurves for each sway as well as the modifiers
                    swayX = armOb.animation_data.action.fcurves.new('pose.bones["' + boneName + '"].rotation_euler', index=0)
                    swayY = armOb.animation_data.action.fcurves.new('pose.bones["' + boneName + '"].rotation_euler', index=2)

                    swayXMod1 = swayX.modifiers.new(type='FNGENERATOR')
                    swayXMod2 = swayX.modifiers.new(type='FNGENERATOR')

                    swayYMod1 = swayY.modifiers.new(type='FNGENERATOR')
                    swayYMod2 = swayY.modifiers.new(type='FNGENERATOR')

                    # Set the parameters for each modifier
                    swayXMod1.amplitude = a1
                    swayXMod1.phase_offset = bxOffset
                    swayXMod1.phase_multiplier = windFreq1

                    swayXMod2.amplitude = a2
                    swayXMod2.phase_offset = 0.7 * bxOffset
                    swayXMod2.phase_multiplier = windFreq2
                    swayXMod2.use_additive = True

                    swayYMod1.amplitude = a1
                    swayYMod1.phase_offset = byOffset
                    swayYMod1.phase_multiplier = windFreq1

                    swayYMod2.amplitude = a2
                    swayYMod2.phase_offset = 0.7 * byOffset
                    swayYMod2.phase_multiplier = windFreq2
                    swayYMod2.use_additive = True

                    #wind bending
                    swayYMod3 = swayY.modifiers.new(type='FNGENERATOR')
                    swayYMod3.amplitude = a3
                    swayYMod3.phase_multiplier = swayFreq
                    swayYMod3.value_offset = .6 * a3
                    swayYMod3.use_additive = True

                    swayXMod3 = swayX.modifiers.new(type='FNGENERATOR')
                    swayXMod3.amplitude = a4
                    swayXMod3.phase_multiplier = swayFreq
                    swayXMod3.value_offset = .6 * a4
                    swayXMod3.use_additive = True

    if leaves:
        bonelist = [b.name for b in arm.edit_bones]
        vertexGroups = OrderedDict()
        for i, cp in enumerate(leafP):
            # find leafs parent bone
            leafParent = roundBone(cp.parBone, boneStep[armLevels])
            idx = int(leafParent[4:-4])
            while leafParent not in bonelist:
                #find parent bone of parent bone
                leafParent = splineToBone[idx]
                idx = int(leafParent[4:-4])
            
            if leafAnim:
                bname = "leaf" + str(i)
                b = arm.edit_bones.new(bname)
                b.head = cp.co
                b.tail = cp.co + Vector((0, 0, .02))
                b.envelope_distance = 0.0
                b.parent = arm.edit_bones[leafParent]
                
                vertexGroups[bname] = [v.index for v in leafMesh.vertices[leafVertSize * i:(leafVertSize * i + leafVertSize)]]

                if armAnim:
                    # Define all the required parameters of the wind sway by the dimension of the spline
                    a1 = wind * .25
                    a1 *= af1
                    
                    bMult = (1 / animSpeed) * 6
                    bMult *= 1 / max(af2, .001)
                    
                    ofstRand = af3
                    bxOffset = uniform(-ofstRand, ofstRand)
                    byOffset = uniform(-ofstRand, ofstRand)

                    # Add new fcurves for each sway as well as the modifiers
                    swayX = armOb.animation_data.action.fcurves.new('pose.bones["' + bname + '"].rotation_euler', index=0)
                    swayY = armOb.animation_data.action.fcurves.new('pose.bones["' + bname + '"].rotation_euler', index=2)
                    
                    # Add keyframe so noise works
                    swayX.keyframe_points.add(1)
                    swayY.keyframe_points.add(1)
                    swayX.keyframe_points[0].co = (0, 0)
                    swayY.keyframe_points[0].co = (0, 0)
                    
                    # Add noise modifiers
                    swayXMod = swayX.modifiers.new(type='NOISE')
                    swayYMod = swayY.modifiers.new(type='NOISE')
                    
                    if loopFrames != 0:
                        swayXMod.use_restricted_range = True
                        swayXMod.frame_end = loopFrames
                        swayXMod.blend_in = 4
                        swayXMod.blend_out = 4
                        swayYMod.use_restricted_range = True
                        swayYMod.frame_end = loopFrames
                        swayYMod.blend_in = 4
                        swayYMod.blend_out = 4
                    
                    swayXMod.scale = bMult
                    swayXMod.strength = a1
                    swayXMod.offset = bxOffset
                    
                    swayYMod.scale = bMult
                    swayYMod.strength = a1
                    swayYMod.offset = byOffset
            
            else:
                if leafParent not in vertexGroups:
                    vertexGroups[leafParent] = []
                vertexGroups[leafParent].extend([v.index for v in leafMesh.vertices[leafVertSize * i:(leafVertSize * i + leafVertSize)]])
        
        for group in vertexGroups:
            leafObj.vertex_groups.new(name=group)
            leafObj.vertex_groups[group].add(vertexGroups[group], 1.0, 'ADD')

    # Now we need the rotation mode to be 'XYZ' to ensure correct rotation
    bpy.ops.object.mode_set(mode='OBJECT')
    for p in armOb.pose.bones:
        p.rotation_mode = 'XYZ'
    
    treeOb.parent = armOb
    if makeMesh:
        treeObj.parent = armOb
    
    return armOb


def kickstart_trunk(addstem, levels, leaves, branches, cu, downAngle, downAngleV, curve, curveRes, curveV, attractUp, length, lengthV,
                    ratio, ratioPower, scale0, scaleV0, scaleVal, taper, minRadius, rootFlare, matIndex):
    newSpline = cu.splines.new('BEZIER')
    newSpline.material_index = matIndex[0]
    newPoint = newSpline.bezier_points[-1]
    newPoint.co = Vector((0, 0, 0))
    
    #start trunk rotation with downAngle
    tempPos = zAxis.copy()
    downAng = downAngle[0] - .5 * pi
    downAng = downAng# + uniform(-downAngleV[0], downAngleV[0])
    downRot = Matrix.Rotation(downAng, 3, 'X')
    tempPos.rotate(downRot)
    downRot = Matrix.Rotation(downAngleV[0], 3, 'Y')
    tempPos.rotate(downRot)
    handle = tempPos
    newPoint.handle_right = handle
    newPoint.handle_left = -handle
    
    branchL = scaleVal * length[0]
    curveVal = curve[0] / curveRes[0]
    if levels == 1:
        childStems = leaves
    else:
        childStems = branches[1]
    startRad = scaleVal * ratio * scale0 * uniform(1-scaleV0, 1+scaleV0)
    endRad = (startRad * (1 - taper[0])) ** ratioPower
    startRad = max(startRad, minRadius)
    endRad = max(endRad, minRadius)
    newPoint.radius = startRad * rootFlare
    addstem(
        stemSpline(newSpline, curveVal, curveV[0], attractUp[0], 0, curveRes[0], branchL / curveRes[0],
                   childStems, startRad, endRad, 0, 0, None))


def fabricate_stems(addsplinetobone, addstem, baseSize, branches, childP, cu, curve, curveRes, curveV, attractUp,
                    downAngle, downAngleV, leafDist, leaves, leafType, length, lengthV, levels, n, ratio, ratioPower,
                    rotate, rotateV, scaleVal, shape, storeN, taper, shapeS, minRadius, radiusTweak, customShape, rMode, segSplits,
                    useOldDownAngle, useParentAngle, boneStep, matIndex):
    
    #prevent baseSize from going to 1.0
    baseSize = min(0.999, baseSize)
    
    # Store the old rotation to allow new stems to be rotated away from the previous one.
    oldRotate = 0
    
    #use fancy child point selection / rotation
    if (n == 1) and (rMode != "original"):
        childP_T = OrderedDict()
        childP_L = []
        for i, p in enumerate(childP):
            if p.offset == 1:
                childP_L.append(p)
            else:
                p.index = i
                if p.offset not in childP_T:
                    childP_T[p.offset] = [p]
                else:
                    childP_T[p.offset].append(p)
                    
        childP_T = [childP_T[k] for k in sorted(childP_T.keys())]

        childP = []
        rot_a = []
        for p in childP_T:
            if rMode == "rotate":
                if rotate[n] < 0.0:
                    oldRotate = -copysign(rotate[n], oldRotate)
                else:
                    oldRotate += rotate[n]
                bRotate = oldRotate + uniform(-rotateV[n], rotateV[n])
                
                #find center of split branches
                #average
                cx = sum([a.co[0] for a in p]) / len(p)
                cy = sum([a.co[1] for a in p]) / len(p)
                #center of range
                #xc = [a.co[0] for a in p]
                #yc = [a.co[1] for a in p]
                #cx = (max(xc) + min(xc)) / 2
                #cy = (max(yc) + min(yc)) / 2
                
                center = Vector((cx, cy, 0))
                center2 = Vector((-cx, cy))
                    
                #choose start point whose angle is closest to the rotate angle
                a1 = bRotate % tau
                a_diff = []
                for a in p:
                    a = a.co
                    a = a - center
                    a2 = atan2(a[0], -a[1])
                    d = min((a1-a2+tau)%tau, (a2-a1+tau)%tau)
                    a_diff.append(d)

                idx = a_diff.index(min(a_diff))
                
                #find branch end point
                
                br = p[idx]
                b = br.co
                vx = sin(bRotate)
                vy = cos(bRotate)
                v = Vector((vx, vy))
                
                bD = ((b[0] * b[0] + b[1] * b[1]) ** .5)
                
                #acount for length
                bL = br.lengthPar * length[1] * shapeRatio(shape, (1 - br.offset) / (1 - baseSize), custom=customShape)
                
                #account for down angle
                if downAngleV[1] > 0:
                    downA = downAngle[n] + (-downAngleV[n] * (1 - (1 - br.offset) / (1 - baseSize)) ** 2)
                else:
                    downA = downAngle[n]
                if downA < (.5 * pi):
                    downA = sin(downA) ** 2
                    bL *= downA
                
                bL *= 0.33 #adjustment constant value
                v *= (bD + bL) #branch end point
                
                #find actual rotate angle from branch location
                bv = Vector((b[0], -b[1]))
                cv = (v - center2) - bv
                a = atan2(cv[0], cv[1])
                    
                childP.append(p[idx])
                rot_a.append(a)
                
            elif rMode == 'distance1': #distance1
                for i, br in enumerate(p):
                    rotV = rotateV[n] * .5
                    bRotate = rotate[n] * br.index
                    bL = br.lengthPar * length[1] * shapeRatio(shape, (1 - br.stemOffset) / (1 - baseSize), custom=customShape)
                    if downAngleV[1] > 0:
                        downA = downAngle[n] + (-downAngleV[n] * (1 - (1 - br.stemOffset) / (1 - baseSize)) ** 2)
                    else:
                        downA = downAngle[n]
                        
                    downRotMat = Matrix.Rotation(downA, 3, 'X')
                    rotMat = Matrix.Rotation(bRotate, 3, 'Z')
                    
                    bVec = zAxis.copy()
                    bVec.rotate(downRotMat)
                    bVec.rotate(rotMat)
                    bVec.rotate(convertQuat(br.quat))
                    bVec *= bL
                    p1 = bVec + br.co
                    
                    #distance to other branches
                    isIntersect = []
                    for branch in p:
                        p2 = branch.co
                        p3 = p2 - p1
                        l = p3.length * uniform(1.0, 1.1)
                        bL = branch.lengthPar * length[1] * shapeRatio(shape, (1 - branch.stemOffset) / (1 - baseSize), custom=customShape)
                        isIntersect.append(l < bL)

                    del isIntersect[i]
                    
                    if not any(isIntersect):
                        childP.append(br)
                        rot_a.append(bRotate + uniform(-rotV, rotV))
            
            elif rMode == 'distance': #distance2
                bRotate = oldRotate + rotate[n]
                
                cP = []
                rA = []
                bN = []
                for i, br in enumerate(p):
                    rotV = uniform(-rotateV[n]*.5, rotateV[n]*.5)
                    
                    bL = br.lengthPar * length[1] * shapeRatio(shape, (1 - br.stemOffset) / (1 - baseSize), custom=customShape)
                    if downAngleV[1] > 0:
                        downA = downAngle[n] + (-downAngleV[n] * (1 - (1 - br.stemOffset) / (1 - baseSize)) ** 2)
                    else:
                        downA = downAngle[n]
                        
                    downRotMat = Matrix.Rotation(downA, 3, 'X')
                    bRotate = bRotate + rotV
                    rotMat = Matrix.Rotation(bRotate, 3, 'Z')
                    
                    bVec = zAxis.copy()
                    bVec.rotate(downRotMat)
                    bVec.rotate(rotMat)
                    bVec.rotate(convertQuat(br.quat))
                    bVec *= bL
                    p1 = bVec + br.co
                    
                    #distance to other branches
                    isIntersect = []
                    dists = []
                    lengths = []
                    for branch in p:
                        p2 = branch.co
                        p3 = p2 - p1
                        l = p3.length#*rotateV[n]# * uniform(.90, 1.00) # (1.0, 1.1)
                        bL = branch.lengthPar * length[1] * shapeRatio(shape, (1 - branch.stemOffset) / (1 - baseSize), custom=customShape)
                        isIntersect.append(l < bL)
                        
                        d = br.co - branch.co
                        dists.append(d.length)
                        lengths.append(bL)
                    
                    del isIntersect[i]
                    del dists[i]
                    del lengths[i]
                    
                    if len(dists) > 0:
                        #nearest = min(dists)
                        farthest = max(dists)
                        bL = lengths[dists.index(farthest)]
                        near = farthest < bL
                    else:
                        near = False
                    
                    if not any(isIntersect):
                        cP.append(br)
                        rA.append(bRotate + rotV)
                        bN.append(near)
                
                #print(bN)
                
                if len(cP) == 1:
                    for i, br in enumerate(cP):
                        childP.append(br)
                        rot_a.append(rA[i])
                else:
                    nearcP = []
                    nearrA = []
                    for i, near in enumerate(bN):
                        if near:
                            nearcP.append(cP[i])
                            nearrA.append(rA[i])
                            #childP.append(cP[i])
                            #rot_a.append(rA[i])
                        else:
                            childP.append(cP[i])
                            rot_a.append(rA[i])
                        
                    if len(nearcP) > 0:
                        i = choice(list(range(len(nearcP))))
                        childP.append(nearcP[i])
                        rot_a.append(nearrA[i])
                
                oldRotate += rotate[n]
                
            else:
                idx = randint(0, len(p)-1)
                childP.append(p[idx])

        childP.extend(childP_L)
        rot_a.extend([0] * len(childP_L))

        oldRotate = 0
    
    for i, p in enumerate(childP):
        # Add a spline and set the coordinate of the first point.
        newSpline = cu.splines.new('BEZIER')
        newSpline.material_index = matIndex[n]
        newPoint = newSpline.bezier_points[-1]
        newPoint.co = p.co
        tempPos = zAxis.copy()
        # If the -ve flag for downAngle is used we need a special formula to find it
        if useOldDownAngle:
            if downAngleV[n] < 0.0:
                downV = downAngleV[n] * (1 - 2 * (.2 + .8 * ((1 - p.offset) / (1 - baseSize))))
            # Otherwise just find a random value
            else:
                downV = uniform(-downAngleV[n], downAngleV[n])
        else:
            if downAngleV[n] < 0.0:
                downV = uniform(-downAngleV[n], downAngleV[n])
            else:
                downV = -downAngleV[n] * (1 - (1 - p.stemOffset) / (1 - baseSize)) ** 2 #(110, 80) = (60, -50)
        
        if p.offset == 1:
            downRotMat = Matrix.Rotation(0, 3, 'X')
        else:
            downRotMat = Matrix.Rotation(downAngle[n] + downV, 3, 'X')
        
        # If the -ve flag for rotate is used we need to find which side of the stem the last child point was and then grow in the opposite direction.
        if rotate[n] < 0.0:
            oldRotate = -copysign(rotate[n], oldRotate)
        # Otherwise just generate a random number in the specified range
        else:
            oldRotate += rotate[n]
        bRotate = oldRotate + uniform(-rotateV[n], rotateV[n])
        
        if (n == 1) and  (rMode in ["rotate", 'distance']):
            bRotate = rot_a[i]
            
        rotMat = Matrix.Rotation(bRotate, 3, 'Z')
        
        # Rotate the direction of growth and set the new point coordinates
        tempPos.rotate(downRotMat)
        tempPos.rotate(rotMat)
        
        #use quat angle
        if (n == 1) and (p.offset != 1):
            if useParentAngle:
                tempPos.rotate(convertQuat(p.quat))
        else:
            tempPos.rotate(p.quat)
        
        newPoint.handle_right = p.co + tempPos * 0.33
        
        # Find branch length and the number of child stems.
        maxbL = scaleVal
        for l in length[:n+1]:
            maxbL *= l
        lMax = length[n] * uniform(1 - lengthV[n], 1 + lengthV[n])
        if n == 1:
            lShape = shapeRatio(shape, (1 - p.stemOffset) / (1 - baseSize), custom=customShape)
            tShape = shapeRatio(shape, 0, custom=customShape)
        else:
            lShape = shapeRatio(shapeS, (1 - p.stemOffset) / (1 - baseSize))
            tShape = shapeRatio(shapeS, 0)
        branchL = p.lengthPar * lMax * lShape
        childStems = branches[min(3, n + 1)] * (0.1 + 0.9 * (branchL / maxbL))
        
        # If this is the last level before leaves then we need to generate the child points differently
        if (storeN == levels - 1):
            if leafType == '4':
                childStems = 0 #False
            else:
                childStems = leaves * (0.1 + 0.9 * (branchL / maxbL)) * shapeRatio(leafDist, (1 - p.offset))

        # Determine the starting and ending radii of the stem using the tapering of the stem
        #startRad = min((p.radiusPar[0] * ((branchL / p.lengthPar) ** ratioPower)) * radiusTweak[n], 10)
        #startRad = min((ratio * p.lengthPar * ((branchL / p.lengthPar) ** ratioPower)) * radiusTweak[n], 10)#p.radiusPar[1]
        
        #ratio = (p.radiusPar[0] - p.radiusPar[2]) / p.lengthPar
        #startRad = min(((ratio * branchL) ** ratioPower) * radiusTweak[n], p.radiusPar[1])#p.radiusPar[1] #10
        
        startRad = min(((p.radiusPar[2] * (1/tShape) * lShape) ** ratioPower) * radiusTweak[n], p.radiusPar[1])
        
        #p.radiusPar[0] is parent start radius
        #p.radiusPar[1] is parent radius
        #p.radiusPar[2] is parent end radius
        
        if p.offset == 1:
            startRad = p.radiusPar[1]
        endRad = (startRad * (1 - taper[n])) ** ratioPower
        startRad = max(startRad, minRadius)
        endRad = max(endRad, minRadius)
        newPoint.radius = startRad

        # stem curvature
        curveVal = curve[n] / curveRes[n]

        # Add the new stem to list of stems to grow and define which bone it will be parented to
        nstem = stemSpline(newSpline, curveVal, curveV[n], attractUp[n], 0, curveRes[n], branchL / curveRes[n], childStems, 
                           startRad, endRad, len(cu.splines) - 1, 0, p.quat)
        if (n == 1) and (p.offset == 1):
            nstem.isFirstTip = True
        addstem(nstem)

        bone = roundBone(p.parBone, boneStep[n-1])
        if p.offset == 1:
            isend = True
        else:
            isend = False
        addsplinetobone((bone, isend))


def grow_branch_level(baseSize, baseSplits, childP, cu, curve, curveBack, curveRes, handles, n, levels, branches, scaleVal,
                  segSplits, splineToBone, splitAngle, splitAngleV, st, branchDist, length, splitByLen, closeTip,
                  splitRadiusRatio, minRadius, nrings, splitBias, splitHeight, attractOut, rMode, splitStraight,
                  splitLength, lengthV, taperCrown, noTip, boneStep, rotate, rotateV, leaves, leafType, attachment, matIndex):
    # Initialise the spline list of split stems in the current branch
    splineList = [st]
    # For each of the segments of the stem which must be grown we have to add to each spline in splineList
    for k in range(curveRes[n]):
        # Make a copy of the current list to avoid continually adding to the list we're iterating over
        tempList = splineList[:]

        #for curve variation
        if curveRes[n] > 1:
            kp = (k / (curveRes[n] - 1)) # * 2
        else:
            kp = 1.0

        #split bias
        splitValue = segSplits[n]
        if n == 0:
            splitValue = ((2 * splitBias) * (kp - .5) + 1) * splitValue
            splitValue = max(splitValue, 0.0)

        # For each of the splines in this list set the number of splits and then grow it
        for spl in tempList:

            #adjust numSplit #this is not perfect, but it's good enough and not worth improving
            lastsplit = getattr(spl, 'splitlast', 0)
            splitVal = splitValue
            if lastsplit == 0:
                splitVal = splitValue ** 0.5 # * 1.33
            elif lastsplit == 1:
                splitVal = splitValue * splitValue

            if k == 0:
                numSplit = 0
            elif (k == 1) and (n == 0):
                numSplit = baseSplits
            elif (n == 0) and (k == int((curveRes[n]) * splitHeight)) and (splitVal > 0): #always split at splitHeight
                numSplit = 1
            elif (n == 0) and (k < ((curveRes[n]) * splitHeight)) and (k != 1): #splitHeight
                numSplit = 0
            else:
                if (n >= 0) and splitByLen:
                    L = ((spl.segL * curveRes[n]) / scaleVal)
                    lf = 1
                    for l in length[:n+1]:
                        lf *= l
                    L = L / lf
                    numSplit = splits2(splitVal * L)
                else:
                    numSplit = splits2(splitVal)

            if (k == int(curveRes[n] / 2 + 0.5)) and (curveBack[n] != 0):
                spl.curv += 2 * (curveBack[n] / curveRes[n]) #was -4 * 
                
            growSpline(n, spl, numSplit, splitAngle[n], splitAngleV[n], splitStraight, splineList, handles, splineToBone,
                       closeTip, splitRadiusRatio, minRadius, kp, splitHeight, attractOut[n], splitLength, lengthV[n], taperCrown, boneStep, rotate, rotateV, matIndex)

    # Sprout child points to grow the next splines or leaves
    if (n == 0) and (rMode == 'rotate'):
        tVals = findChildPoints2(st.children)
    elif (n == 0) and (rMode == 'distance'):
        tVals = findChildPoints3(splineList, st.children, rp=.25) #degrees(rotateV[3])
    
    elif ((n > 0) and (n != levels - 1) and (attachment == "1")) or ((n == levels - 1) and (leafType in ['1', '3'])): # oppositely attached leaves and branches
        tVal = findChildPoints(splineList, ceil(st.children / 2))
        tVals = []
        for t in tVal[:-1]:
            tVals.extend([t, t])
        if (n == levels - 1) and ((leaves / 2) == (leaves // 2)):
            # put two leaves at branch tip if leaves is even
            tVals.extend([1, 1])
        else:
            tVals.append(1)
    else:
        tVals = findChildPoints(splineList, st.children)

    if 1 not in tVals:
        tVals.append(1.0)
    if (n != levels - 1) and (branches[min(3, n+1)] == 0):
        tVals = []

    if (n < levels - 1) and noTip:
        tVals = tVals[:-1]

    # remove some of the points because of baseSize
    tVals = [t for t in tVals if t > baseSize]

    #grow branches in rings/whorls
    if (n == 0) and (nrings > 0):
        tVals = [(floor(t * nrings) / nrings) * uniform(.999, 1.001) for t in tVals[:-1]]
        if not noTip:
            tVals.append(1)
        tVals = [t for t in tVals if t > baseSize]

    #branch distribution
    if n == 0:
        tVals = [((t - baseSize) / (1 - baseSize)) for t in tVals]
        if branchDist <= 1.0:
            tVals = [t ** (1 / branchDist) for t in tVals]
        else:
            #tVals = [1 - (1 - t) ** branchDist for t in tVals]
            tVals = [1 - t for t in tVals]
            p = ((1/.5 ** branchDist) - 1) ** 2
            tVals = [(p ** t - 1) / (p-1) for t in tVals]
            tVals = [1 - t for t in tVals]
        tVals = [t * (1 - baseSize) + baseSize for t in tVals]

    # For all the splines, we interpolate them and add the new points to the list of child points
    maxOffset = max([s.offsetLen + (len(s.spline.bezier_points) - 1) * s.segL for s in splineList])
    for s in splineList:
        #print(str(n)+'level: ', s.segMax*s.segL)
        childP.extend(interpStem(s, tVals, maxOffset, baseSize))

    return splineToBone


#calculate taper automatically
def findtaper(length, taper, shape, shapeS, levels, customShape):
    taperS = []
    for i, t in enumerate(length):
        if i == 0:
            shp = 1.0
        elif i == 1:
            shp = shapeRatio(shape, 0, custom=customShape)
        else:
            shp = shapeRatio(shapeS, 0)
        t = t * shp
        taperS.append(t)

    taperP = []
    for i, t in enumerate(taperS):
        pm = 1
        for x in range(i+1):
            pm *= taperS[x]
        taperP.append(pm)

    taperR = []
    for i, t in enumerate(taperP):
        t = sum(taperP[i:levels])
        taperR.append(t)

    taperT = []
    for i, t in enumerate(taperR):
        try:
            t = taperP[i] / taperR[i]
        except ZeroDivisionError:
            t = 1.0
        taperT.append(t)
    
    taperT = [t * taper[i] for i, t in enumerate(taperT)]
    
    return taperT

def leafRot(leafObjY, leafObjZ):
    def tovector(ax):
        vec = [0, 0, 0]
        a = int(ax[1])
        s = ax[0]
        if s == '+':
            s = 1
        else:
            s = -1
        vec[a] = s
        return Vector(vec)

    yvec = tovector(leafObjY)
    zvec = tovector(leafObjZ)

    xvec = zvec.cross(yvec)

    if zvec[2] in [1, -1]:
        xvec *= -1
    elif xvec[2] in [1, -1]:
        zvec *= -1
        xvec *= -1
    else:
        zvec *= -1
        yvec *= -1
        xvec *= -1

    m = Matrix([xvec, yvec, zvec])
    m = m.to_euler()
    return m


def addTree(props):
    global splitError
    #startTime = time.time()
    # Set the seed for repeatable results
    rseed = props.seed
    seed(rseed)
    
    # Set all other variables
    levels = props.levels#
    length = props.length#
    lengthV = props.lengthV#
    taperCrown = props.taperCrown
    branches = props.branches#
    curveRes = props.curveRes#
    curve = toRad(props.curve)#
    curveV = toRad(props.curveV)#
    curveBack = toRad(props.curveBack)#
    baseSplits = props.baseSplits#
    segSplits = props.segSplits#
    splitByLen = props.splitByLen
    rMode = props.rMode
    splitStraight = props.splitStraight
    splitLength = props.splitLength
    splitAngle = toRad(props.splitAngle)#
    splitAngleV = toRad(props.splitAngleV)#
    scale = props.scale#
    scaleV = props.scaleV#
    attractUp = props.attractUp#
    attractOut = props.attractOut
    shape = int(props.shape)#
    shapeS = int(props.shapeS)#
    customShape = props.customShape
    branchDist = props.branchDist
    nrings = props.nrings
    baseSize = props.baseSize
    baseSize_s = props.baseSize_s
    leafBaseSize = props.leafBaseSize
    splitHeight = props.splitHeight
    splitBias = props.splitBias
    ratio = props.ratio
    minRadius = props.minRadius
    closeTip = props.closeTip
    rootFlare = props.rootFlare
    splitRadiusRatio = props.splitRadiusRatio
    autoTaper = props.autoTaper
    taper = props.taper#
    noTip = props.noTip
    radiusTweak = props.radiusTweak
    ratioPower = props.ratioPower#
    downAngle = toRad(props.downAngle)#
    downAngleV = toRad(props.downAngleV)#
    rotate = toRad(props.rotate)#
    rotateV = toRad(props.rotateV)#
    scale0 = props.scale0#
    scaleV0 = props.scaleV0#
    attachment = props.attachment
    leafType = props.leafType
    leafDownAngle = radians(props.leafDownAngle)
    leafDownAngleV = radians(props.leafDownAngleV)
    leafRotate = radians(props.leafRotate)
    leafRotateV = radians(props.leafRotateV)
    leafScale = props.leafScale#
    leafScaleX = props.leafScaleX#
    leafScaleT = props.leafScaleT
    leafScaleV = props.leafScaleV
    leafShape = props.leafShape
    leafDupliObj = props.leafDupliObj
    leafangle = props.leafangle
    leafDist = int(props.leafDist)#
    bevelRes = props.bevelRes#
    resU = props.resU#
    
    #leafObjX = props.leafObjX
    leafObjY = props.leafObjY
    leafObjZ = props.leafObjZ
    
    useArm = props.useArm
    previewArm = props.previewArm
    armAnim = props.armAnim
    leafAnim = props.leafAnim
    frameRate = props.frameRate
    loopFrames = props.loopFrames
    
    #windSpeed = props.windSpeed
    #windGust = props.windGust
    
    wind = props.wind 
    gust = props.gust 
    gustF = props.gustF
    
    af1 = props.af1
    af2 = props.af2
    af3 = props.af3
    
    makeMesh = props.makeMesh
    armLevels = props.armLevels
    boneStep = props.boneStep
    matIndex = props.matIndex
    
    useOldDownAngle = props.useOldDownAngle
    useParentAngle = props.useParentAngle
    
    if not makeMesh:
        boneStep = [1, 1, 1, 1]
    
    #taper
    if autoTaper:
        taper = findtaper(length, taper, shape, shapeS, levels, customShape)
    
    leafObj = None
    
    leafObjRot = leafRot(leafObjY, leafObjZ)
    
    # Some effects can be turned ON and OFF, the necessary variables are changed here
    if not props.bevel:
        bevelDepth = 0.0
    else:
        bevelDepth = 1.0

    if not props.showLeaves:
        leaves = 0
    else:
        leaves = props.leaves

    if props.handleType == '0':
        handles = 'AUTO'
    else:
        handles = 'VECTOR'

    for ob in bpy.data.objects:
        ob.select_set(state=False)

    # Initialise the tree object and curve and adjust the settings
    cu = bpy.data.curves.new('tree', 'CURVE')
    treeOb = bpy.data.objects.new('tree', cu)
    bpy.context.scene.collection.objects.link(treeOb)
    if not useArm:
        treeOb.location=bpy.context.scene.cursor.location

    cu.dimensions = '3D'
    cu.fill_mode = 'FULL'
    cu.bevel_depth = bevelDepth
    cu.bevel_resolution = bevelRes
    #cu.use_uv_as_generated = True # removed 2.82
    
    #material slots
    for i in range(max(matIndex)+1):
        treeOb.data.materials.append(None)

    # Fix the scale of the tree now
    if rseed == 0: #first tree is always average size
        scaleV = 0
    scaleVal = scale + uniform(-scaleV, scaleV)
    scaleVal += copysign(1e-6, scaleVal)  # Move away from zero to avoid div by zero
    
    childP = []
    stemList = []

    levelCount = []
    splineToBone = deque([''])
    addsplinetobone = splineToBone.append

    # Each of the levels needed by the user we grow all the splines
    for n in range(levels):
        storeN = n
        stemList = deque()
        addstem = stemList.append
        # If n is used as an index to access parameters for the tree it must be at most 3 or it will reference outside the array index
        n = min(3, n)
        splitError = 0.0
        
        #closeTip only on last level
        closeTipp = all([(n == levels-1), closeTip])
        
        # If this is the first level of growth (the trunk) then we need some special work to begin the tree
        if n == 0:
            kickstart_trunk(addstem, levels, leaves, branches, cu, downAngle, downAngleV, curve, curveRes, curveV, attractUp,
                            length,lengthV,ratio, ratioPower, scale0, scaleV0, scaleVal, taper, minRadius, rootFlare, matIndex)
        # If this isn't the trunk then we may have multiple stem to initialize
        else:
            # For each of the points defined in the list of stem starting points we need to grow a stem.
            fabricate_stems(addsplinetobone, addstem, baseSize, branches, childP, cu, curve,
                            curveRes, curveV, attractUp, downAngle, downAngleV, leafDist, leaves, leafType, length, lengthV,
                            levels, n, ratio, ratioPower, rotate, rotateV, scaleVal, shape, storeN,
                            taper, shapeS, minRadius, radiusTweak, customShape, rMode, segSplits,
                            useOldDownAngle, useParentAngle, boneStep, matIndex)
        
        #change base size for each level
        if n > 0:
            baseSize = baseSize_s
        if (n == levels - 1):
            baseSize = leafBaseSize

        childP = []
        # Now grow each of the stems in the list of those to be extended
        for st in stemList:
            splineToBone = grow_branch_level(baseSize, baseSplits, childP, cu, curve, curveBack, curveRes, handles, n, levels,
                                         branches, scaleVal, segSplits, splineToBone, splitAngle, splitAngleV, st, branchDist,
                                         length, splitByLen, closeTipp, splitRadiusRatio, minRadius, nrings, splitBias,
                                         splitHeight, attractOut, rMode, splitStraight, splitLength, lengthV, taperCrown,
                                         noTip, boneStep, rotate, rotateV, leaves, leafType, attachment, matIndex)

        levelCount.append(len(cu.splines))
    
    # Set curve resolution
    cu.resolution_u = resU
    
    # If we need to add leaves, we do it here
    leafVerts = []
    leafFaces = []
    leafNormals = []
    
    leafMesh = None # in case we aren't creating leaves, we'll still have the variable
    
    leafP = []
    if leaves:
        oldRot = 0.0
        n = min(3, n+1)
        # For each of the child points we add leaves
        for ln, cp in enumerate(childP):
            # If the special flag is set then we need to add several leaves at the same location
            if leafType == '4':
                oldRot = -leafRotate / 2
                for g in range(abs(leaves)):
                    (vertTemp, faceTemp, normal, oldRot) = genLeafMesh(leafScale, leafScaleX, leafScaleT, leafScaleV, cp.co, cp.quat, cp.offset,
                                                                       len(leafVerts), leafDownAngle, leafDownAngleV, leafRotate, leafRotateV,
                                                                       oldRot, leaves, leafShape, leafangle, leafType, ln, leafObjRot)
                    leafVerts.extend(vertTemp)
                    leafFaces.extend(faceTemp)
                    leafNormals.extend(normal)
                    leafP.append(cp)
            # Otherwise just add the leaves like splines.
            else:
                (vertTemp, faceTemp, normal, oldRot) = genLeafMesh(leafScale, leafScaleX, leafScaleT, leafScaleV, cp.co, cp.quat, cp.offset,
                                                                   len(leafVerts), leafDownAngle, leafDownAngleV, leafRotate, leafRotateV,
                                                                   oldRot, leaves, leafShape, leafangle, leafType, ln, leafObjRot)
                leafVerts.extend(vertTemp)
                leafFaces.extend(faceTemp)
                leafNormals.extend(normal)
                leafP.append(cp)
        
        # Create the leaf mesh and object, add geometry using from_pydata, edges are currently added by validating the mesh which isn't great
        leafMesh = bpy.data.meshes.new('leaves')
        leafObj = bpy.data.objects.new('leaves', leafMesh)
        bpy.context.scene.collection.objects.link(leafObj)
        leafObj.parent = treeOb
        leafMesh.from_pydata(leafVerts, (), leafFaces)

        #set vertex normals for dupliVerts
        if leafShape == 'dVert':
            leafMesh.vertices.foreach_set('normal', leafNormals)

        # enable duplication
        if leafShape == 'dFace':
            leafObj.instance_type = "FACES"
            leafObj.use_instance_faces_scale = True
            leafObj.instance_faces_scale = 10.0
            try:
                bpy.data.objects[leafDupliObj].parent = leafObj
            except KeyError:
                pass
        elif leafShape == 'dVert':
            leafObj.instance_type = "VERTS"
            leafObj.use_instance_vertices_rotation = True
            try:
                bpy.data.objects[leafDupliObj].parent = leafObj
            except KeyError:
                pass

        #add leaf UVs
        if leafShape == 'rect':
            leafMesh.uv_layers.new(name="leafUV")
            uvlayer = leafMesh.uv_layers.active.data

            u1 = .5 * (1 - leafScaleX)
            u2 = 1 - u1

            for i in range(0, len(leafFaces)):
                uvlayer[i*4 + 0].uv = Vector((u2, 0))
                uvlayer[i*4 + 1].uv = Vector((u2, 1))
                uvlayer[i*4 + 2].uv = Vector((u1, 1))
                uvlayer[i*4 + 3].uv = Vector((u1, 0))

        elif leafShape == 'hex':
            leafMesh.uv_layers.new(name="leafUV")
            uvlayer = leafMesh.uv_layers.active.data

            u1 = .5 * (1 - leafScaleX)
            u2 = 1 - u1

            for i in range(0, int(len(leafFaces) / 2)):
                uvlayer[i*8 + 0].uv = Vector((.5, 0))
                uvlayer[i*8 + 1].uv = Vector((u1, 1/3))
                uvlayer[i*8 + 2].uv = Vector((u1, 2/3))
                uvlayer[i*8 + 3].uv = Vector((.5, 1))

                uvlayer[i*8 + 4].uv = Vector((.5, 0))
                uvlayer[i*8 + 5].uv = Vector((.5, 1))
                uvlayer[i*8 + 6].uv = Vector((u2, 2/3))
                uvlayer[i*8 + 7].uv = Vector((u2, 1/3))

        leafMesh.validate()
    
    leafVertSize = {'hex': 6, 'rect': 4, 'dFace': 4, 'dVert': 1}[leafShape]
    
    armLevels = min(armLevels, levels)
    armLevels -= 1
    
    # unpack vars from splineToBone
    splineToBone1 = splineToBone
    splineToBone = [s[0] if len(s) > 1 else s for s in splineToBone1]
    isend = [s[1] if len(s) > 1 else False for s in splineToBone1]
    issplit = [s[2] if len(s) > 2 else False for s in splineToBone1]
    splitPidx = [s[3] if len(s) > 2 else 0 for s in splineToBone1]
    
    # add mesh object
    treeObj = None
    if makeMesh:
        treeMesh = bpy.data.meshes.new('treemesh')
        treeObj = bpy.data.objects.new('treemesh', treeMesh)
        bpy.context.scene.collection.objects.link(treeObj)
        if not useArm:
            treeObj.location=bpy.context.scene.cursor.location
    
    # If we need an armature we add it
    if useArm:
        # Create the armature and objects
        armOb = create_armature(armAnim, leafP, cu, frameRate, leafMesh, leafObj, leafVertSize, leaves, levelCount, splineToBone,
                        treeOb, treeObj, wind, gust, gustF, af1, af2, af3, leafAnim, loopFrames, previewArm, armLevels, makeMesh, boneStep)
    
    #print(time.time()-startTime)

    #mesh branches
    if makeMesh:
        t1 = time.time()

        treeVerts = []
        treeEdges = []
        root_vert = []
        vert_radius = []
        vertexGroups = OrderedDict()
        lastVerts = []
        
        #vertex group for each level
        levelGroups = []
        for n in range(levels):
            treeObj.vertex_groups.new(name="Branching Level " + str(n))
            levelGroups.append([])

        for i, curve in enumerate(cu.splines):
            points = curve.bezier_points
            
            #find branching level
            level = 0
            for l, c in enumerate(levelCount):
                if i < c:
                    level = l
                    break
            level = min(level, 3)
            
            step = boneStep[level]
            vindex = len(treeVerts)
            
            p1 = points[0]
            
            #add extra vertex for splits
            if issplit[i]:
                pb = int(splineToBone[i][4:-4])
                pn = splitPidx[i] #int(splineToBone[i][-3:])
                p_1 = cu.splines[pb].bezier_points[pn]
                p_2 = cu.splines[pb].bezier_points[pn+1]
                p = evalBez(p_1.co, p_1.handle_right, p_2.handle_left, p_2.co, 1 - 1/(resU + 1))
                treeVerts.append(p)
                
                root_vert.append(False)
                vert_radius.append((p1.radius * .75, p1.radius * .75))
                treeEdges.append([vindex,vindex+1])
                vindex += 1
            
            if isend[i]:
                parent = lastVerts[int(splineToBone[i][4:-4])]
                vindex -= 1
            else:
                #add first point
                treeVerts.append(p1.co)
                root_vert.append(True)
                vert_radius.append((p1.radius, p1.radius))

            #dont make vertex group if above armLevels
            if (i >= levelCount[armLevels]):
                idx = i
                groupName = splineToBone[idx]
                g = True
                while groupName not in vertexGroups:
                    #find parent bone of parent bone
                    b = splineToBone[idx]
                    idx = int(b[4:-4])
                    groupName = splineToBone[idx]
            else:
                g = False

            for n, p2 in enumerate(points[1:]):
                if not g:
                    groupName = 'bone' + (str(i)).rjust(3, '0') + '.' + (str(n)).rjust(3, '0')
                    groupName = roundBone(groupName, step)
                    if groupName not in vertexGroups:
                        vertexGroups[groupName] = []
                
                # parent first vert in split to parent branch bone
                if issplit[i] and n == 0:
                    if g:
                        vertexGroups[groupName].append(vindex - 1)
                    else:
                        vertexGroups[splineToBone[i]].append(vindex - 1)
                    levelGroups[level].append(vindex - 1)

                for f in range(1, resU+1):
                    pos = f / resU
                    p = evalBez(p1.co, p1.handle_right, p2.handle_left, p2.co, pos)
                    radius = p1.radius + (p2.radius - p1.radius) * pos

                    treeVerts.append(p)
                    root_vert.append(False)
                    vert_radius.append((radius, radius))
                    
                    if (isend[i]) and (n == 0) and (f == 1):
                        edge = [parent, n * resU + f + vindex]
                    else:
                        edge = [n * resU + f + vindex - 1, n * resU + f + vindex]
                        #add vert to group
                        vertexGroups[groupName].append(n * resU + f + vindex - 1)
                        levelGroups[level].append(n * resU + f + vindex - 1)
                    treeEdges.append(edge)
                    
                vertexGroups[groupName].append(n * resU + resU + vindex)
                levelGroups[level].append(n * resU + resU + vindex)

                p1 = p2
                
            lastVerts.append(len(treeVerts)-1)

        treeMesh.from_pydata(treeVerts, treeEdges, ())
        
        if useArm:
            for group in vertexGroups:
                treeObj.vertex_groups.new(name=group)
                treeObj.vertex_groups[group].add(vertexGroups[group], 1.0, 'ADD')
        
        for i, g in enumerate(levelGroups):
            treeObj.vertex_groups["Branching Level " + str(i)].add(g, 1.0, 'ADD')

        #add armature
        if useArm:
            armMod = treeObj.modifiers.new('windSway', 'ARMATURE')
            if previewArm:
                armOb.hide_viewport = True
                armOb.data.display_type = 'STICK'
            armMod.object = armOb
            armMod.use_bone_envelopes = False
            armMod.use_vertex_groups = True

        #add skin modifier and set data
        skinMod = treeObj.modifiers.new('Skin', 'SKIN')
        skinMod.use_smooth_shade = True
        if previewArm:
            skinMod.show_viewport = False
        skindata = treeObj.data.skin_vertices[0].data
        for i, radius in enumerate(vert_radius):
            skindata[i].radius = radius
            skindata[i].use_root = root_vert[i]

        print("mesh time", time.time() - t1)
