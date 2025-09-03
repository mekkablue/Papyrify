#MenuTitle: Papyrify
# -*- coding: utf-8 -*-
from __future__ import division, print_function, unicode_literals
__doc__="""
Applies Papyrus effect.
"""

import vanilla, sys
from AppKit import NSPoint
from random import random
from math import sqrt, atan2, cos, sin, pi, degrees, acos
from copy import copy
# from GlyphsApp import GSNode, GSLayer, GSPath, GSPathSegment


Glyphs.clearLog()


def calculateIntersection(A, B, C, D):
	x1, y1 = A
	x2, y2 = B
	x3, y3 = C
	x4, y4 = D
	
	denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
	if denom == 0:
		return None  # Parallel lines, no intersection
	
	ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
	x = x1 + ua * (x2-x1)
	y = y1 + ua * (y2-y1)
	
	return (x, y)


def isNormalAboveAD(A, D, HI):
	ax, ay = A
	dx, dy = D
	hix, hiy = HI
	
	adLength = sqrt((dx-ax)**2 + (dy-ay)**2)
	if adLength == 0:
		return False
	
	projectionLength = ((hix-ax)*(dx-ax) + (hiy-ay)*(dy-ay)) / adLength
	
	return 0 <= projectionLength <= adLength


def calculateNewHI(A, D, HI):
	ax, ay = A
	dx, dy = D
	hix, hiy = HI
	
	midX, midY = (ax + dx) / 2, (ay + dy) / 2
	
	adLength = sqrt((dx-ax)**2 + (dy-ay)**2)
	normalX, normalY = (ay-dy) / adLength, (dx-ax) / adLength
	
	distanceHIAD = abs((dx-ax)*(ay-hiy) - (dy-ay)*(ax-hix)) / adLength
	
	# Determine the side of the original HI
	originalSide = (hix-ax)*(dy-ay) - (hiy-ay)*(dx-ax)
	
	# Limit the distance to at most the length of AD
	distanceHIAD = min(distanceHIAD, adLength)
	
	newHIx = midX + distanceHIAD * normalX
	newHIy = midY + distanceHIAD * normalY
	
	# Check if the new HI is on the same side as the original
	newSide = (newHIx-ax)*(dy-ay) - (newHIy-ay)*(dx-ax)
	
	# If not on the same side, flip the direction
	if originalSide * newSide < 0:
		newHIx = midX - distanceHIAD * normalX
		newHIy = midY - distanceHIAD * normalY
	
	return (newHIx, newHIy)


def setNewPositions(A, D, HI, factor):
	ax, ay = A
	dx, dy = D
	hix, hiy = HI
	
	distanceAHI = sqrt((hix-ax)**2 + (hiy-ay)**2)
	distanceDHI = sqrt((hix-dx)**2 + (hiy-dy)**2)
	
	if distanceAHI != 0:
		bx = ax + factor * distanceAHI * (hix-ax) / distanceAHI
		by = ay + factor * distanceAHI * (hiy-ay) / distanceAHI
	else:
		bx, by = ax, ay
	
	if distanceDHI != 0:
		cx = dx + factor * distanceDHI * (hix-dx) / distanceDHI
		cy = dy + factor * distanceDHI * (hiy-dy) / distanceDHI
	else:
		cx, cy = dx, dy
	
	return (bx, by), (cx, cy)


def correctHandles(A, B, C, D, factor=0.4):
	HI = calculateIntersection(A, B, C, D)
	
	if HI is None:
		return None  # No intersection found
	
	if A==D:
		return None
	
	if not isNormalAboveAD(A, D, HI):
		HI = calculateNewHI(A, D, HI)
		newB, newC = setNewPositions(A, D, HI, factor)
		return newB, newC
	else:
		return None
		#return B, C  # Return original values if HI is already normal


def bringBackAcuteNode(n1, n2, n3, thresholdAngle=120, amount=0.7):
	# Calculate vectors
	v1 = (n2.x - n1.x, n2.y - n1.y)
	v2 = (n3.x - n2.x, n3.y - n2.y)
	
	# Calculate dot product and magnitudes
	dotProduct = v1[0] * v2[0] + v1[1] * v2[1]
	mag1 = sqrt(v1[0]**2 + v1[1]**2)
	mag2 = sqrt(v2[0]**2 + v2[1]**2)
	
	# Calculate angle in degrees
	if mag1 * mag2 == 0:
		angle = 0  # Handle case where vectors have zero length
	else:
		cosAngle = dotProduct / (mag1 * mag2)
		cosAngle = max(-1, min(1, cosAngle))  # Ensure value is in [-1, 1] range
		angle = degrees(acos(cosAngle))
	
	# Check if angle is greater than threshold
	if angle > thresholdAngle:
		# Calculate new position
		newX = n2.x + (((n1.x + n3.x) / 2) - n2.x) * amount
		newY = n2.y + (((n1.y + n3.y) / 2) - n2.y) * amount
		
		# Update n2 position
		n2.x = newX
		n2.y = newY


def isOnLine(p1, p2, p3, threshold=0.9):
	# Calculate vectors
	v1 = (p3[0] - p1[0], p3[1] - p1[1])
	v2 = (p2[0] - p1[0], p2[1] - p1[1])
	
	# Calculate cross product
	crossProduct = abs(v1[0] * v2[1] - v1[1] * v2[0])
	
	# Calculate vector lengths
	v1Length = sqrt(v1[0]**2 + v1[1]**2)
	v2Length = sqrt(v2[0]**2 + v2[1]**2)
	
	# Check if p2 is on or near the line
	if v1Length == 0 or v2Length == 0:
		return p1 == p2 or p2 == p3  # If any vector has zero length, check if points are identical
	
	if crossProduct / (v1Length * v2Length) <= threshold:
		# Check if p2 is between p1 and p3
		dotProduct = v1[0] * v2[0] + v1[1] * v2[1]
		if 0 <= dotProduct <= v1Length**2:
			return True
	
	return False


def handlePositionsForLine(p1, p2):
	# Calculate the vector from p1 to p2
	dx = p2.x - p1.x
	dy = p2.y - p1.y
	
	# Calculate h1 (1/3 of the distance)
	h1X = p1.x + dx / 3
	h1Y = p1.y + dy / 3
	h1 = NSPoint(h1X, h1Y)
	
	# Calculate h2 (2/3 of the distance)
	h2X = p1.x + 2 * dx / 3
	h2Y = p1.y + 2 * dy / 3
	h2 = NSPoint(h2X, h2Y)
	
	return h1, h2


def subdivideBezier(aPoint, bPoint, cPoint, dPoint, minLength, maxLength):
	# Calculate the length of the original curve
	originalLength = calculateCurveLength(aPoint, bPoint, cPoint, dPoint)

	# Initialize the list to store the subdivided curve points
	subdividedPoints = []

	# Add the start point of the curve
	subdividedPoints.append(aPoint)

	# Perform the subdivision
	remainingLength = originalLength
	b1, c1, d1 = bPoint, cPoint, dPoint
	while remainingLength > (minLength+maxLength)/1.5:
		# Determine the length of the next segment
		segmentLength = min(minLength + random()*(maxLength - minLength), remainingLength)

		# Calculate the parameter t for the current segment
		t = segmentLength / remainingLength

		# Calculate the points for the current subdivision
		a0, b0, c0, d0, b1, c1, d1 = divideCurve(aPoint, bPoint, cPoint, dPoint, t)

		# Add the points for this segment (excluding the first point which is shared)
		subdividedPoints.extend([b0, c0, d0])

		# Update the points for the next iteration
		aPoint, bPoint, cPoint, dPoint = d0, b1, c1, d1

		# Update the remaining length
		remainingLength = calculateCurveLength(aPoint, bPoint, cPoint, dPoint)
	else:
		subdividedPoints.extend([b1, c1, d1])

	return subdividedPoints


def subdivideLine(aPoint, bPoint, minLength, maxLength):
	# Calculate the distance between the two points
	distance = ((bPoint[0] - aPoint[0])**2 + (bPoint[1] - aPoint[1])**2)**0.5

	# Initialize the list to store the subdivided line points
	subdividedPoints = []

	# Add the start point of the line
	subdividedPoints.append(aPoint)

	# Calculate the direction vector of the line segment
	dx = (bPoint[0] - aPoint[0]) / max(5.0, distance)
	dy = (bPoint[1] - aPoint[1]) / max(5.0, distance)

	# Perform the subdivision
	remainingDistance = distance
	while remainingDistance > 0:
		# Determine the length of the next segment
		segmentLength = min(minLength + random()*(maxLength - minLength), remainingDistance)

		# Calculate the new point along the line segment
		x = aPoint[0] + segmentLength * dx
		y = aPoint[1] + segmentLength * dy

		# Update the points for the next iteration
		aPoint = (x, y)

		# Update the remaining distance
		remainingDistance -= segmentLength

		# Add the new point to the list
		subdividedPoints.append((x, y))

	return subdividedPoints


def calculateCurveLength(aPoint, bPoint, cPoint, dPoint, numSamples=500):
	# Approximate the length of a cubic Bezier curve
	length = 0
	prevPoint = bezierPoint(aPoint, bPoint, cPoint, dPoint, 0)

	for t in [i / numSamples for i in range(1, numSamples + 1)]:
		point = bezierPoint(aPoint, bPoint, cPoint, dPoint, t)
		length += distance(prevPoint, point)
		prevPoint = point

	return length


def calculateAngle(p1, p2):
	return atan2(p2.y - p1.y, p2.x - p1.x)


def getPerpendicularPoint(p1, p2, distance):
	a = calculateAngle(p1, p2)
	return NSPoint(p1.x - distance * sin(a),
		p1.y + distance * cos(a))


def bezierPoint(aPoint, bPoint, cPoint, dPoint, t):
	# Evaluate a cubic Bezier curve at parameter t
	return (
		(1-t)**3 * aPoint[0] + 3 * (1-t)**2 * t * bPoint[0] + 3 * (1-t) * t**2 * cPoint[0] + t**3 * dPoint[0],
		(1-t)**3 * aPoint[1] + 3 * (1-t)**2 * t * bPoint[1] + 3 * (1-t) * t**2 * cPoint[1] + t**3 * dPoint[1]
	)


def cleanPointPositionsOnLayer(l):
	for p in l.paths:
		for i in range(len(p.nodes)-1, -1, -1):
			n = p.nodes[i]
			if not n.nextNode:
				continue
				
			bothOnCurves = n.type != OFFCURVE and n.nextNode.type != OFFCURVE
			bothOffCurves = n.type == OFFCURVE and n.nextNode.type == OFFCURVE
			if not bothOnCurves and not bothOffCurves:
				continue
	
			if distance(n.prevNode.position, n.position) > distance(n.prevNode.position, n.nextNode.position):
				closerPos = n.nextNode.position
				furtherPos = n.position
				n.position = closerPos
				n.nextNode.position = furtherPos
			
			if bothOnCurves:
				if n.nextNode.nextNode.type == OFFCURVE and n.nextNode.nextNode.nextNode.type == OFFCURVE:
					nextOnCurve = n.nextNode.nextNode.nextNode.nextNode
					if distance(n.nextNode.position, nextOnCurve.position) > distance(n.position, nextOnCurve.position):
						closerPos = n.nextNode.position
						furtherPos = n.position
						n.position = closerPos
						n.nextNode.position = furtherPos
			
			if bothOffCurves:
				correctedHandlePositions = correctHandles(n.prevNode.position, n.position, n.nextNode.position, n.nextNode.nextNode.position)
				if correctedHandlePositions:
					n.position, n.nextNode.position = correctedHandlePositions
			
			if not n.prevNode:
				continue
			
			if n.type != OFFCURVE:
				bringBackAcuteNode(n.prevNode, n, n.nextNode)


def posOfNode(n):
	return (round(n.position.x), round(n.position.y),)


def removeShortSegments(thisLayer, maxLength=10.0, passes=2, protectedPositions=[]):
	thisLayer.selection = None
	hasRemovedSegments = True
	for x in range(passes):
		if hasRemovedSegments: 
			# if no segments were removed in pass 2, do nothing in passes 3+
			# (faster if user enters too many passes)
			hasRemovedSegments = False
			for thisPath in thisLayer.paths:
				for i in range(len(thisPath.nodes))[::-1]: # go backwards through nodes, so i remains correct
					thisNode = thisPath.nodes[i]
					nextNode = thisNode.nextNode

					if posOfNode(thisNode) in protectedPositions: # or posOfNode(nextNode) in protectedPositions:
						thisNode.selected = True
						continue

					if thisNode.type != OFFCURVE and nextNode.type != OFFCURVE:
						if distance(thisNode.position, nextNode.position) < maxLength:
							thisPath.removeNodeCheckKeepShape_(thisNode)
							hasRemovedSegments = True # OK to start another pass

					elif thisNode.type == OFFCURVE and nextNode.type == OFFCURVE and nextNode.nextNode.type == CURVE:
						if posOfNode(nextNode.nextNode) in protectedPositions:
							nextNode.nextNode
							continue

						if calculateCurveLength(thisNode.prevNode.position, thisNode.position, nextNode.position, nextNode.nextNode.position) < maxLength:
							del thisPath.nodes[i:i+2]
							hasRemovedSegments = True # OK to start another pass


def subdivideShape(path, minLength=20, maxLength=60, removeOverlap=True, protectedPositions=[]):
	newNodes = []
	
	if removeOverlap:
		isCounter = path.direction != -1
		if isCounter:
			path.reverse()
		for i in range(len(path.segments)):
			path.removeOutsideOverlaps_(True)
		if isCounter:
			path.reverse()
			
		# l = GSLayer()
		# l.shapes.append(path)
		# l.removeOverlap()
		# path = copy(l.shapes[0])
	
	for segment in path.segments:
		isCurve = segment.countOfPoints() == 4
		if isCurve:
			A, B, C, D = [(p.x, p.y) for p in segment.segmentStruct()[0]]
			coords = subdivideBezier(A, B, C, D, minLength, maxLength)
			for i, coord in enumerate(coords):
				newNode = GSNode()
				newNode.position = NSPoint(*coord)
				if i % 3 == 0:
					if len(newNodes) == 0:
						newNode.type = LINE
					elif newNodes[-1].type == OFFCURVE:
						newNode.type = CURVE
					else:
						continue
				else:
					newNode.type = OFFCURVE
				newNodes.append(newNode)
		else:
			A, B = [(p.x, p.y) for p in segment.segmentStruct()[0][:2]]
			coords = subdivideLine(A, B, minLength, maxLength)
			for coord in coords:
				newNode = GSNode()
				newNode.position = NSPoint(*coord)
				newNode.type = LINE
				if len(newNodes)==0:
					newNodes.append(newNode)
				elif distance(newNodes[-1].position, newNode.position) >= 2.0 and distance(newNodes[0].position, newNode.position) >= 2.0:
					newNodes.append(newNode)
	
	# remove short line segments:
	for i in range(len(newNodes)-1, -1, -1):
		currNode = newNodes[i]
		nextNode = newNodes[(i+1) % len(newNodes)]
		prevNode = newNodes[(i-1) % len(newNodes)]
		if currNode.type != OFFCURVE and nextNode.type != OFFCURVE and prevNode.type != OFFCURVE:
			if isOnLine(prevNode.position, currNode.position, nextNode.position):
				segmentLength = distance(currNode.position, nextNode.position)
				if segmentLength < minLength and not posOfNode(currNode) in protectedPositions:
					newNodes.pop(i)
				elif segmentLength > (minLength + maxLength) / 2:
					pos1, pos2 = handlePositionsForLine(currNode.position, nextNode.position)
					nextNode.type = CURVE
					for pos in (pos2, pos1):
						handle = GSNode()
						handle.type = OFFCURVE
						handle.position = pos
						newNodes.insert(i+1, handle)
					
	newPath = GSPath()
	if newNodes[0].position == newNodes[-1].position:
		newNodes.pop(0)
	newPath.nodes = newNodes
	newPath.closed = True
	return newPath


def prepareForMacroReport():
	splitview = Glyphs.delegate().macroPanelController().consoleSplitView()
	height = splitview.frame().size.height
	currentPos = splitview.positionOfDividerAtIndex_(0) / height
	newPos = 0.1
	splitview.setPosition_ofDividerAtIndex_(height * newPos, 0)

	Glyphs.showMacroWindow()


class Papyrify(object):
	prefID = "com.mekkablue.Papyrify"
	prefDict = {
		# "prefName": defaultValue,
		"pushAmount": 12,
		"minLength": 20,
		"maxLength": 80,
		"removeOverlap": True,
	}
	
	def __init__( self ):
		# Window 'self.w':
		windowWidth  = 200
		windowHeight = 180
		windowWidthResize  = 100 # user can resize width by this value
		windowHeightResize = 0   # user can resize height by this value
		self.w = vanilla.FloatingWindow(
			(windowWidth, windowHeight), # default window size
			"Papyrify", # window title
			minSize = (windowWidth, windowHeight), # minimum size (for resizing)
			maxSize = (windowWidth + windowWidthResize, windowHeight + windowHeightResize), # maximum size (for resizing)
			autosaveName = self.domain("mainwindow") # stores last window position and size
		)
		
		# UI elements:
		linePos, inset, lineHeight = 12, 15, 22
		self.w.descriptionText = vanilla.TextBox((inset, linePos+2, -inset, 14), "Papyrus effect for 2 masters", sizeStyle="small", selectable=True)
		linePos += lineHeight
		
		tabStop = 70
		
		self.w.pushText = vanilla.TextBox((inset, linePos+2, tabStop, 14), "Push", sizeStyle="small", selectable=True)
		self.w.pushAmount = vanilla.EditText((inset+tabStop, linePos, -inset, 19), self.pref("pushAmount"), callback=self.SavePreferences, sizeStyle="small")
		linePos += lineHeight

		self.w.minText = vanilla.TextBox((inset, linePos+2, tabStop, 14), "Min Length", sizeStyle="small", selectable=True)
		self.w.minLength = vanilla.EditText((inset+tabStop, linePos, -inset, 19), self.pref("minLength"), callback=self.SavePreferences, sizeStyle="small")
		linePos += lineHeight

		self.w.maxText = vanilla.TextBox((inset, linePos+2, tabStop, 14), "Max Length", sizeStyle="small", selectable=True)
		self.w.maxLength = vanilla.EditText((inset+tabStop, linePos, -inset, 19), self.pref("maxLength"), callback=self.SavePreferences, sizeStyle="small")
		linePos += lineHeight
		
		self.w.removeOverlap = vanilla.CheckBox((inset, linePos-1, -inset, 20), "Remove Opened Corners", value=True, callback=self.SavePreferences, sizeStyle="small")
		linePos += lineHeight
		
		# Run Button:
		self.w.runButton = vanilla.Button((-90-inset, -20-inset, -inset, -inset), "Papyrify", sizeStyle="regular", callback=self.PapyrifyMain)
		self.w.setDefaultButton(self.w.runButton)
		
		# Load Settings:
		if not self.LoadPreferences():
			print("⚠️ ‘Papyrify’ could not load preferences. Will resort to defaults.")
		
		# Open window and focus on it:
		self.w.open()
		self.w.makeKey()
	
	def domain(self, prefName):
		prefName = prefName.strip().strip(".")
		return self.prefID + "." + prefName.strip()
	
	def pref(self, prefName):
		prefDomain = self.domain(prefName)
		return Glyphs.defaults[prefDomain]
	
	def SavePreferences(self, sender=None):
		try:
			# write current settings into prefs:
			for prefName in self.prefDict.keys():
				Glyphs.defaults[self.domain(prefName)] = getattr(self.w, prefName).get()
			return True
		except:
			import traceback
			print(traceback.format_exc())
			return False

	def LoadPreferences(self):
		try:
			for prefName in self.prefDict.keys():
				# register defaults:
				Glyphs.registerDefault(self.domain(prefName), self.prefDict[prefName])
				# load previously written prefs:
				getattr(self.w, prefName).set(self.pref(prefName))
			return True
		except:
			import traceback
			print(traceback.format_exc())
			return False

	def PapyrifyMain(self, sender=None):
		try:
			# update settings to the latest user input:
			if not self.SavePreferences():
				print("⚠️ ‘Papyrify’ could not write preferences.")
			
			# read prefs:
			for prefName in self.prefDict.keys():
				try:
					setattr(sys.modules[__name__], prefName, self.pref(prefName))
				except:
					fallbackValue = self.prefDict[prefName]
					print(f"⚠️ Could not set pref ‘{prefName}’, resorting to default value: ‘{fallbackValue}’.")
					setattr(sys.modules[__name__], prefName, fallbackValue)
			
			thisFont = Glyphs.font # frontmost font
			if thisFont is None:
				Message(title="No Font Open", message="The script requires a font. Open a font and run the script again.", OKButton=None)
			elif len(thisFont.masters) != 2:
				Message(title="Two masters required", message=f"The script requires exactly 2 masters. The frontmost font has {len(thisFont.masters)}.", OKButton=None)
			else:
				filePath = thisFont.filepath
				if filePath:
					reportName = f"{filePath.lastPathComponent()}\n📄 {filePath}"
				else:
					reportName = f"{thisFont.familyName}\n⚠️ The font file has not been saved yet."
				print(f"Papyrify Report for {reportName}")
				print()
			
				pushAmount = int(self.pref("pushAmount"))
				minLength = int(self.pref("minLength"))
				maxLength = int(self.pref("maxLength"))
				removeOverlap = bool(self.pref("removeOverlap"))
				
				
				for selectedLayer in thisFont.selectedLayers:
					glyph = selectedLayer.parent
					
					pap000 = glyph.layers[thisFont.masters[0].id]
					if pap000.background.paths:
						pap000.clear()
					else:
						pap000.swapForegroundWithBackground()

					pap100 = glyph.layers[thisFont.masters[1].id]
					pap100.clear()
					pap100.width = pap000.width

					protectedPositions = []
					for p in glyph.layers[0].background.paths:
						for n in p.nodes:
							if n.type != OFFCURVE and n.connection == GSSHARP:
								protectedPositions.append(
									(round(n.position.x), round(n.position.y),)
									)
						
						newP = subdivideShape(p, minLength, maxLength, removeOverlap=removeOverlap, protectedPositions=protectedPositions)
						pap000.shapes.append(newP)
						
					removeShortSegments(pap000, maxLength=minLength*0.6, protectedPositions=protectedPositions)
					
					for shape in pap000.shapes:
						pap100.shapes.append(copy(shape))

					for p in pap100.paths:
						for n in p.nodes + p.nodes:
							if n.type == CURVE:
								A, B, C, D, E = n.prevNode.prevNode.prevNode, n.prevNode.prevNode, n.prevNode, n, n.nextNode
								if random() > 0.8:
									bend = random() * 0.5
									B.position = getPerpendicularPoint(B, D, distance(A.position, D.position) * bend)
									C.position = getPerpendicularPoint(C, D, distance(A.position, D.position) * bend)
								if E.type == OFFCURVE:
									push = pushAmount * random()
									newDpos = getPerpendicularPoint(D, E, push)
									move = subtractPoints(newDpos, D.position)
									D.position = newDpos
									C.position = addPoints(C.position, move)
									E.position = addPoints(E.position, move)
							elif n.type == LINE:
									push = pushAmount * random()
									n.position = getPerpendicularPoint(n, n.nextNode, push)
			
					for i in range(2):
						cleanPointPositionsOnLayer(pap100)
				# self.w.close() # delete if you want window to stay open

			print("\nDone.")

		except Exception as e:
			# brings macro window to front and reports error:
			prepareForMacroReport()
			print(f"Papyrify Error: {e}")
			import traceback
			print(traceback.format_exc())

Papyrify()


"""
# GSLayer:
- (FTPointArray *)intersections {
	if (_intersections) {
		return _intersections;
	}
	NSMutableArray *segments = [NSMutableArray new];
	for (GSPath *path in self.paths) {
		[segments addObjectsFromArray:path.segments];
	}
	_intersections = [FTPointArray new];
	int s1idx = -1;
	int s2idx = -1;
	for (GSPathSegment *s1 in segments) {
		s1idx++;
		s2idx = -1;
		for (GSPathSegment *s2 in segments) {
			s2idx++;
			if (s2idx <= s1idx) {
				continue;
			}
			FTPointArray *Is = [s1 intersectionPoints:s2];
			for (NSUInteger idx = 0; idx < Is.count; idx++) {
				NSPoint I = [Is pointAtIndex:idx];
				[_intersections addPoint:I];
			}
		}
	}
	return _intersections;
}
"""