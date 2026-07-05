#MenuTitle: Prepare Font
# -*- coding: utf-8 -*-
from __future__ import division, print_function, unicode_literals
__doc__="""
Pick a master from one of the currently open fonts and turn it into a two-master Papyrify setup: a copy of the font with a single ‘Papyrify’ axis (tag PAPY) and two masters, one at PAPY=0 and one at PAPY=100. Glyphs are decomposed and cleaned up (node names, notes, guides, colors, annotations, backgrounds and backup layers removed) so only what is necessary for the effect remains.
"""

import vanilla
from copy import copy
from Foundation import NSUUID


class PrepareFont(object):
	prefID = "com.mekkablue.PrepareFont"
	prefDict = {
		# "prefName": defaultValue,
		"master": 0,
	}

	def __init__(self):
		# collect all masters of all open fonts:
		self.masterList = self.collectMasters()

		# Window 'self.w':
		windowWidth = 320
		windowHeight = 110
		windowWidthResize = 200  # user can resize width by this value
		windowHeightResize = 0   # user can resize height by this value
		self.w = vanilla.FloatingWindow(
			(windowWidth, windowHeight),  # default window size
			"Prepare Font for Papyrify",  # window title
			minSize=(windowWidth, windowHeight),  # minimum size (for resizing)
			maxSize=(windowWidth + windowWidthResize, windowHeight + windowHeightResize),  # maximum size (for resizing)
			autosaveName=self.domain("mainwindow")  # stores last window position and size
		)

		# UI elements:
		linePos, inset, lineHeight = 12, 15, 22
		self.w.descriptionText = vanilla.TextBox((inset, linePos + 2, -inset, 14), "Pick the master to use for the Papyrify effect:", sizeStyle="small", selectable=True)
		linePos += lineHeight

		self.w.master = vanilla.PopUpButton((inset, linePos, -inset - 22, 19), [entry[0] for entry in self.masterList], callback=self.SavePreferences, sizeStyle="small")
		self.w.master.setToolTip("The master of an open font that should be duplicated into the two Papyrify masters (PAPY=0 and PAPY=100).")
		self.w.updateButton = vanilla.Button((-inset - 20, linePos, -inset, 18), "↺", callback=self.updateMasters)
		self.w.updateButton.setToolTip("Refresh the list of masters from the currently open fonts.")
		linePos += lineHeight

		# Run Button:
		self.w.runButton = vanilla.Button((-90 - inset, -20 - inset, -inset, -inset), "Prepare", sizeStyle="regular", callback=self.PrepareFontMain)
		self.w.setDefaultButton(self.w.runButton)

		# Load Settings:
		if not self.LoadPreferences():
			print("⚠️ ‘Prepare Font’ could not load preferences. Will resort to defaults.")

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

	def collectMasters(self):
		# returns a list of (label, font, masterID) for every master of every open font
		masterList = []
		for thisFont in Glyphs.fonts:
			for thisMaster in thisFont.masters:
				familyName = thisFont.familyName or "Untitled"
				masterList.append(("%s – %s" % (familyName, thisMaster.name), thisFont, thisMaster.id))
		return masterList

	def updateMasters(self, sender=None):
		self.masterList = self.collectMasters()
		self.w.master.setItems([entry[0] for entry in self.masterList])

	def PrepareFontMain(self, sender=None):
		try:
			if not self.SavePreferences():
				print("⚠️ ‘Prepare Font’ could not write preferences.")

			if not self.masterList:
				Message(title="No Font Open", message="The script requires an open font. Open a font and run the script again.", OKButton=None)
				return

			# determine the master the user picked:
			masterIndex = min(int(self.pref("master")), len(self.masterList) - 1)
			label, pickedFont, pickedMasterID = self.masterList[masterIndex]

			print("Prepare Font Report")
			print()
			print("Picked master: %s" % label)

			# duplicate the whole font (keeps master IDs, so we can find the picked master):
			font = copy(pickedFont)
			font.disableUpdateInterface()
			try:
				chosenMaster = font.masters[pickedMasterID]
				if chosenMaster is None:
					# fall back to positional index within the picked font:
					chosenMaster = font.masters[0]

				# remove all masters except the chosen one:
				for thisMaster in list(font.masters):
					if thisMaster.id != chosenMaster.id:
						font.removeFontMaster_(thisMaster)

				# remove all axes and add a single Papyrify (PAPY) axis:
				papyAxis = GSAxis()
				papyAxis.name = "Papyrify"
				papyAxis.axisTag = "PAPY"
				font.axes = [papyAxis]

				# put the chosen master at PAPY=0:
				baseName = chosenMaster.name or "Master"
				chosenMaster.name = "%s PAPY 0" % baseName
				chosenMaster.axes = [0]

				# decompose and clean up every glyph on the chosen master, and strip everything
				# that is not needed for the Papyrify effect (inspired by Garbage Collection):
				for thisGlyph in font.glyphs:
					# remove glyph color and note:
					thisGlyph.color = None
					thisGlyph.note = None

					# remove all backup layers (keep master and special layers):
					for i in range(len(thisGlyph.layers) - 1, -1, -1):
						thisLayer = thisGlyph.layers[i]
						if not thisLayer.isMasterLayer and not thisLayer.isSpecialLayer:
							del thisGlyph.layers[i]

					thisLayer = thisGlyph.layers[chosenMaster.id]
					if thisLayer is None:
						continue

					# decompose components into paths:
					decomposedLayer = thisLayer.copyDecomposedLayer()
					decomposedLayer.layerId = chosenMaster.id
					decomposedLayer.associatedMasterId = chosenMaster.id
					thisGlyph.layers[chosenMaster.id] = decomposedLayer
					thisLayer = thisGlyph.layers[chosenMaster.id]

					# tidy up the decomposed paths:
					thisLayer.cleanUpPaths()

					# strip node names (markers left behind by other scripts):
					for thisPath in thisLayer.paths:
						for thisNode in thisPath.nodes:
							thisNode.name = None

					# clear the background:
					thisLayer.background.clear()

					# remove guides, colors and annotations (foreground and background):
					thisLayer.guides = []
					thisLayer.background.guides = []
					thisLayer.color = None
					thisLayer.annotations = []
					thisLayer.background.annotations = []

				# remove global (master) guides:
				chosenMaster.guides = []

				# duplicate the chosen master and put the copy at PAPY=100:
				secondMaster = copy(chosenMaster)
				secondMaster.id = NSUUID.UUID().UUIDString()
				secondMaster.name = "%s PAPY 100" % baseName
				font.masters.append(secondMaster)
				secondMaster.axes = [100]

				# copy the glyph layers into the new master (appending a master leaves them empty):
				for thisGlyph in font.glyphs:
					thisGlyph.setLayer_forId_(copy(thisGlyph.layers[chosenMaster.id]), secondMaster.id)
			finally:
				font.enableUpdateInterface()

			# open the prepared font:
			Glyphs.fonts.append(font)

			print("✅ Prepared ‘%s’ with axis PAPY and masters at 0 and 100." % (font.familyName or "Untitled"))
			print("\nDone.")
			Glyphs.showNotification("Prepare Font", "Prepared a two-master Papyrify setup. Details in Macro Window.")

		except Exception as e:
			import traceback
			Glyphs.showMacroWindow()
			print("Prepare Font Error: %s" % e)
			print(traceback.format_exc())


PrepareFont()
