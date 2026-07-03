# Papyrify

Glyphs.app script that turns any two-master setup into a VF with a Papyrus effect axis. Inspired by Matteo Bologna.

The repository ships two scripts:

- **Prepare Font** – builds a ready-to-use two-master `PAPY` setup from any master of an open font.
- **Papyrify** – applies the actual Papyrus effect to the two masters.

## Installation

Copy `Prepare Font.py` and `Papyrify.py` into your Glyphs scripts folder (in Glyphs: _Script > Open Scripts Folder,_ then place the files inside), and choose _Script > Reload Scripts_ (or hold down the Option key while opening the Script menu). Both scripts then appear in the _Script_ menu.

## Prepare Font

Turns a single master into the two-master setup that Papyrify needs. This automates the manual steps described under [Manual setup](#manual-setup) below.

1. Open the font (or fonts) you want to use.
2. Run _Script > Prepare Font._
3. In the window, pick the master you want to base the effect on from the pop-up (listed as _Family – Master_). If you open or close fonts while the window is up, click the ↺ button to refresh the list.
4. Click **Prepare.**
5. A new, untitled font opens. It is a copy of the source font in which:
   - all axes are replaced by a single **Papyrify** axis with tag `PAPY`;
   - only the chosen master remains, sitting at `PAPY=0`, plus a duplicate of it at `PAPY=100`;
   - every glyph is decomposed and cleaned up (node names, glyph notes, guides, colors, annotations, backgrounds and backup layers are removed) so only what the effect needs is left.
6. Save the new font wherever you like.

## Papyrify

Applies the Papyrus effect to a two-master font (for example, one produced by _Prepare Font_). The script requires **exactly two masters.**

1. Open the two-master font and select the glyphs you want to papyrify — either in Font View, or by putting them in an Edit tab.
2. Run _Script > Papyrify._
3. Set the parameters:
   - **Push** – how far the outline edges are jittered outward on the `PAPY=100` master (higher = rougher).
   - **Min Length** / **Max Length** – the range of segment lengths the outlines are subdivided into before the effect is applied.
   - **Remove Opened Corners** – cleans up overlaps and opened corners while subdividing.
4. Click **Papyrify.** The first master (`PAPY=0`) receives the subdivided outline, the second master (`PAPY=100`) the roughened one.
5. Experiment: move the `PAPY` slider in a preview to see the effect interpolate. Re-run the script to roll a new random variation, or tweak the values and run again.
6. Export as a variable font.

## Manual setup

If you would rather not use _Prepare Font,_ you can set the axis up by hand. For a single-master font:

1. In _Font Info > Font,_ add an axis called ‘Papyrus’ (or any other arbitrary name) with tag `PAPY`
2. In _Font Info > Masters,_ set the master’s Papyrus coordinate to `0`
3. In _Font Info > Masters,_ duplicate the first master and set the coordinate of the duplicate to `100`
4. Run Papyrify, experiment with the values, apply.
5. Export as variable font.

# License

Copyright 2025 Rainer Erich Scheichelbauer (@mekkablue).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

See the License file included in this repository for further details.
