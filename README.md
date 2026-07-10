# Papyrify

Glyphs.app scripts that turn any font into a variable font with a “Papyrus” effect axis: at one end of the axis, glyphs look normal; at the other, their outlines look roughened and hand-cut, like they were carved into papyrus. Inspired by Matteo Bologna.

The repository ships two scripts that are meant to be used one after the other:

1. **Prepare Font** – turns a single master into the two-master `PAPY` setup that Papyrify needs.
2. **Papyrify** – applies the actual roughening effect to those two masters.

## Requirements

- Glyphs 3 or later (the scripts use `vanilla` and current `GSFont`/`GSAxis` APIs).

## Installation

1. In Glyphs, choose _Window > Plugin Manager._
2. Switch to the _Scripts_ tab and install **Papyrify.**
3. In Glyphs 3, hold down the Option key and choose _Script > Reload Scripts_ (not necessary in Glyphs 4).
4. Both scripts now appear in _Script > Papyrify._

## Suggested workflow

The fastest way from a single-master font to a Papyrus variable font:

1. Open the font you want to Papyrify.
2. Run _Script > Papyrify > Prepare Font_ and pick the master to base the effect on. Click **Prepare.** A new, untitled font opens — this is your two-master `PAPY=0`/`PAPY=100` working file.
3. In the new font, select all glyphs you want to affect (⌘A in Font View, or open an Edit tab with the glyphs/text you care about).
4. Run _Script > Papyrify > Papyrify,_ leave the defaults for a first try, and click **Papyrify.**
5. Open an Edit tab, show the interpolation slider (bottom bar of the Edit view) and scrub the `PAPY` axis from 0 to 100 to see the roughening interpolate.
6. Not rough enough, too rough, or too chunky? Adjust **Push**, **Min Length** and **Max Length** (see [Options](#papyrify-options) below) and run Papyrify again on the same selection — it overwrites the previous result. Because the effect is randomized, running it again with the *same* values also gives you a fresh variation to choose from.
7. Once you're happy, save the font and export it as a variable font (_File > Export…_, format “Variable Font”).
8. Optional: repeat step 6 selectively on individual glyphs that need more or less roughening than the rest.

## Prepare Font

Automates the manual two-master setup described under [Manual setup](#manual-setup).

1. Open the font (or fonts) you want to use.
2. Run _Script > Papyrify > Prepare Font._
3. **Option: master to use** — a pop-up listing every master of every open font (labeled _Family – Master_). Pick the one to base the effect on. If you open or close fonts while the window is up, click the **↺** button to refresh the list.
4. Click **Prepare.**
5. A new, untitled font opens. It is a copy of the source font in which:
   - all axes are replaced by a single **Papyrify** axis with tag `PAPY`;
   - only the chosen master remains, sitting at `PAPY=0`, plus a duplicate of it at `PAPY=100`;
   - every glyph is decomposed, and node names, glyph notes, guides, colors, annotations, backgrounds and backup layers are removed, leaving only what the effect needs.
6. Save the new font wherever you like, then continue with Papyrify.

## Papyrify

Applies the Papyrus effect to a two-master font (for example, one produced by _Prepare Font_). The script requires **exactly two masters** and works on the current glyph selection.

1. Open the two-master font and select the glyphs you want to papyrify — either in Font View, or by putting them in an Edit tab.
2. Run _Script > Papyrify > Papyrify._
3. Set the parameters (see table below), then click **Papyrify.**
4. The first master (`PAPY=0`) receives the outline subdivided into short segments; the second master (`PAPY=100`) receives the same outline pushed outward at random.
5. Move the `PAPY` slider in a preview to see the effect interpolate. Re-run the script to roll a new random variation, or tweak the values and run again — each run overwrites the previous result for the selected glyphs.
6. Export as a variable font.

### Papyrify options

| Option | Default | What it does |
| --- | --- | --- |
| **Push** | `12` | Maximum distance (in font units) a node on the `PAPY=100` master is shifted outward, perpendicular to the outline. The actual shift per node is randomized between `0` and this value, so higher values give a rougher, more jagged edge; lower values give a subtler wobble. |
| **Min Length** | `20` | The shortest segment length (in font units) an outline is subdivided into before the effect is applied. |
| **Max Length** | `80` | The longest segment length (in font units) an outline is subdivided into. Each new segment's length is picked at random between Min Length and Max Length, so together the two values set the range of “bite size” for the roughening — small values give many small, fine notches; large values give fewer, broader ones. Max Length should be larger than Min Length. |
| **Remove Opened Corners** | on | While subdividing, removes outside overlaps and cleans up corners that the subdivision would otherwise leave open. Turn off only if this produces unwanted results on a particular glyph and you prefer to clean up manually. |

Tips for the numbers:

- Keep **Min Length**/**Max Length** roughly proportional to stroke width: for a text-weight face, `20`–`40` / `60`–`100` is a good starting range; for very bold or very light weights, scale accordingly.
- Larger **Push** values relative to Min/Max Length can pull outlines so far that counters collide — if shapes look broken or self-intersecting at `PAPY=100`, lower Push or raise the Length range.
- Since every run uses fresh random numbers, run Papyrify a few times with the same settings and keep the variation you like best.

## Manual setup

If you would rather not use _Prepare Font,_ you can set the axis up by hand. For a single-master font:

1. In _Font Info > Font,_ add an axis called “Papyrus” (or any other arbitrary name) with tag `PAPY`.
2. In _Font Info > Masters,_ set the master's Papyrus coordinate to `0`.
3. In _Font Info > Masters,_ duplicate the first master and set the coordinate of the duplicate to `100`.
4. Run Papyrify, experiment with the values, apply.
5. Export as a variable font.

# License

Copyright 2025 Rainer Erich Scheichelbauer (@mekkablue).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

See the License file included in this repository for further details.
