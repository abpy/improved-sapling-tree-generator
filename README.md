# improved-sapling-tree-generator
A new version of Blenders sapling tree generator addon with improvements, new features, and bug fixes

This addon has been [added](https://developer.blender.org/rBAc3a6d9132a44afd254fa120157c66e72b751e6f1) to blender master as of version [f7cca89](https://github.com/abpy/improved-sapling-tree-generator/commit/720f312cb083c7a1b866f006b38fcf67521a6f39) with small changes. I will continue to make changes here, perhaps larger things or more experimental features, or maybe just small things.

##### For tips on using the tree generator see [tips](https://github.com/abpy/improved-sapling-tree-generator/wiki/Tips) in the wiki.

### Big Changes
Branch splitting is now much improved, especially for trunk splits. Splitting is controlable, more realistic, and works with auto curve handles.
Splitting is more balanced and uniformly random.

Braches now sprout from the tip of the parent branch, these branches grow straight out, ignoring down angle. This creates the apperance of the branch transitioning from one split level to the next. On the trunk, this creates a more realistic crown for central leader trees. This also means branches will always end with the last split level, so there are no branches ending without leaves.

New branch rotation method (for first level only) to grow branches outward from the center of the tree, and setting the rotation so that the distribution of branches on the outside is even.
The rotation method can be changed with the 'Branching Mode' property

New leaf rotation options can set the leaf orientation so that the leaves face upwards and outwards in a much more realistic way.

An object can now be set to use for leaves with dupliverts or duplifaces

Curves can now be converted into a mesh that uses the skin modifier. With the mesh it is now possible to simplfy the armature greatly reducing the number of bones and the time it takes to generate the armature. You can now set the number of branching levels to make bones for, and the number of bones in each branch.

### Change Log
* Rearranged Interface
* Moved radius settings to a seperate panel
* Branch spliting now works more predictably
* Back curvature now adds to curvature
* 'Conical' shape is narrower at tip like real conifers
* Curvature variation now bends branches in multiple directions and increases along length of branch
* Auto curve handles is now enabled for branch splits
* fixed how child branch start points are calculated
* branch length variation now works in the range 0 - 1 and no longer produces errors
* Leaf orientation is much improved, leaves now face upwards and outwards and have correct normals
* The last stem on a branch now grows straight out, ignoring down angle.
* Taper can now be calculated automaticly based on branch lengths to make the branch radius change linearly from each split level to the next.
* 'Use UV for mapping' is now enabled by default
* leaf down angle and rotation now have their own parameters in the leaf panel
* negative down angle values for leaves now works similarly to branches
* leaf scale taper works for palmate compound leaves (negative number of leaves)
* leaf bend removed, is set to zero for old presets with leaf bend
* Length variation now affects split branch stems individually
* Down angle is calculated differently and should be easier to use, the old method is still available.
* (10-12-15) changes to improve split angle, for levels > 0, this results in the angle being a third as much as before, but more like the angle of trunk splits
* leaf down angle now works like the new branch down angle, and negative down angle variation is random
* down angle variation is now on a curve so it affects branches less at the bottom of the tree
* changed order of shapes, added Inverse Tapered Cylindrical shape
* consistent method of calculating the number of child stems and leaves, and more even distribution
* hexagonal leaves now have UVs, and UVs are centered in the map
* (11-4-15) Merge code from Garfield96, leaf settings can now be changed without updating the tree if leaves is disabled.
* 11-17 Made changes to armature and animation
  * added leaf bones, and animation looping
  * fix issues with auto curve handels and intersections
  * changed how amplitude and frequency are calculated
  * directional wind, and new controls for wind animation

* Presets are now saved to the user presets folder
* Trunk splitting now uses the rotate angle to control the direction of splits, trees are now rounder and more even than before when direction was just random
* leaves parameter now controls amount of leaves even if levels is 1
* added random variation to make branch rings work with branching mode at rotate
* (5-15-16) New easier, more intuitive settings for leaf rotation and orientation
  *  options for oppositely attached leaves
  * 'horizontal leaves' replaced with better method of making leaves rotate to face upwards
  * added option to set base size for leaves
* add way of setting the direction of the leaf dupli object
* add new 'Distance' Branching Mode that removes overlapping branches based on distance.
  this mode works about as good as 'rotate' but it particularly improves branching at the top of multi trunked trees where rotate would produce gaps. This should be considered experemental, but it probably won't change much.
* New default tree with better starting settings
* added operator to generate multiple trees

* (3/25/18) 3 commits
  * Unicode fix
  * Down angle now works to set the trunk angle
  * Automatically save backup of previous settings

* (6/10/18) added new options for splitting
  * control of straightness of splits
  * setting for length of split branches releative the the parent branch
  * outward attraction now works on the trunk, (not that it's very usefull, but atleast it does something)

In case of exiting or crash a preset is saved. To recover the previous tree this must be loaded immediately or it will be overwritten


###### New features
* Branch distribution to adjust how the first level is distributed along the height of the tree
* Option to grow branches in rings, like pine trees
* Control the shape of secondary branch split levels
* Minimum branch radius to prevent needle-thin branches
* Close tip for branch radius
* Option to make number of splits proportional to branch length
* Root flare to make trunk wider at base
* Leaf rotation can now use negetive values like branches do
* Vertical atraction can now be set individually for each split level
* 'split bias' for trunk splits to put more splits at top or bottom of tree
* Split start height for longer trunk before splitting
* Outward attraction to make branches point out from the center of the tree
* added 'horizontal leaves' and 'leaf angle' to control how leaves are rotated
* 'Leaf Scale Taper' and 'Leaf Scale Variation' to control leaf scale along branch and random variation in scale
* The shape for branch lengths can now be set to a custom shape
* new method to rotate branches evenly around a tree with splits in the trunk
* leaf shape can now be set to make mesh for dupliFaces or dupliVerts
* crown taper to shorten trunk splits farther from the center of the tree
* the height that pruning starts can be set separately from trunk height
* base size for secondary branch levels
* leaf animation
* animation looping
* option to show only the armature for fast animation playback

###### notes
'leaf bend' seems to be broken and probably shouldn't be used, it may be removed soon

*update*: leaf bend removed form the interface

*update*: split bias works better now
