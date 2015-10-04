# improved-sapling-tree-generator
A new version of Blenders sapling tree generator with improvements, new features, and bug fixes

####Big Changes
Branch splitting is now much improved, especially for trunk splits. Splitting is controlable, more realistic, and works with auto curve handles.
Splitting is more balanced and uniformly random.

Braches now sprout from the tip of the parent branch, these branches grow straight out, ignoring down angle. This creates the apperance of the branch transitioning from one split level to the next. On the trunk, this creates a more realistic crown for central leader trees. This also means branches will always end with the last split level, so there are no branches ending without leaves.

New branch rotation method (for first level only) to grow branches outward from the center of the tree, and setting the rotation so that the distribution of branches on the outside is even.
The rotation method can be changed with the 'Branching Mode' property

New leaf rotation options can set the leaf orientation so that the leaves face upwards and outwards in a much more realistic way.

####Change Log
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
* The last stem on a branch now grows straight out, ignoring down angle. This improves apperance and creates a more realistic crown for central leader trees.
* Taper can now be calculated automaticly based on branch lengths to make the branch radius change linearly from each split level to the next.
* 'Use UV for mapping' is now enabled by default
* leaf down angle and rotation now have their own parameters in the leaf panel
* negative down angle values for leaves now works similarly to branches
* leaf scale taper works for palmate compound leaves (negative number of leaves)
* leaf bend removed, is set to zero for old presets with leaf bend
* Length variation now affects split branch stems individually
* Down angle is calculated differently and should be easier to use, the old method is available.

######New features
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

######notes
'leaf bend' seems to be broken and probably shouldn't be used, it may be removed soon

*update*: leaf bend removed form the interface

*update*: split bias works better now
