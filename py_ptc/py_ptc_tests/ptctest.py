#!/usr/bin/python
import os, sys

#################################################################################################
## Pixie's pointcloud wrapper for Python wrap test
## (c) Cédric PAILLE -- 2012
#################################################################################################

sys.path.append("../../bin/python/")

import ptc

channels = {
	"mycolor": "color",
	"myfloat": "float"
}

matrix = (1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1)

pt = ptc.pointCloud()

# Create a new ptc file with 2 channels
pt.write("./test.ptc", channels, matrix, matrix)

# Set some points
for i in range(0,200):
	fi = (i,i,i)
	pt.setPoint( fi , (0,1,0), i / 10., {"mycolor":(.2,i/200.0,.4), "myfloat":(i/200.0,)} )

# Write out the ptc :)
pt.close()

# Reading test
pt = ptc.pointCloud()
pt.read("./test.ptc")

# Extract variables infos
vi = pt.getVarInfo()
print "Variable (name, type) : " + str(vi)

# Extract pointcloud info
print "Point count %i" % pt.getNPoints()
print "Bounding box " + str(pt.getBbox())
print "W2Eye Matrix : " + str(pt.getWorld2Eye())
print "W2Ndc Matrix : " + str(pt.getWorld2Ndc())

# Extract variable content
for i in range(0, 5):
	print "'mycolor' variable : " + str(pt.getPoint()["data"]["mycolor"])
	print "'myfloat' variable : " + str(pt.getPoint()["data"]["myfloat"])

# All is ok at this point
# ..... :)
