#!/usr/bin/python
import os, sys


#################################################################################################
##
## Pixie's RenderMan interface wrapper for Python wrap test
## (c) Cedric PAILLE -- 2012
##
#################################################################################################

sys.path.append("../../python/")


points = {"P":[-60., 60., 0., -60., 20., 0., -60.,
		-20., 0., -60., -60., 0., -20., 60., 0., -20., 20., 45., -20., -20., 45., -20., -60., 0., 20., 60., 0.,
		20., 20., 45., 20., -20., 45., 20., -60., 0., 60., 60., 0., 60., 20., 0., 60., -20., 0., 60., -60., 0.]}

import ri

ri.Begin()
ri.Projection("perspective",{"fov":[60.0]})
ri.Display("test.tif","framebuffer","rgb")
ri.Translate(0,0,500)
ri.Rotate(-10,0,1,0)
ri.PixelSamples(6,6)
ri.WorldBegin()
ri.SubdivisionMesh ("catmull-clark",[ 4, 4, 4, 4, 4, 4, 4, 4, 4 ],[ 0, 4, 5, 1, 1, 5, 6, 2, 2, 6, 7, 3, 4, 8, 9, 5, 5, 9, 10, 6, 6, 10, 11, 7, 8, 12, 13, 9, 9, 13, 14, 10, 10, 14, 15, 11 ], ["interpolateboundary"], [0,0],[],[], points)
ri.WorldEnd()
ri.End()

