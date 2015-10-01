#!/usr/bin/python
import os, sys

#################################################################################################
##
## Pixie's RenderMan interface wrapper for Python wrap test
## (c) Cedric PAILLE -- 2012
##
#################################################################################################

sys.path.append("../../python/")

points = {"P":[0., 0., 1.,  0., 1., 1.,  0., 2., 1.,  0., 0., 0.,  0., 1., 0.,  0., 2., 0.,
		0., 0.25, 0.5,  0., .75, .75,  0., .75, .25,
		0., 1.25, 0.5,  0., 1.75, .75,  0., 1.75, .25]}

import ri

ri.Begin()
ri.Projection("perspective",{"fov":[60.0]})
ri.Display("test.tif","framebuffer","rgb")
ri.Translate(0,0,5)
ri.Rotate(-45,0,1,0)
ri.PixelSamples(6,6)
ri.WorldBegin()
ri.PointsGeneralPolygons ([2,2],[4,3,4,3], [0,1,4,3,6,7,8,1,2,5,4,9,10,11], points)
ri.WorldEnd()
ri.End()
