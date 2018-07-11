# -*- coding: utf-8 -*-
"""
Created on Tue Mar 27 13:59:43 2018

@author: ofn77899
"""

import numpy
from ccpi.segmentation.SimpleflexSegmentor import SimpleflexSegmentor
from ccpi.viewer.CILViewer import CILViewer
from ccpi.viewer.CILViewer2D import CILViewer2D, Converter
import vtk

def surface2vtkPolyData(coord_list, origin = (0,0,0), spacing=(1,1,1)):
    
    ########################################################################
    # 7. Display
    # with the retrieved data we construct polydata actors to be displayed
    # with VTK. Notice that this part is VTK specific. However, it shows how to 
    # process the data returned by the algorithm.
    
    # Create the VTK output
    # Points coordinates structure
    triangle_vertices = vtk.vtkPoints()
    #associate the points to triangles
    triangle = vtk.vtkTriangle()
    trianglePointIds = triangle.GetPointIds()
    # put all triangles in an array
    triangles = vtk.vtkCellArray()
    isTriangle = 0
    nTriangle = 0
    
    surface = 0
    # associate each coordinate with a point: 3 coordinates are needed for a point
    # in 3D. Additionally we perform a shift from image coordinates (pixel) which
    # is the default of the Contour Tree Algorithm to the World Coordinates.
    # TODO: add this in the algorithm.
    
    mScaling = numpy.asarray([spacing[0], 0,0,0,
                              0,spacing[1],0,0,
                              0,0,spacing[2],0,
                              0,0,0,1]).reshape((4,4))
    mShift = numpy.asarray([1,0,0,origin[0],
                            0,1,0,origin[1],
                            0,0,1,origin[2],
                            0,0,0,1]).reshape((4,4))

    	
    mTransform = numpy.dot(mScaling, mShift)
        
    point_count = 0
    for surf in coord_list:
        print("Image-to-world coordinate trasformation ... %d" % surface)
        for point in surf:
        
            world_coord = numpy.dot(mTransform, point)
            xCoord = world_coord[2]
            yCoord = world_coord[1]
            zCoord = world_coord[0]
    #        i += 3
            triangle_vertices.InsertNextPoint(xCoord, yCoord, zCoord);
    
    
        # The id of the vertex of the triangle (0,1,2) is linked to
        # the id of the points in the list, so in facts we just link id-to-id
            trianglePointIds.SetId(isTriangle, point_count)
            isTriangle += 1
            point_count += 1
    
            if (isTriangle == 3) :
                    isTriangle = 0;
                    # insert the current triangle in the triangles array
                    triangles.InsertNextCell(triangle);
    
    
        surface += 1
    
    # polydata object
    trianglePolyData = vtk.vtkPolyData()
    trianglePolyData.SetPoints( triangle_vertices )
    trianglePolyData.SetPolys(  triangles  )
    return trianglePolyData

reader = vtk.vtkMetaImageReader()
#reader.SetFileName("../../data/fuel_uc_python.mha")
reader.SetFileName(r'C:\Users\ofn77899\Documents\Projects\CCPi\GitHub\CCPi-Simpleflex\data\head.mha')
reader.Update()

seg = SimpleflexSegmentor()
seg.setInputData(Converter.vtk2numpy(reader.GetOutput())) 

seg.calculateContourTree()

#seg.setIsoValuePercent(24.)
seg.setLocalIsoValuePercent(0.)
seg.resetCollapsePriority(seg.PRIORITY_VOLUME)

# 5. Construct the iso-surfaces
print ("calling resetCollapsePriority")


#seg.updateTreeFromLogTreeSize(size=0.6, isGlobal=False)
print ("calling setlogtreesize")
seg.ct.SetLogTreeSize(.3)
print ("calling UpdateTreeFromLogTreeSize")
seg.ct.UpdateTreeFromLogTreeSize()
print ("calling ConstructLocalIsoSurface")
#seg.constructLocalIsoSurfaces()
seg.ct.ConstructLocalIsoSurface()
print ("called ConstructLocalIsoSurface")
#seg.constructIsoSurfaces()

# 6. Retrieve the isosurfaces and display
coord_list = seg.getSurfaces()

del (seg)
#print ("getSurface " , len(coord_list))
spacing = numpy.asarray(reader.GetOutput().GetSpacing())
s1 = spacing[0]
spacing[0] = spacing[2]
spacing[2] = s1

print (len(coord_list))

#fp = (31.88687651673049, 40.44165237344525, 9.095568576800076)
#cp = (20.247985294179337, -81.77039176753509, 129.73458294808282)
#vu = (-0.011457730383621242, 0.6485580152544185, 0.7610789849047903)
#vu = (0,0,1)



v = CILViewer()
v.setInput3DData(reader.GetOutput())
v.displayPolyData(surface2vtkPolyData(coord_list, spacing=spacing))
v.startRenderLoop()
vu = v.getCamera().GetViewUp()
fp = v.getCamera().GetFocalPoint()
cp = v.getCamera().GetPosition()
#v.getCamera().SetClippingRange(10,400)
print ("camera position" , cp)
print ("focal point" , fp)
print ("view up" , vu)
v.getRenderer().Render()
#v.startRenderLoop()
v.saveRender("try")

if vu != (0,0,1):
    Rot = vtk.vtkTransform()
    Rot.RotateX(vu[0])
    Rot.RotateY(vu[1])
    Rot.RotateZ(vu[2])
    Rot.Update()
    mRot = numpy.zeros((4,4), dtype=float)
    for i in range(4):
        for j in range(4):
            mRot[i][j] = Rot.GetMatrix().GetElement(i,j)
    
num=30
angles = numpy.linspace(0,360, num=num)
ncp=list(cp)
Rot = vtk.vtkTransform()
Rot.Translate(fp[0],fp[1],fp[2])
Rot.RotateWXYZ(360/num, vu[0], vu[1],vu[2])
Rot.Translate(-fp[0],-fp[1],-fp[2])
#Rot.Translate(-fp[0],-fp[1],-fp[2])
Rot.Update()
for t in range(num):
    angle = angles[t]
    
    #R = (cp[0] - fp[0] ) ** 2  + (cp[1] - fp[1] )**2
    #X = numpy.sqrt(R) * numpy.sin(angle) + fp[0]
    #Y = numpy.sqrt(R) * numpy.cos(angle) + fp[1]
    
    #cam = numpy.dot(mRot, numpy.asarray([X,Y,cp[2],0]))
    print (">>>>>>>>>> Rotating by {}".format(angle))
    
    ncp = Rot.TransformPoint(ncp[0],ncp[1],ncp[2])
    cam = ncp
    print (cam)
    a = numpy.asarray(cam)-numpy.asarray(fp)
    b = numpy.asarray(cam)-numpy.asarray(vu)
    b = numpy.asarray(cam)
    print ("%%%%%%%%%%%%%%",numpy.sqrt(a.dot(a.T)))
    print ("%%%%%%%%%%%%%%",numpy.sqrt(b.dot(b.T)))
    #print (t, angle, X, Y)
    #camera = vtk.vtkCamera()
    camera = v.getCamera()	
    camera.SetPosition(cam[0],cam[1],cam[2])
    #camera.SetPosition(X,Y,cp[2])
    #camera.SetPosition(cp)
    #camera.SetFocalPoint(fp)
    #camera.SetViewUp(vu)
    #v.getRenderer().SetActiveCamera(camera)
#v.getCamera().SetPosition(cp)
#v.getCamera().SetFocalPoint(fp)
#v.getCamera().SetViewUp(vu)
    v.getRenderer().Render()
#v.startRenderLoop()
    v.saveRender("try{}".format(t))

print (v.getCamera().GetPosition())
print (v.getCamera().GetFocalPoint())
print (v.getCamera().GetViewUp())
