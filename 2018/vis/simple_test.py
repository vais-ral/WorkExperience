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

#Text-based input system
valid = False
while valid == False:
    try:
        InitialCameraPositionX = int(input('Enter the initital camera position on the x-axis:'))
        InitialCameraPositionY = int(input('Enter the initital camera position on the y-axis:'))
        InitialCameraPositionZ = int(input('Enter the initital camera position on the z-axis:'))
        
        FrameCount = int(input('Enter number of frames for the animation:'))
        
        ViewUp1 = int(input('Enter the first viewup value:'))
        ViewUp2 = int(input('Enter the second viewup value:'))
        ViewUp3 = int(input('Enter the third viewup value:'))
        
        FocalPointX = int(input('Enter the x-coordinate for the camera focal point:'))
        FocalPointY = int(input('Enter the y-coordinate for the camera focal point:'))
        FocalPointZ = int(input('Enter the z-coordinate for the camera focal point:'))
        
        AngleRangeStart = int(input('Enter the first value for the angle range:'))
        AngleRangeEnd = int(input('Enter the last value for the angle range:'))
        
        ClippingRangeStart = int(input('Set lowest value for clipping range:'))
        ClippingRangeEnd = int(input('Set highest value for clipping range:'))
        
        InitialCameraPosition = (InitialCameraPositionX, InitialCameraPositionY, InitialCameraPositionZ)
        FocalPoint = (FocalPointX, FocalPointY, FocalPointZ)
        AngleRange = (AngleRangeStart, AngleRangeEnd)
        ClippingRange = (ClippingRangeStart, ClippingRangeEnd)
        ViewUp = (ViewUp1, ViewUp2, ViewUp3)
        
    except ValueError:
        print('One or more of your inputs were not valid! Try again')
    else:
        valid = True


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
reader.SetFileName("../../data/fuel_uc_python.mha")
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
seg.ct.SetLogTreeSize(1)
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

v = CILViewer()
v.setInput3DData(reader.GetOutput())
v.displayPolyData(surface2vtkPolyData(coord_list, spacing=spacing))
#v.startRenderLoop()

dimX, dimY, dimZ = reader.GetOutput().GetDimensions()

#Setting locked values for camera position
locX = InitialCameraPosition[0]
locY = InitialCameraPosition[1]
locZ = InitialCameraPosition[2]

#Setting camera position
v.getCamera().SetPosition(InitialCameraPosition)
v.getCamera().SetFocalPoint(FocalPoint)

#Setting camera viewup 
v.getCamera().SetViewUp(ViewUp)

#Set camera clipping range
v.getCamera().SetClippingRange(ClippingRange)

#Defining distance from camera to focal point
r = numpy.sqrt(((InitialCameraPosition[2]-FocalPoint[2])**2)
+(InitialCameraPosition[1]-FocalPoint[1])**2)
print('Radius: {}'.format(r))


camera = vtk.vtkCamera()
camera.SetPosition(InitialCameraPosition)
camera.SetFocalPoint(FocalPoint)
camera.SetViewUp(ViewUp)
v.getRenderer().SetActiveCamera(camera)

#Animating the camera
for x in range(100):
    angle = ((numpy.pi)*4/100)*x
    NewLocationX = r*(numpy.sin(angle))+FocalPoint[0]
    NewLocationY = r*(numpy.cos(angle))+FocalPoint[1]
    NewLocationZ  = r*(numpy.cos(angle))+FocalPoint[2]
    NewLocation = (NewLocationX, NewLocationY, locZ)
    v.getCamera().SetPosition(NewLocation)

    #Rendering and saving the render
    v.getRenderer().Render()
    v.saveRender('test_{}.png'.format(x))



v.startRenderLoop()
