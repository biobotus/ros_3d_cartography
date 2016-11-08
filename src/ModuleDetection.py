#!/usr/bin/python
from __future__ import division
import oct2py as oc
from oct2py import octave, Struct
import os
import scipy.misc
import shutil
from gridfs import GridFS
import pymongo
import uuid

def SplitRosFile(RosFile, fileToData):
	""" Input : example.txt file output by pcl_to_windows ros node.
	The output is 6 .txt files [x y z r g b] which are then read by function 
	CreateMatFile().
	"""
	shutil.move("/home/ubuntu/ros_ws/example.txt", "/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/example2.txt")
	x = []
	y = []
	z = []
	r = []
	g = []
	b = []
	with open(str(RosFile)) as f:
	    data = f.readlines()
	for line in data:
	    words = line.split()
	    if not len(words) == 6:
		continue
	    else:
		x.append(words[0])
		y.append(words[1])
		z.append(words[2])
		r.append(ord(words[3]))
		g.append(ord(words[4]))
		b.append(ord(words[5]))

	grosseur = len(x)

	with open(os.path.join(fileToData,"x"), "w") as dump:
	    for i in range(0,grosseur):
		dump.write(x[i])
		dump.write(" ")
	with open(os.path.join(fileToData,"y"), "w") as dump:
	    for i in range(0,grosseur):
		dump.write(y[i])
		dump.write(" ")
	with open(os.path.join(fileToData,"z"), "w") as dump:
	    for i in range(0,grosseur):
		dump.write(z[i])
		dump.write(" ")
	with open(os.path.join(fileToData,"r"), "w") as dump:
	    for i in range(0,grosseur):
		dump.write(str(r[i]))
		dump.write(" ")

	with open(os.path.join(fileToData,"g"), "w") as dump:
	    for i in range(0,grosseur):
		dump.write(str(g[i]))
		dump.write(" ")

	with open(os.path.join(fileToData,"b"), "w") as dump:
	    for i in range(0,grosseur):
		dump.write(str(b[i]))
		dump.write(" ")
	return;

def ImageAnalysis(nx,ny, imageName):
	croptedPicFolder = '/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/croptedPictures/'
	fileToData = '/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data'
	dataFolder = '/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src'


	onePixelInMM = (1/17)*10;
	yPhotoRes = 640*onePixelInMM
	xPhotoRes = 480*onePixelInMM

	labware = []

	# Database --------------------------------------------------
	client = pymongo.MongoClient()
	biobot = client['biobot']
	fs = GridFS(biobot)
	#------------------------------------------------------------

	# Transform the data to .mat --------------------------------
	SplitRosFile(dataFolder+'/'+imageName+'.txt', fileToData)
	octave.addpath(fileToData)
	octave.CreateMatFile(fileToData)
	#------------------------------------------------------------

	octave.addpath(fileToData)
	answer = octave.ObjectDetection(imageName)
	answer = dict(answer)
	for v in answer.values():
		item={'type':v[0]}
		item['mod2DJPG'] = str(v[1]) # cropted picture name
		item['mod2D'] = v[2]	# RGB matrices of cropted picture
		item['x'] = float(v[3])
		item['y'] = float(v[4])
		item['z'] = round(float(v[5]),3)
		labware.append(item)
	print('-------------------------------------------------')
	""" x-y values calculated from pictures are in the standard format
	 Those values are, on the platform, y-x because of the design. Below 
	 is the code to correct that.  
		ny = number of square in y [0:3]
		nx = number of square in x [0:4]
	"""

	for x in range(0,len(labware)):
		x_plateforme = round(labware[x]["y"]+yPhotoRes*ny,3)
		y_plateforme = round(xPhotoRes*(nx+1)-labware[x]["x"],3)
		labware[x]["y"] = y_plateforme
		labware[x]["x"] = x_plateforme
		print(labware)
		CP = os.path.join(croptedPicFolder,labware[x]["mod2DJPG"]+'.jpg')
		scipy.misc.imsave(str(CP),labware[x]["mod2D"])
		
		# Send each labware to the database
		with open('croptedPictures/'+labware[x]["mod2DJPG"]+'.jpg', 'rb') as f:
			data = f.read()

		uid = uuid.uuid4().hex
		filename = "{}.jpg".format(uid)
		image_id = fs.put(data, filename=filename)
		item = {'id': x, 'name': labware[x]["type"], 'carto_x': labware[x]["x"], 'carto_y': labware[x]["y"], 'carto_z': labware[x]["z"], 'uuid': uid, 'filename': filename, 'image_id': image_id, 'source': '3d_cartography'}
		biobot.deck.insert_one(item)		
		
