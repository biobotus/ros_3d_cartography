#!/usr/bin/python

from __future__ import division
import oct2py as oc
from oct2py import octave, Struct
import os
import time
import scipy.misc
import shutil
from gridfs import GridFS
import pymongo
import uuid
ws_dir = os.path.join('/home','ubuntu','biobot_ros_jtk','src','ros_3d_cartography','src','object_detection')
os.chdir(ws_dir)

def SplitRosFile(RosFile, fileToData):
    """ Input : example.txt file output by pcl_to_windows ros node.
    The output is 6 .txt files [x y z r g b] which are then read by function
    CreateMatFile().
    """
    source = os.path.join('/home','ubuntu','biobot_ros_jtk','src','pcl_to_windows','example.txt')
    # Move PCL file to /ros_3d_cartography IOT modify it with out being rewritten
    shutil.copy(source, ws_dir)
    #shutil.move("/home/ubuntu/ros_ws/example.txt", "/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/example.txt")
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
    """ ImageAnalysis is calling the octave file calculating the probability that
    there is labware(s) in the actual picture. Because the x/y system of the image is
    different from the x/y system of the platform, corrections are made and add the necessary
    mm if nx and ny are larger than 1.
    INPUT : square numbers on the deck (nx/and ny are provided by planner) and the imageName
    OUTPUT: Data set is store in the DB


    This function also recalculates x-y position on the platform using nx and ny.
         x-y values calculated from pictures are in the standard format
         Those values are, on the platform, y-x because of the design. Below
         is the code to correct that.
            ny = number of square in y [0:3]
            nx = number of square in x [0:4]

         y_plateforme = round(item["y"]+yPhotoRes*ny,3)
         x_plateforme = round(xPhotoRes*(nx+1)-item["x"],3)
        
    """
    croptedPicFolder = '/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/croptedPictures/'
    fileToData = '/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/data'
    # dataFolder = '/home/ubuntu/.ros'
    # wsFolder = '/home/ubuntu/biobot_ros_jtk/src/ros_3d_cartography/src/object_detection'

    onePixelInMM = float((1/17)*10)
    yPhotoRes = float(640*onePixelInMM)
    xPhotoRes = float(480*onePixelInMM)
    labware = []
    # Database --------------------------------------------------
    client = pymongo.MongoClient()
    biobot = client['biobot']
    fs = GridFS(biobot)
    #------------------------------------------------------------
    # Transform the data to .mat --------------------------------
    SplitRosFile(ws_dir +'/'+imageName+'.txt', fileToData)
    print(imageName)
    octave.addpath(fileToData)
    print("Create mat")
    octave.CreateMatFile(fileToData)
    source = os.path.join('/home','ubuntu','.ros','example.mat')
    destination = os.path.join('/home','ubuntu','biobot_ros_jtk','src','ros_3d_cartography','src','object_detection')
    # #Move PCL file to /ros_3d_cartography IOT modify it with out being rewritten
    time.sleep(5)
    shutil.copy(source,destination)
    #------------------------------------------------------------
    print("Done .mat file")
    octave.addpath(ws_dir)
    #print dataFolder
    print imageName

    answer = octave.ObjectDetection(imageName)
    print("Done ObjectDetection")

    try:
        answer = dict(answer)
        for v in answer.values():
            try:
                item={'type':v[0]}
                item['mod2DJPG'] = str(v[1])  # cropted picture name
                item['mod2D'] = v[2]  # RGB matrices of cropted picture
                item['x'] = float(v[3])
                item['y'] = float(v[4])
                item['z'] = round(float(v[5]),3)  # Not send to DB, uses highest mod. instead (To be calculated)
                labware.append(item)
            except:
                print("Received NaN values")
        """ x-y values calculated from pictures are in the standard format
         Those values are, on the platform, y-x because of the design. Below
         is the code to correct that.
            ny = number of square in y [0:3]
            nx = number of square in x [0:4]
        """
        print("Labware result: ")
        print labware
        for item in labware:
            try:
                y_plateforme = round(item["y"]+yPhotoRes*ny,3)
                x_plateforme = round(xPhotoRes*(nx+1)-item["x"],3)
                item["y"] = y_plateforme
                item["x"] = x_plateforme
                CP = os.path.join(croptedPicFolder,item["mod2DJPG"]+'.jpg')
                print(["saved image: ",CP])
                scipy.misc.imsave(str(CP),item["mod2D"])
            except Exception as e:
                print("Nan values passed: {}".format(e))

            # Send each labware to the database
            with open(CP, 'rb') as f:
                data = f.read()

            uid = uuid.uuid4().hex
            filename = "{}.jpg".format(uid)
            image_id = fs.put(data, filename=filename)
            item = {'type': item["type"], 'carto_x': item["x"], \
                    'carto_y': item["y"], 'uuid': uid, 'filename': filename, \
                    'validated': False, 'image_id': image_id, 'source': '3d_cartography'}
            biobot.deck.insert_one(item)
    except: print('Nothing was detected')
