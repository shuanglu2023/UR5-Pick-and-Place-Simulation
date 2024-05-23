#!/home/ziyu/semantic_programming/.sem_env/bin/python3
from pandas import array
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import *
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import rospy, rospkg, rosservice
import sys
import time
import random
import numpy as np
import math
import xml.etree.ElementTree as ET

path = rospkg.RosPack().get_path("levelManager")

costruzioni = ['costruzione-1', 'costruzione-2']

def randomCostruzione():
	return random.choice(costruzioni)

def getPose(modelEl):
	strpose = modelEl.find('pose').text
	return [float(x) for x in strpose.split(" ")]

def get_Name_Type(modelEl):
	if modelEl.tag == 'model':
		name = modelEl.attrib['name']
	else:
		name = modelEl.find('name').text
	return name, name.split('_')[0]
	

def get_Parent_Child(jointEl):
	parent = jointEl.find('parent').text.split('::')[0]
	child = jointEl.find('child').text.split('::')[0]
	return parent, child


def getLego4Costruzione(select=None):
	nome_cost = randomCostruzione()
	if select is not None: nome_cost = costruzioni[select]
	print("spawning", nome_cost)

	tree = ET.parse(f'{path}/lego_models/{nome_cost}/model.sdf')
	root = tree.getroot()
	costruzioneEl = root.find('model')

	brickEls = []
	for modEl in costruzioneEl:
		if modEl.tag in ['model', 'include']:
			brickEls.append(modEl)

	models = ModelStates()
	for bEl in brickEls:
		pose = getPose(bEl)
		models.name.append(get_Name_Type(bEl)[1])
		rot = Quaternion(*quaternion_from_euler(*pose[3:]))
		models.pose.append(Pose(Point(*pose[:3]), rot))

	rospy.init_node("levelManager")
	istruzioni = rospy.Publisher("costruzioneIstruzioni", ModelStates, queue_size=1)
	istruzioni.publish(models)

	return models


def changeModelColor(model_xml, color):
	root = ET.XML(model_xml)
	root.find('.//material/script/name').text = color
	return ET.tostring(root, encoding='unicode')	


#DEFAULT PARAMETERS
package_name = "levelManager"
spawn_name = '_spawn'
level = None
selectBrick = None
maxLego = 11
spawn_pos = (-0.35, -0.42, 0.74)  		#center of spawn area
spawn_dim = (0.32, 0.23)    			#spawning area
min_space = 0.010    					#min space between lego
min_distance = 0.15   					#min distance between lego

#function parsing arguments
def readArgs():
	global package_name
	global level
	global selectBrick
	try:
		argn = 1
		while argn < len(sys.argv):
			arg = sys.argv[argn]
			
			if arg.startswith('__'):
				None
			elif arg[0] == '-':
				if arg in ['-l', '-level']:
					argn += 1
					level = int(sys.argv[argn])
				elif arg in ['-b', '-brick']:
					argn += 1
					selectBrick = brickList[int(sys.argv[argn])]
				else:
					raise Exception()
			else: raise Exception()
			argn += 1
	except Exception as err:
		print("Usage: .\levelManager.py" \
					+ "\n\t -l | -level: assigment from 1 to 4" \
					+ "\n\t -b | -brick: spawn specific grip from 0 to 10")
		exit()
		pass

brickDict = { \
		'cuboid': (0,(-0.052670+0.09,-0.274822,0.057)), \
		'blueBox': (1,(-0.052670,-0.274822,0.038)), \
		'grayBox': (2,(-0.277424,-0.251713,0.057)), \
		'octagon': (3,(0.031,0.063,0.057)), \
		'parallelogram': (4,(0.031,0.063,0.057)), \
		'star': (5,(0.031,0.095,0.057)), \
		}

brickOrientations = { \
		'X1-Y2-Z1': (((1,1),(1,3)),-1.715224,0.031098), \
		'X1-Y2-Z2-CHAMFER': (((1,1),(1,2),(0,2)),2.359515,0.015460), \
		'X1-Y2-Z2-TWINFILLET': (((1,1),(1,3)),2.145295,0.024437), \
		'X1-Y3-Z2-FILLET': (((1,1),(1,2),(0,2)),2.645291,0.014227), \
		'X1-Y4-Z1': (((1,1),(1,3)),3.14,0.019), \
		'X2-Y2-Z2-FILLET': (((1,1),(1,2),(0,2)),2.496793,0.018718) \
		} #brickOrientations = (((side, roll), ...), rotX, height)

#color bricks
colorList = ['Gazebo/Indigo', 'Gazebo/Gray', 'Gazebo/Orange', \
		'Gazebo/Red', 'Gazebo/Purple', 'Gazebo/SkyBlue', \
		'Gazebo/DarkYellow', 'Gazebo/White', 'Gazebo/Green']

brickList = list(brickDict.keys())
counters = [0 for brick in brickList]

lego = [] 	#lego = [[name, type, pose, radius], ...]

#get model path
def getModelPath(model):
	pkgPath = rospkg.RosPack().get_path(package_name)
	return f'{pkgPath}/obj_models/{model}/model.sdf'

class PoseError(Exception):
	pass



#functiont to spawn model
def spawn_model(model, pos, name=None, ref_frame='world', color=None):
	# world frame is the table frame
	if name is None:
		name = model
	
	model_xml = open(getModelPath(model), 'r').read()
	if color is not None:
		model_xml = changeModelColor(model_xml, color)

	spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
	return spawn_model_client(model_name=name, 
	    model_xml=model_xml,
	    robot_namespace='/foo',
	    initial_pose=pos,
	    reference_frame=ref_frame)

#support function delete bricks on table
def delete_model(name):
	delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	return delete_model_client(model_name=name)

#support functon spawn bricks
def spawnaLego(brickType=None, rotated=False, posX=None):
	brickIndex = brickDict[brickType][0]
	name = f'{brickType}_{counters[brickIndex]+1}'
	print(f'spawn object name is {name}')

	print(f'posX  {posX}')
	spawn_model(brickType, posX, name, spawn_name)
	lego.append((name, brickType, posX, 0.001))
	counters[brickIndex] += 1

#main function setup area and level manager
def setUpArea(livello=None, selectBrick=None): 	
	# position of bluebox
	x = 0.4
	y = -0.1
	# difference to bluebox
	dx = 0.034
	dy = 0.03
	#screating spawn area
	spawn_model(spawn_name, Pose(Point(*spawn_pos),None) )

	rot = Quaternion(*quaternion_from_euler(0, 0, 6*math.pi/2))
	point = Point(x, y, 0.2)
	pose_blueBox = Pose(point, rot)

	rot = Quaternion(*quaternion_from_euler(0, 0, 0))
	point = Point(x-dx, y-dy, 0.3)
	# point = Point(x,y,0.3)
	pose_cuboid = Pose(point, rot)

	rot = Quaternion(*quaternion_from_euler(0, 0, 0))
	point = Point(x-dx, y+dy, 0.3)
	pose_octagon = Pose(point, rot)

	rot = Quaternion(*quaternion_from_euler(0, 0, 0))
	point = Point(x+dx, y+dy, 0.3)
	pose_parallelogramm = Pose(point, rot)

	rot = Quaternion(*quaternion_from_euler(0, 0, 0))
	point = Point(x+dx, y-dy, 0.3)
	pose_star = Pose(point, rot)

	x = 0
	y = -0.1
	rot = Quaternion(*quaternion_from_euler(0, 0, math.pi/2))
	point = Point(x, y, 0.2)
	pose_grayBox = Pose(point, rot)


	try:
		spawnaLego('cuboid', False, pose_cuboid)
		spawnaLego('blueBox', False, pose_blueBox)
		spawnaLego('grayBox', False, pose_grayBox)
		spawnaLego('octagon', False, pose_octagon)
		spawnaLego('parallelogram', False, pose_parallelogramm)
		spawnaLego('star', False, pose_star)
	except PoseError as err:
		print("[Error]: no space in spawning area")
		pass
		
	print(f"Added {len(lego)} bricks")

if __name__ == '__main__':

	readArgs()

	try:
		if '/gazebo/spawn_sdf_model' not in rosservice.get_service_list():
			print("Waining gazebo service..")
			rospy.wait_for_service('/gazebo/spawn_sdf_model')
		
		#starting position bricks
		setUpArea(level, selectBrick)
		print("--------------------------------------------------------------")
		print("All done. Ready to start.")
	except rosservice.ROSServiceIOException as err:
		print("No ROS master execution")
		pass
	except rospy.ROSInterruptException as err:
		print(err)
		pass
	except rospy.service.ServiceException:
		print("No Gazebo services in execution")
		pass
	except rospkg.common.ResourceNotFound:
		print(f"Package not found: '{package_name}'")
		pass
	except FileNotFoundError as err:
		print(f"Model not found: \n{err}")
		print(f"Check model in folder'{package_name}/obj_models'")
		pass
