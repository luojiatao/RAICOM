#encoding: UTF-8
#!/usr/bin/env python2

class GrabParams(object):

	# get the results by calibration
	ratio =  0.194559
	
	# increase x_bias to move front, or decrease x_bias to move back
	x_bias = -9
	x_bias1 = 0
	

	# increase y_bias to move left, or decrease y_bias to move right
	y_bias = 37
	y_bias1 = 45

	#               	 (+x)front
	#                 	  ^
	#				 	  :
	#				  	  :
	#                 	  :
	# (+y)< ..............o..............(-y)right
	#					  :
	#					  :
	#					  :
	#					  :
	#					 (-x)

	# increase height_bias to move higher, or decrease height_bias to move lower
	height_bias = 145
	height_bias1 = 135
	

	grab_direct = "front"
	coords_ready = [201.5, -58.7, 225.7, -179.36, -0.47, -135.17]
	coords_ready1 = [195, -30, 245, -175, 0, -138]

	# grab_direct = "right"
	# if grab_direct == "right":
	# 	y_bias = -5
	# 	x_bias = 40
	# 	coords_ready = [-59.3, -181.2, 252.8, -178.51, 0.28, 135]
	GRAB_MOVE_SPEED = 20

	# show image and waitkey
	debug = True #True         

	# please do not change the parameter values below
	ONNX_MODEL = '/home/robuster/beetle_ai/scripts/beetle_obj.onnx'
	IMG_SIZE = 640
	done = False
	cap_num = 2
	usb_dev = "/dev/arm"
	baudrate = 115200

grabParams = GrabParams()

