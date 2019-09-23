from  optitrack.csv_reader import Take


def write_to_file(filename, _sm_m):
	# sm = data.single_markers['Hand:Thumb']
	outF = open(filename, "w")
	for i in range(len(_sm_m.times)):
		if _sm_m.positions[i] == None:
			outF.write(str(_sm_m.times[i]) + ",,," )
  		else:
  			outF.write(str(_sm_m.times[i]) + "," +  str(_sm_m.positions[i][0]) + "," + str(_sm_m.positions[i][1]) + "," + str(_sm_m.positions[i][2]) )
  		outF.write("\n")
	outF.close()

reader = Take()
data = reader.readCSV("data/some_new_data.csv", verbose=True)
#print(data.rigid_bodies['RigidBody 1'].positions)
#print(data.single_markers['Hand:Middle'].positions)

# print(data.single_markers['Hand:Thumb'].times)
# print(data.single_markers['Hand:Thumb'].positions[0])

sm_t = data.single_markers['Hand:Thumb']
sm_m = data.single_markers['Hand:Middle']
sm_i = data.single_markers['Hand:Index']


write_to_file("hand_thumb.csv", sm_t)
write_to_file("hand_middle.csv", sm_m)
write_to_file("hand_index.csv", sm_i)

rb = data.rigid_bodies['RigidBody 1']
write_to_file("hand.csv", rb)
