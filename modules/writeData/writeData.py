

def writeData (data):
	#check if the input paramenter a list. 	
	if (1==1):
		with open('mydata.csv', 'a') as export:
			export.write(data)
			export.write("\n")