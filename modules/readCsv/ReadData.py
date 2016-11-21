import time
import csv

def data (path = ""):
    #time.sleep (1)
    file=path
    f = open (file,"r" )
    lineList = f.readlines()
    f.close()
    lastRow=lineList[len(lineList)-1]
    myList = lastRow.split(",")
    del myList[0]
    print (myList)
   


