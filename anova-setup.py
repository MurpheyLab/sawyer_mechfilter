import csv
from numpy import genfromtxt
import numpy as np
DIR = "/home/kt-fitz/data/"


filetypes = {
    1:"_c_set01",
    2:"_apple_v_set02",
    3:"_apple_h_set02",
    4:"_apple_p_set03",
    5:"_banana_v_set02",
    6:"_banana_h_set02",
    7:"_banana_p_set03",
    8:"_house_v_set02",
    9:"_house_h_set02",
    10:"_house_p_set03",
    11:"_umbrella_v_set02",
    12:"_umbrella_h_set02",
    13:"_umbrella_p_set03",}

group = {2:2,3:1,4:1,5:2,6:1,7:2,9:1,10:2,11:1,12:2,13:2,14:1,15:2,18:2,19:2,20:2,21:2,22:1,
         23:1,24:1,25:1,26:1,27:2}

newfile=DIR+"alldata2.csv"
columns = ["subject","image","set","trial","assistance","TimeUsed","meand","vard","meandTU","vardTU","maxdist","dkl","paa"]
with open(newfile, 'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter = ',')
    testwriter.writerow(columns)
for subj in range(2,28):
    for f in range(1,14):
        try:
            singlefile = DIR+"s"+str(subj).zfill(2)+filetypes[f]+"-metrics.csv"
            data = genfromtxt(singlefile,delimiter=',',dtype=float)
            data = np.delete(data,0,0)
            for i in range(0,np.shape(data)[0]):
                row = data[i,:]
                row[4]=group[subj]
                with open(newfile,'a') as csvfile:
      	            testwriter = csv.writer(csvfile,delimiter=',')
                    testwriter.writerow(row)
        except:
            print "ERROR Processing Subject ", subj, " File ",f
   