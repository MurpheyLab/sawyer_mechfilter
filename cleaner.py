import csv
import numpy as np
from numpy import genfromtxt

DIR = "/home/kt-fitz/data/"


def crop_data(oldfile):
    filename =oldfile[:oldfile.rfind('set')+5]+"-clean.csv"
    datanames = genfromtxt(oldfile,delimiter=',',dtype=str)
    columns = datanames[0,:]
    with open(filename, 'w') as csvfile:
        testwriter = csv.writer(csvfile,delimiter = ',')
        testwriter.writerow(columns)

    data = genfromtxt(oldfile,delimiter=',',dtype=float)
    data = np.delete(data,0,0)
    for i in range(0,np.shape(data)[0]):
        if data[i,1]<=10.0:
            row = data[i,:]
            with open(filename,'a') as csvfile:
      	        testwriter = csv.writer(csvfile,delimiter=',')
                testwriter.writerow(row)
    return

filetypes = {
    1:"_c_set01_mda.csv",
    2:"_apple_v_set02_mda.csv",
    3:"_apple_h_set02_mda.csv",
    4:"_apple_p_set03_mda.csv",
    5:"_banana_v_set02_mda.csv",
    6:"_banana_h_set02_mda.csv",
    7:"_banana_p_set03_mda.csv",
    8:"_house_v_set02_mda.csv",
    9:"_house_h_set02_mda.csv",
    10:"_house_p_set03_mda.csv",
    11:"_umbrella_v_set02_mda.csv",
    12:"_umbrella_h_set02_mda.csv",
    13:"_umbrella_p_set03_mda.csv",}

for subj in range(2,28):
    substr = str(subj).zfill(2)
    print "Cleaning data of subject " substr
    for f in range(1,14):
        try:
            crop_data(DIR+"s"+substr+filetypes[f])
        except:
            print "No file of type "+"s02"+filetypes[f]