import csv
import numpy as np
import random

TRAINNUM=20

filename = "image-testing-order.csv"
columns = ['trial_num','image','image_counter']
with open(filename, 'w') as csvfile:
    testwriter = csv.writer(csvfile,delimiter = ',')
    testwriter.writerow(columns)

images = {
    1:"apple",
    2:"banana",
    3:"house",
    4:"umbrella"}

trialnum = 1
imagecheck = np.zeros(4)
while np.sum(imagecheck)<40:
    image = random.randint(0,3)
    if imagecheck[image]<10:
    	row=[trialnum,images[image+1],imagecheck[image]+1]
        with open(filename,'a') as csvfile:
      	    testwriter = csv.writer(csvfile,delimiter=',')
            testwriter.writerow(row)
            trialnum+=1
        imagecheck[image]=imagecheck[image]+1  

imagecheck = np.zeros(4)
while np.sum(imagecheck)<4:
    image = random.randint(0,3)
    if imagecheck[image]<1:
        row=[str(trialnum)+"-"+str(trialnum+TRAINNUM+10),images[image+1],TRAINNUM+10]
        with open(filename,'a') as csvfile:
      	    testwriter = csv.writer(csvfile,delimiter=',')
            testwriter.writerow(row)
            trialnum+=TRAINNUM+10
        imagecheck[image]=imagecheck[image]+1   

file2 = "subject-testing-order.csv"
columns = ['subject_num','assistance']
group = {
    0:"virtual walls",
    1:"hybrid shared control"}

SAMPS = 42
acheck = np.zeros(2)
subnum = 1
while subnum<=SAMPS:
    assistance = random.randint(0,1)
    if acheck[assistance]<SAMPS/2:
        row = [subnum,group[assistance]]
        with open(file2,'a') as csvfile:
      	    testwriter = csv.writer(csvfile,delimiter=',')
            testwriter.writerow(row)
            subnum+=1
        acheck[assistance]=acheck[assistance]+1  

