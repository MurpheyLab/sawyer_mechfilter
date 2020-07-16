from numpy import genfromtxt
import numpy as np
import matplotlib.pyplot as plt
import imgwalls as wall
from multiprocessing import Pool
import csv
import cv2
import random

DIR = "/home/kt-fitz/data/"
SAMPS = 65
images = {
    "apple":1,
    "banana":2,
    "house":3,
    "umbrella":4,
    "discard":0}

assist = {
    "c":0,
    "h":1,
    "v":2,
    "p":0}

filetypes = {
    1:"_c_set01-clean.csv",
    2:"_apple_v_set02-clean.csv",
    3:"_apple_h_set02-clean.csv",
    4:"_apple_p_set03-clean.csv",
    5:"_banana_v_set02-clean.csv",
    6:"_banana_h_set02-clean.csv",
    7:"_banana_p_set03-clean.csv",
    8:"_house_v_set02-clean.csv",
    9:"_house_h_set02-clean.csv",
    10:"_house_p_set03-clean.csv",
    11:"_umbrella_v_set02-clean.csv",
    12:"_umbrella_h_set02-clean.csv",
    13:"_umbrella_p_set03-clean.csv",}

def splitter(input):
    sub=str(input[0]).zfill(2)
    f=input[1]
    scale =2200.
    offset = 0.
    if f ==2 or f==5 or f==8 or f==11: scale =1.
    if f==3 or f==6 or f==9 or f==12: offset = 0.5
    oldfile = DIR+"cpp2/s"+sub+filetypes[f];
    labels = DIR+"raw/image-testing-order-s"+sub+".csv"
    session = int(oldfile[oldfile.rfind('set')+3:oldfile.rfind('-cl')])
    asstlab = oldfile[oldfile.rfind('_set')-1:oldfile.rfind('_set')]
    newfile = oldfile[:oldfile.rfind('-c')] + "-metrics.csv"
    metrics = []
    tf=10.
    datanames = genfromtxt(oldfile,delimiter=',',dtype=str)
    columns = datanames[0,:]
    del datanames
    qind = np.where(columns=='x')[0][0]
    dqind = np.where(columns=='field.dq0')[0][0]
    paaind = np.where(columns=='field.accept')[0][0]
    dind =np.where(columns=='pixdist')[0][0]
    klind =np.where(columns=='dkl')[0][0]
    data = genfromtxt(oldfile,delimiter=',',dtype=float)
    data = np.delete(data,0,0)
    dlabel = genfromtxt(labels,delimiter=',',dtype=str)
    NumSamps = np.shape(data)[0]
    time = [row[1] for row in data]
    dT = (time[6]-time[5])
    intIndex = int(round(tf/dT +2))
    jj=0
    trialnum = 0
    columns = ["subject","image","set","trial","assistance","TimeUsed","meand","vard","meandTU","vardTU","maxdist","dkl","paa","ratingcode"]
    with open(newfile, 'w') as csvfile:
        testwriter = csv.writer(csvfile,delimiter = ',')
        testwriter.writerow(columns)
    while jj<NumSamps:
        j=0
        trialflag=True
        init=jj
        endstate = init+1001
        kldiv=1000.
        distcounter = [];dklist = [];
        x =[];y=[];v=[100.]*10
        if session ==1:
            imglab = dlabel[trialnum+1,1]
            tlab = float(dlabel[trialnum+1,2]);
        elif session ==2 or session ==3:
            imglab =oldfile[oldfile.rfind(sub+"_")+3:oldfile.rfind('_set')-2]
            tlab = (session*10)+trialnum-9
        mir=1.
        refl=0.
        if input[0]<=13:
            if imglab=='apple' or imglab=='umbrella':
                mir=-1.
                refl=2200.
        while trialflag == True:
            j=j+1
            jj=jj+1
            try:
                x.append(data[jj,qind]); y.append(data[jj,qind+1])
                v.append((data[jj,dqind]**2+data[jj,dqind+1]**2)**0.5)
                distcounter.append(data[jj,dind])
                dklist.append(data[jj,klind])
            except:
                x.append(data[jj-1,5]); y.append(data[jj-1,6])
                v.append((data[jj-1,dqind]**2+data[jj-1,dqind+1]**2)**0.5)
            
            v=v[1:]; 
                        
            if jj==(NumSamps) or time[jj]<time[jj-1]:
                trialflag=False; trialnum=trialnum+1; #print "Trial ", trialnum
                if endstate==init+1001: endstate=jj-1
            elif sum(v)/len(v)<= 10**(-2) and endstate == init+1001 and time[jj]>6.0: 
                endstate = jj-10; 
                   
        end = init+j-1
        trialfull= data[init:end]
        trialcrop = data[init:endstate]
        paa = np.mean(data[init:end,paaind])
        x=x[:-1]#;x = np.array(x)
        y=y[:-1]#;y=np.array(y)
        q=np.array([x,y])
        dkl=dklist[400]
        #if dkl==inf
        maxdist = max(distcounter)
        dist = sum(distcounter[100:300])/len(distcounter[100:300])
        distvar = np.var(distcounter)
        distcrop=distcounter[0:(endstate-init)]
        distTU = sum(distcrop)/len(distcrop)
        distvarTU = np.var(distcrop)
        if endstate>=NumSamps: endstate = jj-1
        imcode = random.randint(0,999999)
        imname = DIR +"imgratings/"+str(imcode).zfill(6)+".png"
        row =[input[0],images[imglab],session, tlab,assist[asstlab],time[endstate],dist,distvar,
              dist,distvarTU,maxdist,dkl,paa,imcode]
              #distTU,distvarTU,maxdist,dkl,paa]
        
        with open(newfile,'a') as csvfile:
      	        testwriter = csv.writer(csvfile,delimiter=',')
                testwriter.writerow(row)
        
        #print imglab
        if imglab!="discard":
            img = cv2.imread("src/"+imglab+".png",cv2.IMREAD_GRAYSCALE)
            x1=(np.array(x[0:500])+0.5)*2200
            if imglab=="umbrella" or imglab=="apple":x1=2200-x1
            x2=(np.array(y[0:500])+0.5)*2200
            plt.figure()
            plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
            plt.plot(x1,x2,'o',color='green')#plt.scatter(x1,x2,marker='o')
            plt.savefig(imname)
            #plt.show()
            plt.close()
    
    #metrics = np.array(metrics)
    #metlabels="subject,image,set,trial,assistance,TimeUsed,meand,vard,meandTU,vardTU,maxdist,dkl,paa"
    #np.savetxt(oldfile[:oldfile.rfind('-c')] + "-metricsv2.csv",metrics,fmt="%9.6f",delimiter=",",header=metlabels,comments='')
    return 

def func(x):
    #print "Processing Subject ", x[0], " File ",x[1]
    try:
        splitter(x)
    except:
        print "Could not find s"+str(x[0]).zfill(2)+filetypes[x[1]]
    return
sublist = [2,3,4,8,9,10,11,12,13,14,15,16,17]#20,27]
list1 = [[subj,f] for f in range(1,14) for subj in range(13,28)]
p=Pool(8)
p.map(func,list1)


