from numpy import genfromtxt
import numpy as np
import matplotlib.pyplot as plt
#import sys
#sys.path.append('/home/kt-fitz/act3d_ws/src/mda_act3d_cart/src')

from multiprocessing import Pool
DIR = "/home/kt-fitz/data/"
#MASTERFILE = 
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

def splitter(sub,f):
    oldfile = DIR+"s"+sub+filetypes[f]
    labels = DIR+"raw/image-testing-order-s"+sub+".csv"
    session = int(oldfile[oldfile.rfind('set')+3:oldfile.rfind('-cl')])
    asstlab = oldfile[oldfile.rfind('_set')-1:oldfile.rfind('_set')]
    metrics = []
    tf=10.
    data = genfromtxt(oldfile,delimiter=',',dtype=float)
    data = np.delete(data,0,0)
    dlabel = genfromtxt(labels,delimiter=',',dtype=str)
    NumSamps = np.shape(data)[0]
    #print NumSamps
    time = [row[1] for row in data]
    #x = [row[3] for row in data]
    #dth = [row[4] for row in data]
    dT = (time[6]-time[5])
    intIndex = int(round(tf/dT +2))
    jj=0
    trialnum = 0
    while jj<NumSamps:
        j=0
        trialflag=True
        init=jj
        endstate = init+1001
        #successtemp = 0.0
        #balancecounter = 0.0
        x =[];y=[];
        while trialflag == True:
            j=j+1
            jj=jj+1
            try:
                state = [data[jj,5],data[jj,6],data[jj,7],data[jj,8]]
            except:
                state = [data[jj-1,5],data[jj-1,6],data[jj-1,7],data[jj-1,8]]
            x.append(state[0])
            y.append(state[1])
            #print jj
            if jj==(NumSamps) or time[jj]<time[jj-1]:
                trialflag=False; trialnum=trialnum+1; #print "Trial ", trialnum
            #elif abs(state[2])<= 0.0001 and abs(state[3])<=0.0001 and endstate == init+1002: 
                #endstate = jj; 
                   
        #print j
        end = init+j-1
        trialfull= data[init:end]
        trialcrop = data[init:endstate]
        paa = np.mean(data[init:end,-1])
        x = np.array(x)
        y=np.array(y)
        #[metric1,metric2] =importedfuction(x,y)
        if endstate>=NumSamps: endstate = jj-1
        if session==1:
            metrics.append(np.hstack((int(sub),images[dlabel[trialnum,1]],session,
                                      int(dlabel[trialnum,2]),assist[asstlab],time[endstate],
                                      300,paa)))
        elif session==2:
            imglab =oldfile[oldfile.rfind(sub+"_")+3:oldfile.rfind('_set')-2]
            metrics.append(np.hstack((int(sub),images[imglab],session,
                                      (session*10)+trialnum-10,assist[asstlab],time[endstate],
                                      300,paa)))
        #print "Trial ",trialnum,"trial", dlabel[trialnum,0], dlabel[trialnum,1]
        #plt.scatter(x,y)
        #plt.show()
        
    
    metrics = np.array(metrics)
    #norm = np.ones_like(metrics[0,:])
    #norm[0] = max(np.array(metrics)[:,0])
    #print norm
    metlabels="subject,image,set,trial,assistance,TimeUsed,KLdiv,paa"
    np.savetxt(oldfile[:oldfile.rfind('-c')] + "-metrics.csv",metrics,fmt="%9.6f",delimiter=",",header=metlabels,comments='')
    return #norm[0]



for subj in range(2,28):
    print"PROCESSING SUBJECT", subj
    for f in range(1,2):
        try:
            splitter(str(subj).zfill(2),f)
        except:
            print "Could not find s"+str(subj).zfill(2)+filetypes[f]

"""    
p=Pool(6)
list1 = [nomda[subj] for subj in nomda if nomda.get(subj) and subj<42]
p.map(splitter,list1)
list2 = [mda[subj] for subj in mda if mda.get(subj)]
p.map(splitter,list2)
#list3 = [nomda2[subj] for subj in nomda2 if nomda2.get(subj)]
#p.map(splitter,list3)
"""
"""
for subj in xrange(1,28):
    print "Subject", subj, " is processing."
    if nomda.get(subj):
        index=splitter(nomda[subj],0)
    if mda.get(subj):
        index=splitter(mda[subj],0)
    if nomda2.get(subj):
        index=splitter(nomda2[subj],0)
index=splitter(nomda[51],0)
#index=splitter("/home/kt-fitz/data/ref/s1-noMDA-expert_trep.csv",0)
"""
