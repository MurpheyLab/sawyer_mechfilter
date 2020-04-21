import numpy as np
import random
import math
import cv2
from matplotlib import pyplot as plt 
THRESHOLD = 130
DT=1./100.

class imagewalls:
    def __init__(self,imageName,K):
        imagetemp = cv2.imread(imageName,cv2.IMREAD_GRAYSCALE)
        self.img = 255-imagetemp;
        self.img = cv2.blur(self.img,(50,50));  
        self.width = self.img.shape[1]; #print self.width
        self.height = self.img.shape[0]
        self.ps_i = [None]*K
        self.domainsamps = np.zeros([2,K])
        self.sigma = 0.01*np.eye(2)
        self.resample()
        
    def pixelcheck(self,y,x):
        black = False
        if(x<=0 or x>=self.width or y<=0 or y>=self.height):
            do=[]#do nothing
        elif((255-self.img.item(int(y),int(x)))<THRESHOLD):
            black=True
        return black
    
    def euclidist (self,x,y,xt,yt):
        d=((x-xt)**2.+(y-yt)**2.)**0.5
        return d
    
    def findnearest(self,x,y):
        nearpixel = [0.,0.,0.]
        j=1
        found = False
        while(found==False and j<=self.width):
            dist =self.width*np.ones(8*j)
            pixels = np.zeros((8*j,2))
            for m in range(-j,j+1):
                if(self.pixelcheck(y+m,x-j)):
                    found = True
                    dist = np.roll(dist,1); dist[0] = self.euclidist(x,y,x-j,y+m)
                    pixels=np.roll(pixels,1,axis=0);
                    pixels[0,0]=x-j; pixels[0,1]=y+m
                if(self.pixelcheck(y+m,x+j)):
                    found = True;
                    dist = np.roll(dist,1);dist[0] = self.euclidist(x,y,x+j,y+m)
                    pixels = np.roll(pixels,1,axis=0); 
                    pixels[0,0]=x+j; pixels[0,1]=y+m;
            for n in range(-j+1,j):
                if(self.pixelcheck(y-j,x+n)):
                    found=True;
                    dist =np.roll(dist,1);dist[0] = self.euclidist(x,y,x+n,y-j)
                    pixels = np.roll(pixels,1,axis=0)
                    pixels[0,0]=x+n; pixels[0,1]=y-j
                if(self.pixelcheck(y+j,x+n)):
                    found = True
                    dist = np.roll(dist,1);dist[0] = self.euclidist(x,y,x+n,y+j)
                    pixels = np.roll(pixels,1,axis=0)
                    pixels[0,0]=x+n; pixels[0,1]=y+j
            nearpixel[2]=np.amin(dist)
            minindex = np.where(nearpixel[2])[0][0]
            nearpixel[0]=pixels[minindex,0]
            nearpixel[1]=pixels[minindex,1]  
            j=j+1
        #print np.where(nearpixel[2])[0]
        return nearpixel
    def resample(self):
        #Choose K samples over the domain [[-L1,L1],[-L2,L2]]
        localps = [None]*len(self.ps_i)
        #setup CDF of image for inverse transform sampling
        epsilon = 10**-1
        imgCDF = [None]*(self.width*self.height) 
        imgNorm = (cv2.mean(self.img)[0]+epsilon)*self.width*self.height
        imgCDF[0] = (self.img.item(0,0)+epsilon)/imgNorm; 
        for m in range(0,self.height):
            for n in range (0,self.width):
                try: imgCDF[(m*self.height)+n+1] = imgCDF[(m*self.height)+n]+(self.img.item(m,n)+epsilon)/imgNorm;
                except: print imgCDF[-1]
        #end CDF setup  
        for n in range(0,len(self.ps_i)):
            k = random.uniform(0,1); 
            i = 0;
            while imgCDF[i+1]<=k: i=i+1;
            self.domainsamps[1,n] = round(float(i)/float(self.width))#/self.width-0.5
            self.domainsamps[0,n] = round(float(i)%float(self.width))#/self.width-0.5
            localps[n] = self.phid(self.domainsamps[:,n]);
        self.ps_i=[localps[t]/sum(localps) for t in range(0,len(localps))]#normalise the discrete pdf over the samples
        
    def phid(self,x):
        intensity = self.img.item(int(x[1]),int(x[0]))
        totalInt = cv2.mean(self.img)[0]*self.width*self.height
        intensity = intensity/totalInt
        return intensity
    
    def qs_disc(self,X):
        qs_i=[None]*len(self.ps_i)
        for n in range(0,len(self.ps_i)):
            qs_i[n] = 0.;
            for j in range(0,np.shape(X)[1]):
                sj = np.array(self.domainsamps[:,n]-X[:,j])/self.height;#print sj
                #print((-0.5*np.transpose(sj).dot(np.linalg.inv(self.sigma)).dot(sj)))
                qs_i[n]+=DT*math.exp(-0.5*np.transpose(sj).dot(np.linalg.inv(self.sigma)).dot(sj))
        #print qs_i
        try:qs_final=[qs_i[t]/sum(qs_i) for t in range(0,len(qs_i))]#normalise the discrete pdf over the samples
        except:
            #print qs_i
            qs_i=[x+10**-2 for x in qs_i]
            qs_final=[qs_i[t]/sum(qs_i) for t in range(0,len(qs_i))]
            print "There's a problem"
        dklist = [self.ps_i[t]*math.log(qs_final[t]) for t in range(0,len(qs_i))]
        return -sum(dklist);
        
"""
imgName="src/apple.png";
walltest=imagewalls(imgName,20);
plt.imshow(255-walltest.img, cmap = 'gray', interpolation = 'bicubic')
plt.scatter(walltest.domainsamps[0,:],walltest.domainsamps[1,:])
#plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
plt.show()
x=[1,2,4,5.5,7]
y=[1,2,4,5,7]
xtot = np.array([x,y])
print walltest.qs_disc(xtot)
"""