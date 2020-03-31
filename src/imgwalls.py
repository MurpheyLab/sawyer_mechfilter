import numpy as np
import cv2
from matplotlib import pyplot as plt 
THRESHOLD = 130

class imagewalls:
    def __init__(self,imageName):
        self.img = cv2.imread(imageName,cv2.IMREAD_GRAYSCALE)
        self.width = self.img.shape[1]; #print self.width
        self.height = self.img.shape[0]
        
    def pixelcheck(self,y,x):
        black = False
        if(x<=0 or x>=self.width or y<=0 or y>=self.height):
            do=[]#do nothing
        elif(self.img.item(y,x)<THRESHOLD):
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
    
imgName="apple.png";
walltest=imagewalls(imgName);
testpt = np.random.randint(2200,size=2)
nearestpixel = walltest.findnearest(testpt[0],testpt[1])
print testpt
print nearestpixel
imagetemp = cv2.imread(imgName,cv2.IMREAD_GRAYSCALE)
plt.scatter(testpt[0],testpt[1])
plt.scatter(nearestpixel[0],nearestpixel[1],marker='x')
plt.imshow(imagetemp, cmap = 'gray', interpolation = 'bicubic')
#plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
plt.show()