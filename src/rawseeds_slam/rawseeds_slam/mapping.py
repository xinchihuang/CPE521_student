"""ROS-independent occupancy map helper. Students change the pose estimator."""
import math
import numpy as np


def compose(a,b):
    x,y,t=a; u,v,w=b; c,s=math.cos(t),math.sin(t)
    return np.array([x+c*u-s*v,y+s*u+c*v,t+w])


def inverse(a):
    x,y,t=a; c,s=math.cos(t),math.sin(t)
    return np.array([-c*x-s*y,s*x-c*y,-t])


def ray(x0,y0,x1,y1):
    dx,dy=abs(x1-x0),-abs(y1-y0)
    sx,sy=1 if x0<x1 else -1,1 if y0<y1 else -1
    err=dx+dy
    while True:
        yield x0,y0
        if x0==x1 and y0==y1: break
        e=2*err
        if e>=dy: err+=dy; x0+=sx
        if e<=dx: err+=dx; y0+=sy


class Occupancy:
    def __init__(self,resolution=.1,size=2400,origin=(-100.,-100.)):
        self.resolution=resolution;self.size=size;self.origin=np.array(origin)
        self.logodds=np.zeros((size,size),np.float32)
        self.seen=np.zeros((size,size),bool)

    def cell(self,p):
        return np.floor((np.asarray(p)-self.origin)/self.resolution).astype(int)

    def integrate(self,pose,ranges,angle_min,increment,max_range=30.):
        start=self.cell(pose[:2]); inside=lambda p:0<=p[0]<self.size and 0<=p[1]<self.size
        if not inside(start): return
        for i,r in enumerate(ranges):
            if not np.isfinite(r) or r<.05: continue
            a=pose[2]+angle_min+i*increment; d=min(float(r),max_range)
            end=self.cell(pose[:2]+d*np.array([math.cos(a),math.sin(a)]))
            for x,y in ray(*start,*end):
                if not inside((x,y)): break
                hit=(x==end[0] and y==end[1] and r<max_range)
                self.logodds[y,x]=max(-5.,min(5.,float(self.logodds[y,x])+(.85 if hit else -.4)))
                self.seen[y,x]=True

    def values(self):
        result=np.full(self.logodds.shape,-1,np.int8)
        result[self.seen]=np.rint(100/(1+np.exp(-self.logodds[self.seen]))).astype(np.int8)
        return result
