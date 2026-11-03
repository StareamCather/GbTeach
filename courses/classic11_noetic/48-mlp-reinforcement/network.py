"""A tiny tanh MLP, explicit backpropagation, no ML runtime dependency."""
import numpy as np


class MLP:
    def __init__(self,seed=7):
        rng=np.random.default_rng(seed)
        self.W1=rng.normal(0,.25,(2,12)); self.b1=np.zeros(12)
        self.W2=rng.normal(0,.1,(12,1)); self.b2=np.zeros(1)

    def forward(self,X):
        X=np.atleast_2d(X); h=np.tanh(X@self.W1+self.b1)
        return (h@self.W2+self.b2).reshape(-1)

    def backward(self,X,output_derivative,rate,ascent=False):
        X=np.atleast_2d(X); h=np.tanh(X@self.W1+self.b1)
        d=np.asarray(output_derivative).reshape(-1,1)/len(X)
        dh=(d@self.W2.T)*(1-h*h)
        gradients=[X.T@dh,dh.sum(axis=0),h.T@d,d.sum(axis=0)]
        norm=np.sqrt(sum(float((g*g).sum()) for g in gradients))
        scale=min(1.,5./max(norm,1e-12))
        for key,g in zip(('W1','b1','W2','b2'),gradients):
            setattr(self,key,getattr(self,key)+(1 if ascent else -1)*rate*scale*g)
        return norm

    def save(self,path): np.savez(path,**{k:getattr(self,k) for k in ('W1','b1','W2','b2')})
    def load(self,path):
        with np.load(path,allow_pickle=False) as data:
            for key in ('W1','b1','W2','b2'): setattr(self,key,data[key].copy())
        return self
