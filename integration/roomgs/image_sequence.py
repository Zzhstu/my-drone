"""Bounded-memory RGB access for large, already checksum-verified captures."""
import numpy as np
from PIL import Image


class CapturedImages:
    def __init__(self,paths,width,height):
        self.paths=paths;self.width=width;self.height=height
        self.shape=(len(paths),height,width,3)

    def __getitem__(self,index):
        with Image.open(self.paths[index]) as source:
            image=source.convert('RGB').resize((self.width,self.height),Image.Resampling.LANCZOS)
            return np.asarray(image,dtype=np.uint8)
