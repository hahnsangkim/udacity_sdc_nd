import pandas as pd
import numpy as np

with open('../data/sample-laser-radar-measurement-data-1.txt') as f:
    df = pd.read_table(f, sep='\t', header=None, lineterminator='\n')

print(df.shape[0])

#L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
#R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

fname = '../data/sample-laser-radar-measurement-data-1.txt'
class sf(object, fname):
    def __init__(self):
        self.px = 0
        self.py = 0
        self.rho = 0
        self.phi = 0
        self.rhod = 0
        self.timestamp = 0
        self.gtpx = 0
        self.gtpy = 0
        self.gtvx = 0
        self.gtvy = 0
        
        self.laser = False
        self.radar = False
        self.count = 0
        with open(fname) as f:
            self.df = pd.read_table(f, sep='\t', header=None, lineterminator='\n')
        
    def read(self):
        row = df.loc[self.count]
        i = 0
        if 'R' in row:
            self.radar = True
            self.laser = False
            self.rho = row[1]
            self.phi = row[2]
            self.rhod = row[3]
            i = 4
        if 'L' in row:
            self.radar = False
            self.laser = True
            self.px = row[1]
            self.py = row[2]
            i = 3
        self.timestamp = row[i]
        self.gtpx = row[i+1]
        self.gtpy = row[i+2]
        self.gtvx = row[i+3]
        self.gtvy = row[i+4]
        
    def getNumRows(self):
        return df.shape[0]

class ukf(object):
    def predict(self):
        