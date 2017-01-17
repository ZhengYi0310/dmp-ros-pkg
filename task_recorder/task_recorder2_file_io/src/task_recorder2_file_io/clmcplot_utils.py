'''
Created on Dec 12, 2011

@author: righetti
'''

import os
import numpy as np
import struct

class ClmcFile:
    #defines cols, rows, freq, names and units
    def __init__(self):
        self.freq = 0
        self.names = {}
        self.units = []
        self.data = None
    
    def read(self, filename):
        if os.path.exists(filename) == False:
            print 'Error the file >' + filename + '< does not exist.'
            return False
        
        #read the file
        with open(filename, 'rb') as my_file:
            #get the header right
            temp = my_file.readline().split()
            cols = int(temp[1])
            rows = int(temp[2])
            self.freq = float(temp[3])
    
            #get the names and units
            temp = my_file.readline().split()
            for i in range(0, cols) :
                self.names[(temp[2*i])] = i
                self.units.append(temp[2*i+1])
            #get all of the data
            self.data = np.array(struct.unpack('>'+'f'*cols*rows,my_file.read(4*cols*rows))).reshape(rows, cols).transpose()
            return True
        
    def getVariables(self, names):
        data = np.array([[]])
        if(isinstance(names, str)):
            if names in self.names:
                data = self.data[self.names[names]]
            else:
                print 'ERROR: ' + names + ' has an invalid type. Must be a string.'
                return None, False
        else:    
            for i, name in enumerate(names):
                if name in self.names:
                    if i == 0:
                        data = self.data[self.names[name]]
                    else:
                        data = np.vstack((data, self.data[self.names[name]]))
                else:
                    print 'ERROR: ' + name + ' is not a valid name.'
                    return None, False

        return data.transpose(), True

filename = '/home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive_utilities/test/data/test_rollout_joint_reproduction_test_2.clmc'
clmc_parser = ClmcFile()

clmc_parser.read(filename)
names = "R_WAA"
print clmc_parser.data[1]
