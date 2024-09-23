# Load the data, create a split, and return the particular sample. 
# TO DO: 
# Build / Load model
# Train and test
# Save in whichever format#

import csv
import numpy as np
import random

random.seed(1)

class Dataset:

    def __init__(self, filepath=None):
        self.filepath = filepath

    def load_data(self):
        with open(self.filepath, mode='r', newline='') as csvfile:
            data = csv.reader(csvfile, delimiter=',')
            data_matrix = []
            for row in data:
                feats = [float(x) for x in row[:-1]]
                label = [int(row[-1])]
                data_matrix.append(feats+label)

        data_matrix = np.array(data_matrix)
        random.shuffle(data_matrix)
        random.shuffle(data_matrix)

        self.labels = data_matrix[:, -1].reshape(-1)
        self.data_matrix = np.array(data_matrix[:,:-1])
    
    def train_test_split(self,train_len = 0.8):
        self.train_data = self.data_matrix[:int(train_len*len(self.labels)),:]
        self.train_labels = self.labels[:int(train_len*len(self.labels))]
        self.test_data = self.data_matrix[int(train_len*len(self.labels)):, :]
        self.test_labels = self.labels[int(train_len*len(self.labels)):]

    def get_train_data(self):
        return self.train_data, self.train_labels
    
    def get_test_data(self):
        return self.test_data, self.test_labels
    

class KNN:
    # Either build a KNN model from scatch or use scikit
    def __init__(self):
        pass


if __name__=="__main__":
    path = "data_features.csv"
    dataset = Dataset(path)
    dataset.load_data()
    dataset.train_test_split(0.75)
    train_data, train_labels = dataset.get_train_data()
    test_data, test_labels = dataset.get_test_data()
    print(len(train_data), len(train_labels))
    print(len(test_data), len(test_labels))
