import csv
import numpy as np
import random
from feature_extraction import FeatureExtractor

random.seed(0)

class Dataset:
    def __init__(self, filepath=None):
        self.filepath = filepath

    def load_data(self):
        with open(self.filepath, mode='r', newline='') as csvfile:
            data = csv.reader(csvfile, delimiter=',')
            data_matrix = []
            for row in data:
                data_vec = [float(x) for x in row[:-1]]
                label = int(row[-1])
                train_feats = FeatureExtractor(data_vec,label)
                data_matrix.append(train_feats)
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

    def get_cross_validation_folds(self, num_folds=5, fold_len=0.75):
        
        dataset_len = len(self.data_matrix)

        test_data = self.data_matrix[int(dataset_len*fold_len): , :]
        test_label = self.labels[int(dataset_len*fold_len):]

        data_folds = []
        data_folds_labels = []
        for i in range(num_folds): 
            start_idx = int(dataset_len * i * fold_len / num_folds)
            end_idx = int(dataset_len * (i + 1) * fold_len / num_folds)
            
            data_folds.append(self.data_matrix[start_idx:end_idx, :])
            data_folds_labels.append(self.labels[start_idx:end_idx])


        return data_folds, data_folds_labels, test_data, test_label
    
    def get_train_data(self):
        return self.train_data, self.train_labels
    
    def get_test_data(self):
        return self.test_data, self.test_labels
    def get_data(self):
        return self.data_matrix, self.labels
    

class KNN:
    def __init__(self, k=0):
        self.k = k
        self.neighbours=None
        self.eval = True

    def load_data(self,data,label):
        self.data = data
        self.label = label

    def evaluate(self, eval = True):
        self.eval=eval

    def predict(self,x):
        if self.eval:
            self.input_feature_vec=x
        else:
            self.input_feature_vec = FeatureExtractor(x)
        self.find_nearest_neighbours()

    def set_distance(self, dist_name):
        if dist_name=="euclidean":
            self.dist_func = lambda x,y : np.linalg.norm(x-y, ord=2)
        elif dist_name=="manhattan":
            self.dist_func = lambda x,y : np.linalg.norm(x-y, ord=1)

    def find_nearest_neighbours(self):
        dist_list = []
        dist_list = [self.dist_func(point, self.input_feature_vec) for point in self.data]
        distance = sorted(zip(dist_list, self.label), key=lambda x: x[0])
        self.neighbours = [distance[i][1] for i in range(self.k)]

    
    def get_label(self):
        return max(set(self.neighbours), key= self.neighbours.count)


class LogisticRegression():
    ## 
    # Work is on...
    ## 
    def __init__(self):
        pass

    def loadData(self):
        pass

    def train(self):
        pass

    def softmax(self, score):
        return np.exp(score)/np.sum(self.scores)

    def prediction(self):
        weight = []
        scores = []
        # for i in range(len(classes)):
        #     scores.append(weight[i].T@self.x + self.bias[i])
        probabilities= []
        for score in self.scores:
            probabilities.append(np.exp(score)/np.sum(scores))
        
        return max(probabilities) # index of this is the answer. 