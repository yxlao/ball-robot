import numpy as np
from sklearn import linear_model
from os.path import expanduser

clf = linear_model.LogisticRegression()

def get_accuracy(labels_predict, labels):
    return np.sum(labels_predict == labels) / float(len(labels))

home = expanduser("~")
ball_factors = np.loadtxt(home + '/robot/src/ball_detect/src/ball_factors.txt')
paper_factors = np.loadtxt(home + '/robot/src/ball_detect/src/paper_factors.txt')

# try:
all_factors = np.concatenate((ball_factors, paper_factors), axis=0)
all_labels = [1] * len(ball_factors) + [-1] * len(paper_factors)

clf.fit(all_factors, all_labels)
all_labels_predict = clf.predict(all_factors)

accuracy = get_accuracy(clf.predict(all_factors), all_labels)
print "imported with accuracy", accuracy
