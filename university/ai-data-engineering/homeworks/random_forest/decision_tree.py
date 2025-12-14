from util import entropy, information_gain, split_node
import numpy as np


class DecisionTree(object):
    def __init__(self):
        self.tree = {}  # you can use different data structure to build your tree
        self.depth = 0  # depth of the tree

    def train(self, X, y, max_depth=20):
        """
        This method trains decision tree (trains = construct = build)
        Args:
            X: data excluded target feature
            y: target feature

        Returns:

        NOTES:  You can add more parameter to build your algorithm, if necessary.
                You will have to use the functions from util.py
                Construct your tree using dictionary / or you can choose any other data structure.
                Each key should represent a property of your tree and value is corresponding value for your key
        IMPORTANT:  ADD RANDOMNESS
                    You should add randomness to your decision tree for random forest
                    At each node: select random 5 features from feature list (22 feature) and
                    compare the information gain of only 5 randomly selected features to select splitting attribute with the highest information gain
                    The selected random features should change at every node choice
        Example: you can think each node as Node object and node object should have some properties.
                A node should have split_value and split_attribute properties that give us the information of that node.
                (Below example is just an example, each tree should have more properties)
                Like: tree["split_value"] = split_value_you_find
                      tree["split_attribute"] = split_feature_with_highest_information_gain
        """
        self.depth += 1
        features = list(range(22))  # indexes for each feature
        info_gains = dict()
        node = dict()

        # randomly select five feature and compare their info gain
        for feature in np.random.choice(features, 5):
            if len(np.unique(X)) > 1:
                x_l, x_r, y_l, y_r = split_node(X, y, split_feature=feature)
                i_gain = information_gain(y, [y_l, y_r])
                info_gains[feature] = i_gain

        # select the feature with the highest info gain
        if len(info_gains) != 0:
            max_gain_feature = max(info_gains, key=info_gains.get)  # type: ignore
            info_gains.clear()

        # if entropy of the given y's are zero or max depth is reached
        # assign node to a leaf value (node["leaf"] = value) else
        # recursively do the same steps assigning node's key to the rule of split
        cond_to_split = (
            entropy(y) != 0
            and self.depth != max_depth
            and len(np.unique(X[:, max_gain_feature])) > 1
        )
        if cond_to_split:
            x_l, x_r, y_l, y_r = split_node(X, y, split_feature=max_gain_feature)
            if len(np.unique(x_l)) > 1:
                split_val = np.mean(X[:, max_gain_feature])
                node[f"{max_gain_feature}_ <{split_val}"] = self.train(
                    x_l, y_l, max_depth
                )
                node[f"{max_gain_feature}_ >{split_val}"] = self.train(
                    x_r, y_r, max_depth
                )
        else:
            # find majority labels
            node["leaf"] = np.bincount(y.astype(int)).argmax()

            # Normally this function returns the tree at the end of execution
            # (after recursion is done completely) but since tree of this object
            # is wanted to be set in this method we set tree to node at each end
            # of recursion. (normally setting self.tree = self.train() at elswhere
            # would be more efficient)
        self.tree = node
        self.depth -= 1
        return node

    def classify(self, record):
        """
        This method classifies the record using the tree you build and returns the predicted la
        Args:
            record: each data point

        Returns:
            predicted_label

        """
        tree = self.tree
        predicted_labels = np.zeros(len(record))
        for i, rec in enumerate(record):
            while "leaf" not in tree.keys():  # until leaf is reached
                # get feature and split value of node from node' key
                feature = int(list(tree.keys())[0].split("_")[0])
                split_val = float(list(tree.keys())[0].split("_")[1][2:])

                # proceed on the tree based on condition
                if rec[feature] < split_val:
                    tree = tree[f"{feature}_ <{split_val}"]
                else:
                    tree = tree[f"{feature}_ >{split_val}"]

            predicted_labels[i] = tree["leaf"]  # set label
            tree = self.tree  # restart from top of the tree
        return predicted_labels
