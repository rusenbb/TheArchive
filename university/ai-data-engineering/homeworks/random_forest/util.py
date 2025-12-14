import numpy as np


def entropy(class_y):
    """
    This method calculates entropy
    Args:
        class_y: list of class labels
    Returns:
        entropy: entropy value

    Example: entropy for [0,0,0,1,1,1] should be 1.
    """
    _, counts = np.unique(class_y, return_counts=True)
    probs = counts / len(class_y)
    entropy = (-(probs * np.log2(probs))).sum()
    return entropy


def information_gain(previous_y, current_y):
    """
    This method calculates information gain. In this method, use the entropy function you filled
    Args:
        y_before: the distribution of target values before splitting
        y_splitted: the distribution of target values after splitting

    Returns:
        information_gain

    Example: if y_before = [0,0,0,1,1,1] and y_splitted = [[0,0],[0,1,1,1]], information_gain = 0.4691

    """
    l = len(previous_y)
    e_before = entropy(previous_y)
    e_after = sum([entropy(sub_split) * len(sub_split) / l for sub_split in current_y])
    info_gain = e_before - e_after
    return info_gain


def split_node(X, y, split_feature):
    """
    This method implements binary split to your X and y.
    Args:
        X: dataset without target value
        y: target labels
        split_feature: column index of the feature to split on
        split_value: value that is used to split X and y into two parts

    Returns:
        X_left: X values for left subtree
        X_right: X values for right subtree
        y_left: y values of X values in the left subtree
        y_right: y values of X values in the right subtree

    Notes:  Implement binary split.
            You can use mean for split_value or you can try different split_value values for better Random Forest results
            Assume you are only dealing with numerical features. You can ignore the case there are categorical features.
            Example:
                Divide X into two list.
                X_left: where values are <= split_value.
                X_right: where values are > split_value.
    """

    if len(y) == len(X[:, split_feature]) == 1:
        return X, X, y, y
    split_value = np.mean(X[:, split_feature])
    X_left = X[X[:, split_feature] < split_value]
    X_right = X[X[:, split_feature] >= split_value]

    y_left = y[X[:, split_feature] < split_value].flatten()
    y_right = y[X[:, split_feature] >= split_value].flatten()
    return X_left, X_right, y_left, y_right


def confusion_matrix_(y_predicted, y):
    """
    Args:
        y_predicted: predicted number of features
        y: your true labels

    Returns:
        confusion_matrix: with shape (2, 2)

    """
    y = np.array(y)
    confusion_matrix = np.zeros((2, 2))
    # assuming there are only two labels, if it is not one label then must be the other
    confusion_matrix = np.zeros((2, 2))
    confusion_matrix[0][0] = np.count_nonzero((y == 1) & (y_predicted == 1))
    confusion_matrix[0][1] = np.count_nonzero((y == 0) & (y_predicted == 1))
    confusion_matrix[1][0] = np.count_nonzero((y == 1) & (y_predicted == 0))
    confusion_matrix[1][1] = np.count_nonzero((y == 0) & (y_predicted == 0))

    return confusion_matrix


def eval_metrics(conf_matrix):
    """
    Args:
        conf_matrix: Use confusion matrix you calculated

    Returns:
        accuracy, recall, precision, f1_score

    """
    TP = conf_matrix[0][0]
    TN = conf_matrix[1][1]
    FP = conf_matrix[0][1]
    FN = conf_matrix[1][0]

    accuracy = (TN + TP) / (TP + TN + FP + FN)
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)
    f1_score = 2 * precision * recall / (precision + recall)

    return accuracy, recall, precision, f1_score
