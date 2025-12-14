# Linear Separability Analysis of SVM and MLP Classifiers

This report presents solutions to a take-home final exam focused on analyzing the performance of Support Vector Machine (SVM) and Multi-Layer Perceptron (MLP) classifiers on linearly separable and non-separable datasets. The tasks involve generating datasets, implementing the classifiers, and comparing their performance.

## Author

Muhammed Rüşen Birben 150220755

## Dataset Generation

Two datasets were generated:
- D1: Linearly separable data points (100 samples per class)
- D2: Linearly non-separable data points (100 samples per class)

For each dataset, 10 vectors from each class were randomly selected to form the test sets T1 and T2. The remaining vectors were used for training.

## Classifiers

### Hard-Margin SVM on D1

A hard-margin SVM was implemented to classify the linearly separable dataset D1. It achieved perfect classification accuracy on the test set T1.

### Soft-Margin SVM on D2

A soft-margin SVM was implemented to classify the linearly non-separable dataset D2. It allowed for some misclassifications, achieving an accuracy of 85% on the test set T2.

### Two-Layer MLP

A two-layer MLP with 2 hidden layers of 40 neurons each was implemented and trained on both D1 and D2. It achieved 100% accuracy on the test sets for both datasets.

## Results and Discussion

Detailed classification reports are provided comparing the precision, recall, F1 score and support for the SVM and MLP models on the D1 and D2 test sets.

The report concludes that while SVM is effective for linearly separable data, MLP with its ability to learn complex non-linear decision boundaries outperforms SVM on the non-linearly separable dataset.

## Source Code

The full source code for the analysis is provided in a Jupyter notebook linked in the Appendix.

## Requirements

- Python 
- Scikit-learn
- NumPy
- Matplotlib

## Usage

1. Install the required dependencies
2. Run the Jupyter notebook to reproduce the results
