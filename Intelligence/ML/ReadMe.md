

|     |                                      | Training Time [^1]         | Inference |
| --- | ------------------------------------ | -------------------------- | --------- |
|     | Linear Regression (OLS)              |                            | $O(m)$    |
|     | Linear Regression (SGD)              | $O(n_{epoch}nm)$           | $O(m)$    |
|     | Logistic Regression (Binary)         | $O(n_{epoch}nm)$           | $O(m)$    |
|     | Logistic Regression (Multiclass OvR) | $O(n_{epoch}nmc)$          | $O(mc)$   |
|     | Decision Tree                        | $O(n\cdot \log(n)\cdot m)$ | $O(d)$          |
|     | Random Forest Classifier             |$O(n_{trees}\cdot n\cdot \log(n)\cdot m)$| $O(n_{trees}\cdot d)$          |
|     | Support Vector Machines (SVMs)       |  $O(n^{2}m+n^{3})$                         |  $O(n\cdot n_{SV})$         |
|     | k-Nearest Neighbours                 |                            |  $O(nm)$         |
|     | Naive Bayes                          | $O(nm)$                    | $O(mc)$   |
|     | Principal Component Analysis (PCA)   | $O(nm^{2}+m^{3})$          |           |
|     | t_SNE                                | $O(n^{2}m)$                |           |
|     | K-Means Clustering                   |                            |           |

[^1]: n: samples, m: dimensions, n_epoch: epochs, c: classes, d:tree depth,  k: clusters, n_sv: support vectors, i: iterations