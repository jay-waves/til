

|            |                                      | Training Time [^1]                        | Inference             |
| ---------- | ------------------------------------ | ----------------------------------------- | --------------------- |
| **线性回归**   | Linear Regression (OLS)              |                                           | $O(m)$                |
|            | Linear Regression (SGD)              | $O(n_{epoch}nm)$                          | $O(m)$                |
| **逻辑回归**   | Logistic Regression (Binary)         | $O(n_{epoch}nm)$                          | $O(m)$                |
|            | Logistic Regression (Multiclass OvR) | $O(n_{epoch}nmc)$                         | $O(mc)$               |
| **决策树**     | Decision Tree                        | $O(n\cdot \log(n)\cdot m)$                | $O(d)$                |
| 随机森林   | Random Forest Classifier             | $O(n_{trees}\cdot n\cdot \log(n)\cdot m)$ | $O(n_{trees}\cdot d)$ |
| 支持向量机 | Support Vector Machines (SVMs)       | $O(n^{2}m+n^{3})$                         | $O(n\cdot n_{SV})$    |
| K-临近     | k-Nearest Neighbours                 |                                           | $O(nm)$               |
| 朴素贝叶斯 | Naive Bayes                          | $O(nm)$                                   | $O(mc)$               |
| 主成分分析           | Principal Component Analysis (PCA)   | $O(nm^{2}+m^{3})$                         |                       |
| T-分布和随机近邻嵌入           | T Distribution and Stochastic Neighbour Embedding (t-SNE)                                | $O(n^{2}m)$                               |                       |
| **K-均值聚类** | K-Means Clustering                   |                                           |                       |
| **神经网络**   | Neural Networks                      |                                           |                       |
| **梯度下降**   | Gradient Descent                                     |                                           |                       |

[^1]: n: samples, m: dimensions, n_epoch: epochs, c: classes, d:tree depth,  k: clusters, n_sv: support vectors, i: iterations