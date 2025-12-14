# Anime Recommender Systems Project

By: Muhammed Rüşen Birben, 150220755

## Introduction

In this project, I developed a recommender system to suggest animes to users based on their past ratings. My goal was to improve the user experience by providing personalized recommendations. I used python as the programming language.

## Dependencies & Installation

The external python libraries used in this project are:

* numpy (1.23.4)
* pandas (1.3.4)
* scikit-learn (1.0.2)
* scipy (1.8.0)
* matplotlib (3.5.0)
* seaborn (0.12.0)
* yake (0.4.8)
* surprise (1.1.3)
* tqdm (4.62.3)

they can be installed via the following commands:

```shell
pip install numpy
pip install pandas
pip install scikit-learn
pip install scipy
pip install matplotlib
pip install seaborn
pip install yake
pip install surprise
pip install tqdm
```

or if you're using linux:

```shell
pip3 install numpy
pip3 install pandas
pip3 install scikit-learn
pip3 install scipy
pip3 install matplotlib
pip3 install seaborn
pip3 install yake
pip3 install surprise
pip3 install tqdm
```

## Data

Data is found [here](https://www.kaggle.com/datasets/hernan4444/anime-recommendation-database-2020) on Kaggle which has been prepared by scraping the [myanimelist.net](https://myanimelist.net) webiste. It includes ratings of users and data about the animes like their genres, studios etc. I put the 5 csv that's on the Kaggle dataset inside a folder called **data_raw/data/**. I created my main .ipynb file in the same directory as **data_raw** folder.

## Methodology

I used both collaborative and content based approach for implementing this recommender system. I utilized user ratings to implement matrix factorization via Stochastic Gradient Descent algorithm for RMSE. I used sparse matrix and extracted user and item representations. And utilized these extracted latent features of users and items to measure similarity. Also using data (meta data) about animes, I found similarities between them. And combining these two approaches I wrote a hybrid function.

## Evaluation

I evaluated the performance of my recommender system using several metrics, including RMSE, MAE, and MSE. On the test set, I achieved the following results for ratings between 0 to 1:

* RMSE: 0.289
* MAE: 0.213
* MSE: 0.084

## Conclusion

Anime recommender system performed well comparing with similar libraries on python such as *Surprise* and it was also able to process huge loads of data without causing memory problems unlike Surprise. It was able to provide personalized recommendations to users. In future work, I plan to explore other recommendation algorithms to further improve the system's performance.
