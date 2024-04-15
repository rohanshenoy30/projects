import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#pandas for handling the dataset in DataFrames.
#numpy for numerical operations.
#matplotlib.pyplot for data visualization.


def main(filetobechecked):
    # Read the dataset given to us
    data = pd.read_csv(filetobechecked)

    # Extract features and target variable
    y = data['SalePrice']
    x1 = data['GrLivArea']
    x2 = data['GarageCars']
    x3 = data['TotalBsmtSF']
    x4 = data['1stFlrSF']
    x5 = data['FullBath']
    x6 = data['TotRmsAbvGrd']
    x7 = data['YearBuilt']
    x8 = data['YearRemodAdd']
    x9 = data['OverallQual']

    #normalisation function to scale values between 0 and 1 to get more accurate and faster convergence in gradient descent
    def min_max_normalisation(values):
        minimum=min(values)
        maximum=max(values)

        if maximum==minimum:
            normalised_val=[0.0 for val in values]
            # to prevent zero division error

        else:
            normalised_val=[(((x-minimum)/(maximum-minimum))*(1-0))+0 for x in values]
            #normalising using the normalisation formula

        return normalised_val


    #gradient descent funtion
    #Gradient Descent: Applies gradient descent individually to each normalized feature to find the slope m and intercept c.
    def gradient_descent(y,x,m,c,learning_rate,iterations): 

        n=len(y) 
        #n=number of data points

        for i in range(iterations):
            dm=0
            dc=0

            for j in range(n):
                # Calculate the predicted value
                y_pred = m * x[j] + c

                # Update the partial derivatives
                dm += (2 / n) * x[j] * (y_pred - y[j])
                dc += (2 / n) * (y_pred - y[j])

            # Update m and c using the learning rate
            m -= learning_rate * dm
            c -= learning_rate * dc

        return m, c


    #function call now begining

    #normalising all the values im the data given to us
    normalisedx1=min_max_normalisation(x1)
    normalisedx2=min_max_normalisation(x2)
    normalisedx3=min_max_normalisation(x3)
    normalisedx4=min_max_normalisation(x4)
    normalisedx5=min_max_normalisation(x5)
    normalisedx6=min_max_normalisation(x6)
    normalisedx7=min_max_normalisation(x7)
    normalisedx8=min_max_normalisation(x8)
    normalisedx9=min_max_normalisation(x9)

    # Initial guesses and hyperparameters
    initial_m = 0.0  # Initial guess for the slopes
    initial_c = 0.0  # Initial guess for the intercept
    learning_rate = 0.0007
    iterations = len(normalisedx1)

    m1, c = gradient_descent(y, normalisedx1, initial_m, initial_c, learning_rate, iterations)
    m2, c = gradient_descent(y, normalisedx2, initial_m, initial_c, learning_rate, iterations)
    m3, c = gradient_descent(y, normalisedx3, initial_m, initial_c, learning_rate, iterations)
    m4, c = gradient_descent(y, normalisedx4, initial_m, initial_c, learning_rate, iterations)
    m5, c = gradient_descent(y, normalisedx5, initial_m, initial_c, learning_rate, iterations)
    m6, c = gradient_descent(y, normalisedx6, initial_m, initial_c, learning_rate, iterations)
    m7, c = gradient_descent(y, normalisedx7, initial_m, initial_c, learning_rate, iterations)
    m8, c = gradient_descent(y, normalisedx8, initial_m, initial_c, learning_rate, iterations)
    m9, c = gradient_descent(y, normalisedx9, initial_m, initial_c, learning_rate, iterations)


    # Convert lists to NumPy arrays

    x1 = np.array(normalisedx1)
    x2 = np.array(normalisedx2)
    x3 = np.array(normalisedx3)
    x4 = np.array(normalisedx4)
    x5 = np.array(normalisedx5)
    x6 = np.array(normalisedx6)
    x7 = np.array(normalisedx7)
    x8 = np.array(normalisedx8)
    x9 = np.array(normalisedx9)
   

    #print(m1*x1)linear regression formlua
    #Predictions: Calculates predictions a by using the obtained coefficients for each feature and sums them up
    a = m1*x1 + m2*x2 + m3*x3 + m4*x5 + m5*x5 + m6*x6 + m7*x7 +m8*x8 + m9*x9 + c 
    print(a) #this is working

    #calculate accuracy using rsquared method
    def accuracy(targets, prediction):
        residual_sum_square=np.sum((targets-prediction)**2)
        total_sum_square=np.sum((targets-np.mean(targets))**2)
        rsquared= 1-(residual_sum_square/total_sum_square)
        return rsquared
                


    # Scatter plot
    # Plots a scatter plot between actual (y) and predicted (a) values and also a black line that represents a perfect fit.
    plt.scatter( y, a, color="red")

    # Plot the regression line
    plt.plot( a,a, color="black")

    # Display the plot
    plt.show()

    print(accuracy(y,a))

              

main("/Users/rohanshenoy/Downloads/train (5).csv")
main("/Users/rohanshenoy/Downloads/test.csv")









        


        
