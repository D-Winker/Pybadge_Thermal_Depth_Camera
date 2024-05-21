# Calibration
#
# Uses multiple thermal images and multiple depth images
# to calculate per-pixel gain and offset for the two "cameras".
#
# Solves for gain and offset using the Pyomo library.
#
# Optionally, applies the calibration and offset to some sample
# images and displays the original and corrected images side by side.
#
# Daniel Winker, November 10, 2023

import os
import numpy as np
import pandas as pd
from pyomo.environ import *
import matplotlib.pyplot as plt

# Folders containing captured thermal and depth images respectively
# The thermal images are expected to be of a surface with uniform emissivity and 
# with uniform temperature, and the depth data is expected to be of a flat surface 
# parallel to the depth camera.
# The files should be named with the temperature (in degrees C) or distance (in mm)
# respectively.
thermalFolder = "CalibrationData/Thermal"
depthFolder = "CalibrationData/Depth"

# Folders containing example thermal and depth images.
# They will be processed using the calculated calibration values,
# then the before and after images will be shown side by side.
thermalExamples = "Examples/Thermal"
depthExamples = "Examples/Depth"


def listFilesInFolder(pth):
    return [os.path.join(pth, filename) for filename in os.listdir(pth) if (os.path.isfile(os.path.join(pth, filename)) and ".csv" in filename)]


# List all files in the folders
thermalCalFiles = listFilesInFolder(thermalFolder)
depthCalFiles = listFilesInFolder(depthFolder)
thermalExampleFiles = listFilesInFolder(thermalExamples)
depthExampleFiles = listFilesInFolder(depthExamples)

# Read in the data. Each image will get appended to a list. A paralle list will hold 
# the relevant nominal temperature or distance for the image.
thermalData = []
temperatures = []
temperatureGain = np.ones((768))  # One value for each pixel
temperatureOffset = np.zeros((768))
for f in thermalCalFiles:
    tmp = pd.read_csv(f, header=None)
    tmp = tmp.iloc[:, :-1]  # Each line of the file ends with a comma, so drop the last column (all NaNs)
    thermalData.append(tmp.values.ravel())  # Reshape it from a 24x32 matrix to a 768 value vector
    temperatures.append(float(os.path.split(f)[-1].replace(".csv", "")))

depthData = []
distances = []
depthGain = np.ones((64))  # One value for each pixel
depthOffset = np.zeros((64))
for f in depthCalFiles:
    tmp = pd.read_csv(f, header=None)
    tmp = tmp.iloc[:, 0]  # Only keep the first column - the nearest returns
    depthData.append(tmp.values)
    distances.append(float(os.path.split(f)[-1].replace(".csv", "")))


def solve(gainArr, offsetArr, dataList, nominalValueList):
    # Solve for gain and offset for each pixel with Pyomo (written with ChatGPT)
    # Iterate over each pixel
    for pxItr in range(len(gainArr)):
        # Collect all of the data for this specific pixel. (measurement, actual value) pairs.
        data = []
        for image, nom in zip(dataList, nominalValueList):
            data.append((image[pxItr], nom))

        # Create a Concrete Model
        model = ConcreteModel()

        # Define the decision variables
        model.gain = Var()
        model.offset = Var()

        # Define the error as the objective function to minimize
        model.error = Objective(expr=sum(((nominal - (measurement * model.gain + model.offset))**2) for measurement, nominal in data))
        
        # Create a solver
        # ipopt executable can be downloaded from https://github.com/coin-or/Ipopt/releases
        # I had to install via conda, and download the executable and put the path to it below
        solver = SolverFactory('ipopt',executable='C:/Users/17572/anaconda3/envs/3denv/Library/Ipopt-3.14.13-win64-msvs2019-md/bin/ipopt')  # Choose a solver (e.g., 'ipopt')

        # Solve the optimization problem
        results = solver.solve(model)

        # Check the solver status
        if results.solver.status == SolverStatus.ok and results.solver.termination_condition == TerminationCondition.optimal:
            # Get the optimal values of gain and offset
            optimal_gain = model.gain()
            optimal_offset = model.offset()
            print(f"Pixel: {pxItr}, Optimal Gain: {optimal_gain}, Optimal Offset: {optimal_offset}")
            gainArr[pxItr] = optimal_gain
            offsetArr[pxItr] = optimal_offset
        else:
            print("Solver did not converge to an optimal solution.")


if len(thermalData) > 0:
    solve(temperatureGain, temperatureOffset, thermalData, temperatures)

if len(depthData) > 0:
    solve(depthGain, depthOffset, depthData, distances)


def plotSideBySideImages(_suptitle, _title1, _title2, im1, im2):
    # Create a figure with two subplots side by side
    fig, axs = plt.subplots(1, 2, figsize=(10, 4))  # 1 row, 2 columns of subplots

    # Plot the first image on the left subplot
    img1 = axs[0].imshow(im1, cmap='plasma')
    axs[0].set_title(_title1)
    axs[0].axis('off')
    plt.colorbar(img1, ax=axs[0])  # Add a colorbar for the left subplot

    # Plot the second image on the right subplot
    img2 = axs[1].imshow(im2, cmap='plasma')  # You can use a different colormap
    axs[1].set_title(_title2)
    axs[1].axis('off')
    plt.colorbar(img2, ax=axs[1])  # Add a colorbar for the left subplot

    plt.tight_layout()
    plt.suptitle(_suptitle)
    plt.show()


if len(depthData) > 0:
    # Plot images of the gain and offset values
    gainImg = depthGain.reshape((8,8))  # Reshape to be like the physical sensor
    offsetImg = depthOffset.reshape((8,8))  # Reshape to be like the physical sensor
    plotSideBySideImages("Depth Sensor", "Gain", "Offset", np.flipud(gainImg), np.flipud(offsetImg))


if len(thermalData) > 0:
    gainImg = temperatureGain.reshape(24, 32)  # Reshape to be like the physical sensor
    offsetImg = temperatureOffset.reshape(24, 32)  # Reshape to be like the physical 
    plotSideBySideImages("Thermal Camera", "Gain", "Offset", np.flipud(gainImg), np.flipud(offsetImg))


# Apply the calibration and offset to some sample images and display the original and corrected images side by side.


def applyCal(rawData, gainVals, offsetVals):
    newImg = np.zeros(rawData.shape)
    if len(newImg) != len(gainVals) != len(offsetVals):
        print("Error!", len(rawData), len(gainVals), len(offsetVals))
        return
    
    for i in range(len(newImg)):
        newImg[i] = rawData[i] * gainVals[i] + offsetVals[i]
    
    return newImg


for f in thermalExampleFiles:
    raw = pd.read_csv(f, header=None)
    raw = raw.iloc[:, :-1]  # Each line of the file ends with a comma, so drop the last column (all NaNs)
    corr = applyCal(raw.values.ravel(), temperatureGain, temperatureOffset).reshape(24, 32)
    plotSideBySideImages("Thermal Camera Example", "Raw", "Corrected", np.flipud(raw), np.flipud(corr))

for f in depthExampleFiles:
    raw = pd.read_csv(f, header=None)
    raw = raw.iloc[:, 0]  # Only keep the first column - the nearest returns
    corr = applyCal(raw.values.ravel(), depthGain, depthOffset).reshape(8,8).T
    plotSideBySideImages("Depth Sensor Example", "Raw", "Corrected", np.flipud(raw), np.flipud(corr))

print("Operations complete.")
