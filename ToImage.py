import pandas as pd
from PIL import Image
import numpy as np

RESULT_NAME = "Result.xlsx"

dataFrame = pd.read_excel(RESULT_NAME)

#removing Unneeded Variables 
dataFrame.pop("Var4")
dataFrame.pop("Var5")
dataFrame.pop("Var6")
dataFrame.pop("Var7")
dataFrame.pop("Var8")
dataFrame.pop("Var9")

#format to desired datatype and labels
readings = pd.DataFrame(dataFrame[1:])
readings = readings.rename(columns={'Var1':'Magnitude', 'Var2':'Elevation', 'Var3':'Azimuth'})

ConvertDict = {
    "Magnitude": int,
    "Elevation": int,
    "Azimuth": int
}

readings = readings.astype(ConvertDict)

#sort readings into desired reading order
readings = readings.sort_values(by=['Elevation','Azimuth'], axis=0, ignore_index=True)


print(readings)

#Calculate mean and standard dev
means = np.mean(readings, axis=0)
standardDev = np.std(readings, axis=0)

MagMean = means[0]
MagStd = standardDev[0]

print(means)
print(standardDev)

zScores = []

#calculate the z- score of each reading
for x in readings["Magnitude"]:
    z = (x - MagMean)/MagStd
    zScores.append(float(z))

print(zScores)


# Create Image 
im = Image.new(mode="RGB", size=(200,200), color='red')


#Save Image
im.save("resultingImg.png")

